# whether a scenario's replay plays its car like the route's window does, checked on a PC when scenarios are generated
import logging
from collections import Counter
from contextlib import contextmanager
from dataclasses import dataclass, replace
from functools import partial

from opendbc.can.dbc import DBC
from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.safety.tests.libsafety import libsafety_py

from openpilot.cereal import log
from openpilot.cereal.services import SERVICE_LIST
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.car.cruise import V_CRUISE_INITIAL, VCruiseHelper
from openpilot.selfdrive.selfdrived.events import EVENT_NAME, EVENTS, ET, Events
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.selfdrive.test.cars.replay import (DT, Frame, _dbcs, build_replay, fingerprint_problems, needed_frames, over_budget, packets,
                                                  radar_problems, window_frames)
from openpilot.selfdrive.test.cars.rules import ENGAGE_LIMIT, signal_values
from openpilot.selfdrive.test.cars.scenario import MIN_ENGAGED, TAIL, RouteCar, Scenario, Window, build_candidates, first_can

EventName = log.OnroadEvent.EventName
ButtonType = structs.CarState.ButtonEvent.Type

# the UP loops the window, so a loop starts from the state the previous one left the panda, the parsers and openpilot in.
# the first loop warms up the parsers and openpilot from nothing, the second engages as a station's previous loop did, the last is judged
LOOPS = 3
# the panda calls safety_tick and its heartbeat check at 1 Hz
TICK = round(1. / DT)
# frames a carState change may move by on the replay
SLACK = 2
# the rules see the panda's edge after a pandaStates of it disallowed, pandad publishes one every this many frames
PANDA_STATES = round(1. / (SERVICE_LIST['pandaStates'].frequency * DT))
# the carState fields that block or trigger an enable in card and selfdrived
FIELDS = (
  'canValid', 'canTimeout', 'gearShifter', 'seatbeltUnlatched', 'doorOpen', 'brakePressed', 'gasPressed', 'regenBraking',
  'standstill', 'parkingBrake', 'brakeHoldActive', 'espDisabled', 'accFaulted', 'carNotReady', 'steerFaultTemporary',
  'steerFaultPermanent', 'cruiseState.available', 'cruiseState.enabled', 'cruiseState.nonAdaptive', 'cruiseState.standstill',
)


def _field(CS: structs.CarState, name: str):
  value = CS
  for part in name.split('.'):
    value = getattr(value, part)
  return value


@contextmanager
def _quiet_carlog():
  # the parsers log a bus timeout on every gap of the thinned frames
  carlog = logging.getLogger("carlog")
  level, carlog.level = carlog.level, logging.CRITICAL
  try:
    yield
  finally:
    carlog.setLevel(level)


def _looped(frames: list[Frame]):
  # (loop, time, packet) of each frame of LOOPS loops
  loop = round(len(frames) * DT * 1e9)
  for n in range(LOOPS):
    for t, packet in packets(frames):
      yield n, t + n * loop, packet


def car_states(CP: structs.CarParams, frames: list[Frame]) -> list[structs.CarState]:
  # the DUT's carState each frame of LOOPS loops of the frames, as card publishes it
  car = interfaces[CP.carFingerprint](CP)
  with _quiet_carlog():
    return [car.update([(t, packet)]) for _, t, packet in _looped(frames)]


def controller_idle(CP: structs.CarParams, frames: list[Frame], enabled: list[bool]) -> int | None:
  # the longest run of cycles the DUT's car controller sends nothing between two commands while openpilot is engaged in the
  # last loop, its command cadence on the replay. None without two commands
  car = interfaces[CP.carFingerprint](CP)
  longest = None
  run = None
  with _quiet_carlog():
    for i, (n, t, packet) in enumerate(_looped(frames)):
      CS = car.update([(t, packet)])
      on = enabled[i]
      # as controlsd sets them
      standstill = abs(CS.vEgo) <= max(CP.minSteerSpeed, 0.3) or CS.standstill
      lat = on and not CS.steerFaultTemporary and not CS.steerFaultPermanent and (not standstill or CP.steerAtStandstill)
      # card passes the carControl it reads
      CC = structs.CarControl(enabled=on, latActive=lat, longActive=on and CP.openpilotLongitudinalControl).as_reader()
      _, sends = car.apply(CC, t)
      if n < LOOPS - 1 or not on:
        run = None
      elif sends:
        if run is not None:
          longest = max(longest or 0, run)
        run = 0
      elif run is not None:
        run += 1
  return longest


@dataclass
class Step:
  enabled: bool
  # an enable event and the events that keep it from enabling
  enable: bool
  no_entry: list[str]


def selfdrive(CP: structs.CarParams, states: list[structs.CarState]) -> list[Step]:
  # the part of card and selfdrived that engages on carState: car events, set speed, pedals and CAN errors through the state machine
  car_events = CarEvents(CP)
  machine = StateMachine()
  v_cruise = VCruiseHelper(CP)
  prev = structs.CarState()
  enabled = False
  steps = []
  for CS in states:
    v_cruise.update_v_cruise(CS, enabled, False)
    # the car events that disengage depend on whether openpilot is engaged
    CC = structs.CarControl(enabled=enabled, latActive=enabled, longActive=enabled and CP.openpilotLongitudinalControl)
    events = Events()
    if CS.canValid:
      events = car_events.update(CS, prev, CC)
      # openpilot's default doesn't disengage on the accelerator
      if (CS.brakePressed and (not prev.brakePressed or not CS.standstill)) or \
         (CS.regenBraking and (not prev.regenBraking or not CS.standstill)):
        events.add(EventName.pedalPressed)
    resume = any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents)
    if not CP.pcmCruise and v_cruise.v_cruise_kph > 250 and resume:
      events.add(EventName.resumeBlocked)
    if CS.canTimeout:
      events.add(EventName.canBusMissing)
    elif not CS.canValid:
      events.add(EventName.canError)
    was_enabled = enabled
    enabled, _ = machine.update(events)
    if enabled and not was_enabled:
      v_cruise.initialize_v_cruise(prev, V_CRUISE_INITIAL)
    no_entry = [EVENT_NAME[e] for e in events.names if ET.NO_ENTRY in EVENTS.get(e, {})]
    steps.append(Step(enabled, events.contains(ET.ENABLE), no_entry))
    prev = CS
  return steps


def panda_allowed(mode: int, param: int, frames: list[Frame], enabled: list[bool],
                  lost: tuple[int, int, int] | None = None) -> tuple[list[bool], dict[int, tuple[int, int]]]:
  # the panda's controls_allowed each frame of LOOPS loops, with openpilot's engaged state in its heartbeat, and the frames
  # a message's rx drops controls at -> its (bus, addr). lost: (frame, bus, addr) of a message's frame the station's lane loses
  safety = libsafety_py.libsafety
  if safety.set_safety_hooks(mode, param) != 0:
    raise ValueError(f"libsafety has no safety mode {mode} param {param}")
  # as card sets it
  safety.set_alternative_experience(0)
  allowed = []
  drops = {}
  for i in range(LOOPS * len(frames)):
    safety.set_timer(round(i * DT * 1e6))
    for addr, dat, bus in frames[i % len(frames)]:
      if (i, bus, addr) == lost:
        continue
      was = safety.get_controls_allowed()
      safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
      if was and not safety.get_controls_allowed():
        drops[i] = (bus, addr)
    if i % TICK == 0:
      safety.safety_heartbeat(enabled[i])
      safety.safety_tick()
      safety.safety_heartbeat_check()
    allowed.append(bool(safety.get_controls_allowed()))
  return allowed, drops


def logged_controls(msgs: list, window: Window) -> list | None:
  # the route's carControl each frame of the window, the latest logged at or before it, None without any
  first = first_can(msgs)
  n = round((window.end - window.start) / DT)
  controls: list = [None] * n
  for m in msgs:
    if m.which() != 'carControl':
      continue
    i = int(((m.logMonoTime - first) / 1e9 - window.start) // DT)
    if i < 0:
      controls[0] = m.carControl
    elif i < n:
      controls[i] = m.carControl
  if all(c is None for c in controls):
    return None
  for i in range(1, n):
    if controls[i] is None:
      controls[i] = controls[i - 1]
  return controls


# (frame of the last loop, bus, addr) of a command libsafety blocks
Block = tuple[int, int, int]


def tx_blocks(CP: structs.CarParams, mode: int, param: int, frames: list[Frame], controls: list) -> list[Block]:
  # the commands the DUT's car controller makes from the route's carControl that libsafety blocks in the last of LOOPS loops,
  # while openpilot is engaged and stays engaged the next frame as the rules judge them. the panda takes each frame's rx,
  # then card updates carState from it and sends what it makes of the frame's carControl
  safety = libsafety_py.libsafety
  if safety.set_safety_hooks(mode, param) != 0:
    raise ValueError(f"libsafety has no safety mode {mode} param {param}")
  safety.set_alternative_experience(0)
  car = interfaces[CP.carFingerprint](CP)
  n = len(frames)
  blocks = []
  with _quiet_carlog():
    for i, (loop, t, packet) in enumerate(_looped(frames)):
      j = i % n
      safety.set_timer(round(i * DT * 1e6))
      for addr, dat, bus in frames[j]:
        safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
      CC = controls[j]
      enabled = CC is not None and CC.enabled
      if i % TICK == 0:
        safety.safety_heartbeat(enabled)
        safety.safety_tick()
        safety.safety_heartbeat_check()
      car.update([(t, packet)])
      if CC is None:
        continue
      _, sends = car.apply(CC, t)
      judged = loop == LOOPS - 1 and enabled and j + 1 < n and controls[j + 1].enabled
      for addr, dat, bus in sends:
        allowed = safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
        if judged and not allowed:
          blocks.append((j, bus, addr))
  return blocks


def extra_blocks(route: list[Block], replay: list[Block]) -> list[Block]:
  # the replay's blocks with none of the same message within SLACK frames on the route's window
  return [b for b in replay if not any(r[1:] == b[1:] and abs(r[0] - b[0]) <= SLACK for r in route)]


def _blocks(blocks: list[Block]) -> str:
  per: dict[tuple[int, int], list[int]] = {}
  for j, bus, addr in blocks:
    per.setdefault((bus, addr), []).append(j)
  return "; ".join(f"{addr:#x} bus {bus} at {_times(frames[:4])}" + (f" (+{len(frames) - 4})" if len(frames) > 4 else "")
                   for (bus, addr), frames in sorted(per.items()))


@dataclass
class Engage:
  # the last loop, in frames from its start: the panda's engage edges and the frame openpilot is enabled at each, None if it isn't
  rises: list[int]
  enabled_at: list[int | None]
  # why openpilot doesn't enable at an edge
  blocked: list[str]
  allowed_at_start: bool
  # openpilot stays engaged from the loop's start through its first edge
  carried: bool
  # the first frame the panda doesn't allow controls with openpilot engaged longer than the engage limit
  dropped: int | None
  # the panda's controls_allowed and openpilot's enabled each frame of the last loop up to its end
  allowed: list[bool]
  enabled: list[bool]
  # openpilot's enabled each frame of every loop
  looped_enabled: list[bool]
  # frames the panda is disallowed before its first edge, and whether its 1 Hz tick or heartbeat check dropped it, which a
  # station's panda does up to a tick later as its ticks aren't in phase with the loop
  disallowed: int | None = None
  tick_drop: bool = False
  # the frame of every loop the panda drops controls at before its first edge, and the (bus, addr) whose rx drops them
  drop: int | None = None
  dropper: tuple[int, int] | None = None
  # the panda still drops controls in time if the station's lane loses the frame that drops them
  robust: bool = True


def _need(tick_drop: bool) -> int:
  # frames the panda has to be disallowed before its edge for a pandaStates to see it
  return PANDA_STATES + SLACK + (TICK if tick_drop else 0)


def _drop_before(allowed: list[bool], drops: dict[int, tuple[int, int]], rise: int) -> tuple[int, int, tuple[int, int] | None, bool]:
  # the frame the panda drops controls at before the edge, its frames disallowed, the message that drops them and whether its tick does
  drop = next((i + 1 for i in range(rise - 1, -1, -1) if allowed[i]), 0)
  dropper = drops.get(drop)
  return drop, rise - drop, dropper, dropper is None and drop > 0


def engage(CP: structs.CarParams, mode: int, param: int, frames: list[Frame], end: float,
           states: list[structs.CarState] | None = None) -> Engage:
  # the last loop as the station plays it up to end s: the panda's engage edges and openpilot's enable at each
  n = len(frames)
  start = (LOOPS - 1) * n
  states = car_states(CP, frames) if states is None else states
  steps = selfdrive(CP, states)
  allowed, drops = panda_allowed(mode, param, frames, [s.enabled for s in steps])
  last = start + min(round(end / DT), n - 1)
  limit = round(ENGAGE_LIMIT / DT)
  rises = [i for i in range(start, last + 1) if allowed[i] and not allowed[i - 1]]
  enabled_at = []
  blocked = []
  for rise in rises:
    # the rules count openpilot enabled at the edge, it may enable on the same message a little before the panda
    at = next((i for i in range(rise, min(rise + limit, last) + 1) if steps[i].enabled), None)
    enabled_at.append(None if at is None else at - start)
    if at is None:
      window = steps[rise:rise + limit + 1]
      names = sorted({name for s in window if s.enable for name in s.no_entry})
      if not names and not any(s.enable for s in window):
        names = ["no enable event"]
      blocked.append(f"{', '.join(names)} at {(rise - start) * DT:.2f} s")
  first = rises[0] if rises else last
  carried = all(s.enabled for s in steps[start:first + 1])
  dropped = None
  since = None
  for i in range(first, last + 1):
    since = None if not steps[i].enabled else (i if since is None else since)
    if since is not None and not allowed[i] and i - since > limit:
      dropped = i - start
      break
  drop = disallowed = dropper = None
  tick_drop = False
  robust = True
  if rises:
    drop, disallowed, dropper, tick_drop = _drop_before(allowed, drops, rises[0])
    if dropper is not None:
      # the station's lane may lose a frame, the next frames of the message have to drop controls in time too
      lost, lost_drops = panda_allowed(mode, param, frames, [s.enabled for s in steps], (drop, *dropper))
      _, lost_disallowed, _, lost_tick = _drop_before(lost, lost_drops, rises[0])
      robust = lost[rises[0]] and lost_disallowed >= _need(lost_tick)
  looped_enabled = [s.enabled for s in steps]
  return Engage([r - start for r in rises], enabled_at, blocked, allowed[start], carried, dropped, allowed[start:last + 1],
                looped_enabled[start:last + 1], looped_enabled, disallowed, tick_drop, drop, dropper, robust)


def engage_problems(e: Engage, end: float) -> list[str]:
  # what keeps the last loop from a panda engage edge that openpilot engages on and stays engaged with
  if not e.rises:
    if e.allowed_at_start:
      return ["the panda stays allowed across the loop's wrap, nothing before the edge drops it"]
    return ["the panda never allows controls"]
  problems = []
  if end - e.rises[0] * DT < MIN_ENGAGED:
    problems.append(f"the panda's edge at {e.rises[0] * DT:.2f} s leaves less than {MIN_ENGAGED:g} s engaged")
  need = _need(e.tick_drop)
  if e.disallowed is not None and e.disallowed < need:
    how = "its tick drops controls" if e.tick_drop else "it drops controls"
    problems.append(f"the panda's edge at {e.rises[0] * DT:.2f} s comes {e.disallowed * DT:.2f} s after {how}, less than {need * DT:.2f} s")
  elif not e.robust and e.dropper is not None:
    bus, addr = e.dropper
    problems.append(f"the panda drops controls before its edge on one frame of {addr:#x} bus {bus}, not in time if the station loses it")
  if e.carried:
    problems.append("openpilot stays engaged across the loop's wrap")
  elif e.blocked:
    problems.append(f"openpilot doesn't engage at the panda's edge: {'; '.join(e.blocked)}")
  if e.dropped is not None:
    problems.append(f"the panda drops controls at {e.dropped * DT:.2f} s with openpilot engaged")
  return problems


def _changes(values: list) -> list[tuple[int, object]]:
  return [(i, v) for i, v in enumerate(values) if i == 0 or v != values[i - 1]]


def _buttons(states: list[structs.CarState]) -> list[tuple[int, str, bool]]:
  return [(i, str(b.type), b.pressed) for i, CS in enumerate(states) for b in CS.buttonEvents]


def _matches(a: list[tuple], b: list[tuple]) -> bool:
  # the same changes in order, each within SLACK frames
  return len(a) == len(b) and all(x[1:] == y[1:] and abs(x[0] - y[0]) <= SLACK for x, y in zip(a, b, strict=True))


def _describe(changes: list[tuple]) -> str:
  shown = ", ".join(f"{'/'.join(str(v) for v in c[1:])} at {c[0] * DT:.2f} s" for c in changes[:4])
  return shown + (f" (+{len(changes) - 4})" if len(changes) > 4 else "") if changes else "none"


def diverging(route: list[structs.CarState], replay: list[structs.CarState]) -> dict[str, str]:
  # field -> how the replay's last loop differs from the route window's, for the fields that engage or block openpilot
  n = len(route) // LOOPS
  route, replay = route[-n:], replay[-n:]
  out = {}
  for name in FIELDS:
    a = _changes([_field(CS, name) for CS in route])
    b = _changes([_field(CS, name) for CS in replay])
    if not _matches(a, b):
      out[name] = f"route {_describe(a)}, replay {_describe(b)}"
  a = _buttons(route)
  b = _buttons(replay)
  if not _matches(a, b):
    out['buttonEvents'] = f"route {_describe(a)}, replay {_describe(b)}"
  return out


@dataclass
class Fidelity:
  # what keeps the route's window, as the station loops it, from engaging at its edge
  window: list[str]
  # what keeps the replay from engaging like the window
  replay: list[str]
  # the fields the replay changes, field -> how
  fields: dict[str, str]
  # frames after the panda's edge that openpilot engages on the replay
  delay: int | None
  # the DUT controller's longest run of cycles without a command while engaged on the replay
  idle: int | None = None
  # the commands libsafety blocks while engaged, the DUT's car controller driven by the route's carControl on the route's
  # window and on the replay, None without a logged carControl
  route_tx: list[Block] | None = None
  replay_tx: list[Block] | None = None
  # what drops the panda's controls before its edge on the replay
  drop: str = ""

  @property
  def problems(self) -> list[str]:
    # changed fields only count when they change how the replay engages, which the replay's own problems tell
    if self.window:
      return [f"the route's window: {p}" for p in self.window]
    replay = list(self.replay)
    if self.route_tx is not None and self.replay_tx is not None:
      extra = extra_blocks(self.route_tx, self.replay_tx)
      if extra:
        replay.append(f"libsafety blocks the DUT's commands from the route's carControl the route's window doesn't: {_blocks(extra)}")
    if not replay:
      return []
    return [f"the replay: {p}" for p in replay] + [f"the replay changes {name}: {how}" for name, how in self.fields.items()]

  def summary(self) -> str:
    delay = f", openpilot engages {self.delay * DT:.2f} s after the panda's edge" if self.delay is not None else ""
    if self.fields:
      changed = "; ".join(f"{name}: {how}" for name, how in self.fields.items())
      line = f"fidelity: the panda and openpilot engage as on the route{delay}, the replay changes {changed}"
    else:
      line = f"fidelity: carState and the panda's engage edge match the route{delay}"
    drop = f", {self.drop}" if self.drop else ""
    return f"{line}{drop}. {self.tx_summary()}"

  def tx_summary(self) -> str:
    if self.route_tx is None or self.replay_tx is None:
      return "TX: route logged no carControl, not checked"
    if not self.route_tx and not self.replay_tx:
      return "TX: libsafety allows every command the DUT's controller makes from the route's carControl"
    lines = []
    if self.route_tx:
      lines.append(f"TX: libsafety blocks the DUT's commands from the route's carControl on the route's window too: {_blocks(self.route_tx)}")
    extra = extra_blocks(self.route_tx, self.replay_tx)
    if extra:
      lines.append(f"TX: only on the replay: {_blocks(extra)}")
    return ". ".join(lines) if lines else "TX: the replay blocks as the route's window does"


def drop_cause(CP: structs.CarParams, frames: list[Frame], e: Engage) -> str:
  # what drops the panda's controls before its edge: the message and its signals that change at the frame that drops them
  if e.drop is None or e.disallowed is None:
    return ""
  when = f"the panda drops controls {e.disallowed * DT:.2f} s before its edge"
  if e.dropper is None:
    return f"{when} on its tick" if e.tick_drop else ""
  bus, addr = e.dropper
  n = len(frames)
  seen = [dat for i in range(e.drop + 1) for a, dat, b in frames[i % n] if (b, a) == e.dropper]
  dbc = _dbcs(CP).get(e.dropper)
  if dbc is None or len(seen) < 2:
    return f"{when} on {addr:#x} bus {bus}"
  msg = DBC(dbc).addr_to_msg[addr]
  before = signal_values(msg, seen[-2])
  now = signal_values(msg, seen[-1])
  changed = ", ".join(f"{name} {before[name]:g} → {value:g}" for name, value in now.items() if value != before[name])
  return f"{when} on {msg.name} {addr:#x} bus {bus} ({changed or 'no signal changes'})"


def _times(frames: list[int]) -> str:
  return ", ".join(f"{i * DT:.2f} s" for i in frames) or "none"


def fidelity(CP: structs.CarParams, mode: int, param: int, window: list[Frame], replay: list[Frame], end: float,
             controls: list | None, route_states: list[structs.CarState] | None = None, route_tx: list[Block] | None = None) -> Fidelity:
  # the replay against the route's window it was thinned from, both looped as the station plays them, end s from the window's start.
  # controls: the route's carControl each frame of the window
  route_states = car_states(CP, window) if route_states is None else route_states
  replay_states = car_states(CP, replay)
  route = engage(CP, mode, param, window, end, route_states)
  played = engage(CP, mode, param, replay, end, replay_states)
  window_problems = engage_problems(route, end)
  played_problems = engage_problems(played, end)
  if not window_problems and not played_problems:
    for name, a, b in (("the panda's controls_allowed", route.allowed, played.allowed), ("openpilot's enabled", route.enabled, played.enabled)):
      route_changes = _changes(a)
      replay_changes = _changes(b)
      if not _matches(route_changes, replay_changes):
        played_problems.append(f"{name}: route {_describe(route_changes)}, replay {_describe(replay_changes)}")
  delay = None
  if played.rises and played.enabled_at[0] is not None:
    delay = played.enabled_at[0] - played.rises[0]
  idle = controller_idle(CP, replay, played.looped_enabled)
  replay_tx = None
  if controls is not None:
    route_tx = tx_blocks(CP, mode, param, window, controls) if route_tx is None else route_tx
    replay_tx = tx_blocks(CP, mode, param, replay, controls)
  return Fidelity(window_problems, played_problems, diverging(route_states, replay_states), delay, idle, route_tx, replay_tx,
                  drop_cause(CP, replay, played))


def _keys(keys) -> str:
  return ", ".join(f"{addr:#x} bus {bus}" for bus, addr in keys)


def replay_problems(s: Scenario, car: RouteCar, msgs: list, budget: dict[int, float]) -> tuple[list[str], Fidelity | None]:
  # what keeps a scenario's replay from playing its car on a station, and its fidelity once the rest plays
  window = window_frames(msgs, s.window)
  replay = build_replay(s, car, msgs, budget).frames
  problems = fingerprint_problems(car, replay)
  problems += [f"lane budget: {p}" for p in over_budget(replay, budget)]
  problems += radar_problems(car.dut_CP, window, replay)
  if problems:
    return problems, None
  f = fidelity(car.dut_CP, s.mode, s.param, window, replay, s.window.end - TAIL - s.window.start, logged_controls(msgs, s.window))
  return f.problems, f


def faithful_messages(s: Scenario, car: RouteCar, msgs: list, budget: dict[int, float]) -> list[tuple[int, int]] | None:
  # the fewest of the messages thinning drops frames of whose every frame makes the replay engage like the route's window,
  # the others thinned as they are, None if all of them every frame doesn't
  CP = car.dut_CP
  window = window_frames(msgs, s.window)
  end = s.window.end - TAIL - s.window.start
  route_states = car_states(CP, window)
  controls = logged_controls(msgs, s.window)
  route_tx = tx_blocks(CP, s.mode, s.param, window, controls) if controls is not None else None
  needed = needed_frames(CP, s.rx_checks, window)
  replay = build_replay(s, car, msgs, budget).frames
  full = Counter((bus, addr) for frame in needed for addr, _, bus in frame)
  played = Counter((bus, addr) for frame in replay for addr, _, bus in frame)
  # the busiest first, they cost the most budget
  thinned = sorted((key for key in full if played[key] < full[key]), key=lambda key: -full[key])

  def faithful(keys: set[tuple[int, int]]) -> bool:
    mixed = [[f for f in thin_frame if (f[2], f[0]) not in keys] + [f for f in full_frame if (f[2], f[0]) in keys]
             for thin_frame, full_frame in zip(replay, needed, strict=True)]
    f = fidelity(CP, s.mode, s.param, window, mixed, end, controls, route_states, route_tx)
    return not f.problems and not f.fields

  if not faithful(set(thinned)):
    return None
  keep = set(thinned)
  for key in thinned:
    if faithful(keep - {key}):
      keep.discard(key)
  return sorted(keep)


def _cadence(s: Scenario, f: Fidelity | None) -> Scenario:
  # the route logged an older controller's cadence, the DUT runs the current one, the longer of the two.
  # and the fidelity line the rack prints
  if f is None:
    return s
  summary = f.summary()
  if s.kept:
    summary += f", keeps {', '.join(f'{addr:#x} bus {bus} every {k}' for bus, addr, k in s.kept)} frames"
  max_idle = s.max_idle if f.idle is None else max(s.max_idle, f.idle)
  return replace(s, max_idle=max_idle, fidelity=summary)


def check_scenario(s: Scenario, car: RouteCar, msgs: list, budget: dict[int, float]) -> tuple[Scenario, list[str]]:
  # the scenario, with the messages its replay keeps more frames of where thinning changes carState, and its problems.
  # changed fields that don't change how the replay engages only cost the scenario if keeping their messages doesn't fit
  problems, f = replay_problems(s, car, msgs, budget)
  if f is None or f.window or not (problems or f.fields):
    return _cadence(s, f), problems
  keys = faithful_messages(s, car, msgs, budget)
  if keys is None:
    if not problems:
      return _cadence(s, f), []
    return s, problems + ["keeping every frame of the messages thinning drops doesn't help"]
  # the fewest frames first, every k-th frame moves a change by at most k - 1 frames
  first_problems = None
  # the first that plays, though thinning the rest harder changes a field
  plays = None
  for k in range(SLACK + 1, 0, -1):
    kept = replace(s, kept=tuple((bus, addr, k) for bus, addr in keys))
    kept_problems, kept_f = replay_problems(kept, car, msgs, budget)
    if not kept_problems and kept_f is not None and not kept_f.fields:
      return _cadence(kept, kept_f), []
    if not kept_problems and kept_f is not None and plays is None:
      plays = (kept, kept_f)
    first_problems = kept_problems if first_problems is None else first_problems
  if not problems:
    return _cadence(s, f), []
  if plays is not None:
    return _cadence(*plays), []
  return s, problems + [f"keeping {_keys(keys)} every {SLACK + 1} frames: {', '.join(first_problems or ['carState still differs'])}"]


def pick_scenarios(routes, budget: dict[int, float]) -> tuple[list[Scenario], list[str]]:
  # what a rack run drives: each pair's first scenario whose replay plays on a station, and the gaps
  candidates, gaps = build_candidates(routes, partial(check_scenario, budget=budget))
  return [pair_candidates[0] for pair_candidates in candidates.values()], gaps
