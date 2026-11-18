# whether a scenario's replay plays its car like the route's window does, checked on a PC when scenarios are generated
import logging
from collections import Counter
from dataclasses import dataclass, replace
from functools import partial

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.safety.tests.libsafety import libsafety_py

from openpilot.cereal import log
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.car.cruise import V_CRUISE_INITIAL, VCruiseHelper
from openpilot.selfdrive.selfdrived.events import EVENT_NAME, EVENTS, ET, Events
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.selfdrive.test.cars.replay import (DT, Frame, build_replay, fingerprint_problems, needed_frames, over_budget, packets,
                                                  radar_problems, window_frames)
from openpilot.selfdrive.test.cars.rules import ENGAGE_LIMIT
from openpilot.selfdrive.test.cars.scenario import MIN_ENGAGED, TAIL, RouteCar, Scenario, build_candidates

EventName = log.OnroadEvent.EventName
ButtonType = structs.CarState.ButtonEvent.Type

# the UP loops the window, so the second loop starts from the state the first left the panda, the parsers and openpilot in
LOOPS = 2
# the panda calls safety_tick and its heartbeat check at 1 Hz
TICK = round(1. / DT)
# frames a carState change may move by on the replay
SLACK = 2
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


def car_states(CP: structs.CarParams, frames: list[Frame]) -> list[structs.CarState]:
  # the DUT's carState each frame of LOOPS loops of the frames, as card publishes it
  car = interfaces[CP.carFingerprint](CP)
  carlog = logging.getLogger("carlog")
  level, carlog.level = carlog.level, logging.CRITICAL
  loop = round(len(frames) * DT * 1e9)
  states = []
  try:
    for n in range(LOOPS):
      for t, packet in packets(frames):
        states.append(car.update([(t + n * loop, packet)]))
  finally:
    carlog.setLevel(level)
  return states


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


def panda_allowed(mode: int, param: int, frames: list[Frame], enabled: list[bool]) -> list[bool]:
  # the panda's controls_allowed each frame of LOOPS loops, with openpilot's engaged state in its heartbeat
  safety = libsafety_py.libsafety
  if safety.set_safety_hooks(mode, param) != 0:
    raise ValueError(f"libsafety has no safety mode {mode} param {param}")
  # as card sets it
  safety.set_alternative_experience(0)
  allowed = []
  for i in range(LOOPS * len(frames)):
    safety.set_timer(round(i * DT * 1e6))
    for addr, dat, bus in frames[i % len(frames)]:
      safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
    if i % TICK == 0:
      safety.safety_heartbeat(enabled[i])
      safety.safety_tick()
      safety.safety_heartbeat_check()
    allowed.append(bool(safety.get_controls_allowed()))
  return allowed


@dataclass
class Engage:
  # the second loop, in frames from its start: the panda's engage edges and the frame openpilot is enabled at each, None if it isn't
  rises: list[int]
  enabled_at: list[int | None]
  # why openpilot doesn't enable at an edge
  blocked: list[str]
  allowed_at_start: bool
  # openpilot stays engaged from the loop's start through its first edge
  carried: bool
  # the first frame the panda doesn't allow controls with openpilot engaged longer than the engage limit
  dropped: int | None
  # the panda's controls_allowed and openpilot's enabled each frame of the second loop up to its end
  allowed: list[bool]
  enabled: list[bool]


def engage(CP: structs.CarParams, mode: int, param: int, frames: list[Frame], end: float,
           states: list[structs.CarState] | None = None) -> Engage:
  # the second loop as the station plays it up to end s: the panda's engage edges and openpilot's enable at each
  n = len(frames)
  states = car_states(CP, frames) if states is None else states
  steps = selfdrive(CP, states)
  allowed = panda_allowed(mode, param, frames, [s.enabled for s in steps])
  last = n + min(round(end / DT), n - 1)
  limit = round(ENGAGE_LIMIT / DT)
  rises = [i for i in range(n, last + 1) if allowed[i] and not allowed[i - 1]]
  enabled_at = []
  blocked = []
  for rise in rises:
    # the rules count openpilot enabled at the edge, it may enable on the same message a little before the panda
    at = next((i for i in range(rise, min(rise + limit, last) + 1) if steps[i].enabled), None)
    enabled_at.append(None if at is None else at - n)
    if at is None:
      window = steps[rise:rise + limit + 1]
      names = sorted({name for s in window if s.enable for name in s.no_entry})
      if not names and not any(s.enable for s in window):
        names = ["no enable event"]
      blocked.append(f"{', '.join(names)} at {(rise - n) * DT:.2f} s")
  first = rises[0] if rises else last
  carried = all(s.enabled for s in steps[n:first + 1])
  dropped = None
  since = None
  for i in range(first, last + 1):
    since = None if not steps[i].enabled else (i if since is None else since)
    if since is not None and not allowed[i] and i - since > limit:
      dropped = i - n
      break
  return Engage([r - n for r in rises], enabled_at, blocked, allowed[n], carried, dropped, allowed[n:last + 1],
                [s.enabled for s in steps[n:last + 1]])


def engage_problems(e: Engage, end: float) -> list[str]:
  # what keeps the second loop from a panda engage edge that openpilot engages on and stays engaged with
  if not e.rises:
    if e.allowed_at_start:
      return ["the panda stays allowed across the loop's wrap, nothing before the edge drops it"]
    return ["the panda never allows controls"]
  problems = []
  if end - e.rises[0] * DT < MIN_ENGAGED:
    problems.append(f"the panda's edge at {e.rises[0] * DT:.2f} s leaves less than {MIN_ENGAGED:g} s engaged")
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
  # field -> how the replay's second loop differs from the route window's, for the fields that engage or block openpilot
  n = len(route) // LOOPS
  route, replay = route[n:], replay[n:]
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

  @property
  def problems(self) -> list[str]:
    # changed fields only count when they change how the replay engages, which the replay's own problems tell
    if self.window:
      return [f"the route's window: {p}" for p in self.window]
    if not self.replay:
      return []
    return [f"the replay: {p}" for p in self.replay] + [f"the replay changes {name}: {how}" for name, how in self.fields.items()]

  def summary(self) -> str:
    delay = f", openpilot engages {self.delay * DT:.2f} s after the panda's edge" if self.delay is not None else ""
    if self.fields:
      changed = "; ".join(f"{name}: {how}" for name, how in self.fields.items())
      return f"fidelity: the panda and openpilot engage as on the route{delay}, the replay changes {changed}"
    return f"fidelity: carState and the panda's engage edge match the route{delay}"


def _times(frames: list[int]) -> str:
  return ", ".join(f"{i * DT:.2f} s" for i in frames) or "none"


def fidelity(CP: structs.CarParams, mode: int, param: int, window: list[Frame], replay: list[Frame], end: float,
             route_states: list[structs.CarState] | None = None) -> Fidelity:
  # the replay against the route's window it was thinned from, both looped as the station plays them, end s from the window's start
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
  return Fidelity(window_problems, played_problems, diverging(route_states, replay_states), delay)


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
  f = fidelity(car.dut_CP, s.mode, s.param, window, replay, s.window.end - TAIL - s.window.start)
  return f.problems, f


def faithful_messages(s: Scenario, car: RouteCar, msgs: list, budget: dict[int, float]) -> list[tuple[int, int]] | None:
  # the fewest of the messages thinning drops frames of whose every frame makes the replay engage like the route's window,
  # the others thinned as they are, None if all of them every frame doesn't
  CP = car.dut_CP
  window = window_frames(msgs, s.window)
  end = s.window.end - TAIL - s.window.start
  route_states = car_states(CP, window)
  needed = needed_frames(CP, s.rx_checks, window)
  replay = build_replay(s, car, msgs, budget).frames
  full = Counter((bus, addr) for frame in needed for addr, _, bus in frame)
  played = Counter((bus, addr) for frame in replay for addr, _, bus in frame)
  # the busiest first, they cost the most budget
  thinned = sorted((key for key in full if played[key] < full[key]), key=lambda key: -full[key])

  def faithful(keys: set[tuple[int, int]]) -> bool:
    mixed = [[f for f in thin_frame if (f[2], f[0]) not in keys] + [f for f in full_frame if (f[2], f[0]) in keys]
             for thin_frame, full_frame in zip(replay, needed, strict=True)]
    f = fidelity(CP, s.mode, s.param, window, mixed, end, route_states)
    return not f.problems and not f.fields

  if not faithful(set(thinned)):
    return None
  keep = set(thinned)
  for key in thinned:
    if faithful(keep - {key}):
      keep.discard(key)
  return sorted(keep)


def check_scenario(s: Scenario, car: RouteCar, msgs: list, budget: dict[int, float]) -> tuple[Scenario, list[str]]:
  # the scenario, with the messages its replay keeps more frames of where thinning changes carState, and its problems.
  # changed fields that don't change how the replay engages only cost the scenario if keeping their messages doesn't fit
  problems, f = replay_problems(s, car, msgs, budget)
  if f is None or f.window or not (problems or f.fields):
    return s, problems
  keys = faithful_messages(s, car, msgs, budget)
  if keys is None:
    return s, problems + ["keeping every frame of the messages thinning drops doesn't help"] if problems else []
  # the fewest frames first, every k-th frame moves a change by at most k - 1 frames
  first_problems = None
  for k in range(SLACK + 1, 0, -1):
    kept = replace(s, kept=tuple((bus, addr, k) for bus, addr in keys))
    kept_problems, kept_f = replay_problems(kept, car, msgs, budget)
    if not kept_problems and kept_f is not None and not kept_f.fields:
      return kept, []
    first_problems = kept_problems if first_problems is None else first_problems
  if not problems:
    return s, []
  return s, problems + [f"keeping {_keys(keys)} every {SLACK + 1} frames: {', '.join(first_problems or ['carState still differs'])}"]


def pick_scenarios(routes, budget: dict[int, float]) -> tuple[list[Scenario], list[str]]:
  # what a rack run drives: each pair's first scenario whose replay plays on a station, and the gaps
  candidates, gaps = build_candidates(routes, partial(check_scenario, budget=budget))
  return [pair_candidates[0] for pair_candidates in candidates.values()], gaps
