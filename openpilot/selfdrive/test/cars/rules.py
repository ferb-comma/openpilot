import bisect
import json
import os
from contextlib import suppress
from dataclasses import dataclass, field
from functools import cached_property

from opendbc.can.dbc import DBC, Msg, SignalType
from opendbc.car.structs import CarParams
from opendbc.car.values import PLATFORMS
from opendbc.safety.tests.libsafety import libsafety_py

# seconds after the panda's engage edge by which openpilot must be enabled
ENGAGE_LIMIT = 2.
# the panda marks what it sent with src + 128 and what it blocked with src + 192
ECHO_BASE = 128
BLOCKED = 192
BUSES = 3
LOG_LINE_LIMIT = 200
NO_CAR_MODES = (
  int(CarParams.SafetyModel.noOutput),
  int(CarParams.SafetyModel.silent),
  int(CarParams.SafetyModel.elm327),
)
SAFETY_MODELS = {v: k for k, v in CarParams.SafetyModel.schema.enumerants.items()}
# openpilot's control cycle
DT = 0.01
# the panda takes a CAN edge (a pedal, a cancel, controls dropped) the moment it arrives, card actuates the carControl
# controlsd made from the previous carState, so a command up to two cycles after the edge still comes from before it,
# and a recording's can message carries its frames at the time pandad read them, up to a cycle late
TRANSITION = 3
# libsafety's state the TX checks depend on
SAFETY_STATE = ('controls_allowed', 'gas_pressed_prev', 'brake_pressed_prev', 'regen_braking_prev', 'steering_disengage_prev',
                'cruise_engaged_prev', 'vehicle_moving', 'acc_main_on', 'relay_malfunction')


def is_counter(sig) -> bool:
  return sig.type == SignalType.COUNTER or sig.name == "COUNTER"


def signal_values(msg: Msg, dat: bytes) -> dict[str, float]:
  # the message's signals but its counter and checksum
  values = {}
  for sig in msg.sigs.values():
    if is_counter(sig) or sig.calc_checksum is not None:
      continue
    raw = sig.get_raw_value(dat)
    if sig.is_signed:
      raw -= ((raw >> (sig.size - 1)) & 0x1) * (1 << sig.size)
    values[sig.name] = raw * sig.factor + sig.offset
  return values


@dataclass
class Recording:
  msgs: list
  # logMonoTime the recording window starts at, engage edges and engaged spans before it are not judged
  start: int
  # message index and enabled of each selfdriveState
  sd_idx: list[int]
  sd_enabled: list[bool]
  # engaged span number of each selfdriveState, counting enable edges
  sd_span: list[int]

  def t(self, i: int) -> float:
    return (self.msgs[i].logMonoTime - self.start) / 1e9

  def next_sd(self, i: int) -> int | None:
    # position in sd_idx of the first selfdriveState after message i
    pos = bisect.bisect_right(self.sd_idx, i)
    return pos if pos < len(self.sd_idx) else None

  def enabled_at(self, i: int) -> bool:
    pos = bisect.bisect_right(self.sd_idx, i) - 1
    return pos >= 0 and self.sd_enabled[pos]

  def span_at(self, i: int) -> int:
    pos = bisect.bisect_right(self.sd_idx, i) - 1
    return self.sd_span[pos] if pos >= 0 else 0

  def stays_enabled(self, i: int) -> bool:
    # the next selfdriveState after message i is still enabled, or there is none
    pos = self.next_sd(i)
    return pos is None or self.sd_enabled[pos]

  def judged_span(self, span: int) -> bool:
    first, _ = self.spans[span - 1]
    return self.t(self.sd_idx[first]) >= 0.

  @cached_property
  def spans(self) -> list[tuple[int, int]]:
    # (first, last) positions in sd_idx of each run of enabled samples
    spans = []
    first = None
    for pos, enabled in enumerate(self.sd_enabled):
      if enabled and first is None:
        first = pos
      if not enabled and first is not None:
        spans.append((first, pos - 1))
        first = None
    if first is not None:
      spans.append((first, len(self.sd_enabled) - 1))
    return spans


def _load(msgs: list, start: int | None) -> Recording:
  msgs = sorted(msgs, key=lambda m: m.logMonoTime)
  sd_idx = []
  sd_enabled = []
  sd_span = []
  span = 0
  prev = False
  for i, msg in enumerate(msgs):
    if msg.which() == 'selfdriveState':
      enabled = msg.selfdriveState.enabled
      if enabled and not prev:
        span += 1
      prev = enabled
      sd_idx.append(i)
      sd_enabled.append(enabled)
      sd_span.append(span)
  if start is None:
    start = msgs[0].logMonoTime if msgs else 0
  return Recording(msgs, start, sd_idx, sd_enabled, sd_span)


def _panda_states(rec: Recording):
  for i, msg in enumerate(rec.msgs):
    if msg.which() == 'pandaStates' and len(msg.pandaStates) > 0:
      yield i, msg.pandaStates[0]


BLOCKING_FLAGS = ('noEntry', 'softDisable', 'immediateDisable', 'userDisable')


def log_record(text: str) -> dict:
  try:
    record = json.loads(text)
  except ValueError:
    return {'msg': text}
  return record if isinstance(record, dict) else {'msg': text}


def is_event(text: str) -> bool:
  # cloudlog.event records carry a dict instead of a message and no exception
  record = log_record(text)
  return isinstance(record.get('msg'), dict) and not record.get('exc_info')


def log_line(text: str) -> str:
  # a cloudlog record in one short line, its process and message, or the last line of its exception
  record = log_record(text)
  msg = record.get('msg', '')
  if record.get('exc_info'):
    msg = record['exc_info'].strip().splitlines()[-1]
  elif not isinstance(msg, str):
    msg = json.dumps(msg)
  daemon = record.get('ctx', {}).get('daemon', '?')
  line = ' '.join(f"{daemon}: {msg}".split())
  return line[:LOG_LINE_LIMIT]


def fingerprinted(msgs: list) -> str:
  # card's fingerprinted record, how it got its car model and firmware
  for msg in msgs:
    if msg.which() != 'errorLogMessage':
      continue
    log = log_record(msg.errorLogMessage).get('msg')
    if isinstance(log, dict) and log.get('event') == 'fingerprinted':
      source = log.get('source')
      return f"card: fingerprinted {log.get('car_fingerprint')} from {source}, cached firmware {log.get('cached')}, {log.get('fw_count')} carFw entries"
  return "card: no fingerprinted record"


def _comm_issue(log: dict | None) -> str:
  # the services selfdrived logs with its commIssue event
  if log is None:
    return "commIssue: no log recorded"
  return f"commIssue: invalid [{', '.join(log.get('invalid', []))}], not alive [{', '.join(log.get('not_alive', []))}]"


def _openpilot_reasons(rec: Recording, end: float) -> str:
  # latest selfdriveState, onroadEvents and commIssue log at or before end
  state = None
  events = None
  comm_issue = None
  for i, msg in enumerate(rec.msgs):
    if rec.t(i) > end:
      break
    if msg.which() == 'selfdriveState':
      state = msg.selfdriveState
    elif msg.which() == 'onroadEvents':
      events = msg.onroadEvents
    elif msg.which() == 'errorLogMessage':
      log = log_record(msg.errorLogMessage).get('msg')
      if isinstance(log, dict) and log.get('event') == 'commIssue':
        comm_issue = log
  parts = []
  if state is not None:
    parts.append(f"state {state.state}, engageable {state.engageable}")
  if events is None:
    parts.append("no onroadEvents recorded")
  else:
    names = []
    for event in events:
      flags = [flag for flag in BLOCKING_FLAGS if getattr(event, flag)]
      if flags:
        names.append(f"{event.name} ({', '.join(flags)})")
    parts.append(f"events [{', '.join(names)}]")
    if any(str(event.name) == 'commIssue' for event in events):
      parts.append(_comm_issue(comm_issue))
  return ", ".join(parts)


def _edges(rec: Recording) -> list[tuple[float, float | None]]:
  # each judged engage edge of the panda and how long after it openpilot was enabled, None if not within the limit
  edges = []
  prev_allowed = None
  for i, state in _panda_states(rec):
    allowed = state.controlsAllowed
    if allowed and prev_allowed is False and rec.t(i) >= 0.:
      edge = rec.t(i)
      delay = 0. if rec.enabled_at(i) else None
      for pos, idx in enumerate(rec.sd_idx):
        if delay is None and idx > i and rec.t(idx) <= edge + ENGAGE_LIMIT and rec.sd_enabled[pos]:
          delay = rec.t(idx) - edge
      edges.append((edge, delay))
    prev_allowed = allowed
  return edges


@dataclass
class Check:
  # a rule's evidence on a recording, and its failures as (t, line)
  evidence: str
  failures: list[tuple[float, str]]
  # what explains each failure, and what the rule let pass
  notes: list[str] = field(default_factory=list)

  def lines(self) -> list[str]:
    mark = "✗" if self.failures else "✓"
    return [f"{mark} {self.evidence}"] + [f"  {line}" for _, line in self.failures] + [f"    {note}" for note in self.notes]


def _judged_engaged(rec: Recording, i: int) -> bool:
  # openpilot is enabled at message i and stays so, in an engaged span that starts in the recording window
  return rec.enabled_at(i) and rec.stays_enabled(i) and rec.judged_span(rec.span_at(i))


def _not_engaged(rec: Recording) -> Check:
  failures = []
  evidence = []
  edges = _edges(rec)
  if not edges:
    return Check("no controls_allowed edge of the panda to engage on", [(0., "no engage edge in the recording")])
  for edge, delay in edges:
    if delay is None:
      reasons = _openpilot_reasons(rec, edge + ENGAGE_LIMIT)
      failures.append((edge, f"t={edge:.2f} not engaged within {ENGAGE_LIMIT:.0f}s of controls_allowed: {reasons}"))
      evidence.append(f"not engaged at the panda's controls_allowed edge (t={edge:.2f})")
    else:
      evidence.append(f"engaged at the panda's controls_allowed edge (t={edge:.2f}, {delay:.2f} s after)")
  return Check("; ".join(evidence), failures)


def load_libsafety():
  # libsafety to replay a recording through, and why there is none. a device has no UBSan runtime for the default build,
  # so there it builds without the sanitizer
  try:
    return libsafety_py.libsafety, ""
  except Exception:
    pass
  coverage = os.environ.get("SAFETY_COVERAGE")
  os.environ["SAFETY_COVERAGE"] = "1"
  try:
    libsafety_py.load(libsafety_py._build_libsafety(release=True))
    return libsafety_py.libsafety, ""
  except Exception as e:
    return None, f"libsafety doesn't build here: {e!r}"
  finally:
    if coverage is None:
      del os.environ["SAFETY_COVERAGE"]
    else:
      os.environ["SAFETY_COVERAGE"] = coverage


@dataclass
class Decision:
  # libsafety's decision on a frame the panda sent or blocked, its state then and how long before it last changed
  allowed: bool
  state: dict[str, bool]
  since_change: float

  def describe(self) -> str:
    state = ", ".join(f"{name} {int(value)}" for name, value in self.state.items())
    return f"libsafety {'allows' if self.allowed else 'blocks'} it: {state}, changed {self.since_change * 1e3:.0f} ms before"


def _safety_state(safety) -> dict[str, bool]:
  return {name: bool(getattr(safety, f"get_{name}")()) for name in SAFETY_STATE}


def safety_decisions(rec: Recording, mode: int, param: int, safety) -> dict[tuple[int, int], Decision]:
  # (message index, frame index) of each frame openpilot sent that the panda sent (src + 128) or blocked (src + 192) -> libsafety's decision,
  # fed the recording's rx (src 0-2) and openpilot's engaged state in the heartbeat in the order pandad read them
  if safety.set_safety_hooks(mode, param) != 0:
    return {}
  # the panda also echoes the frames it forwards, only what openpilot sent went through its tx hook
  sent = {(c.address, c.src) for m in rec.msgs if m.which() == 'sendcan' for c in m.sendcan}
  first = rec.msgs[0].logMonoTime if rec.msgs else 0
  decisions = {}
  enabled = False
  tick = None
  state = _safety_state(safety)
  changed = 0.
  for i, msg in enumerate(rec.msgs):
    which = msg.which()
    t = (msg.logMonoTime - first) / 1e9
    if which == 'selfdriveState':
      enabled = msg.selfdriveState.enabled
    elif which == 'pandaStates' and len(msg.pandaStates):
      safety.set_alternative_experience(msg.pandaStates[0].alternativeExperience)
    if which != 'can':
      continue
    safety.set_timer(round(t * 1e6) & 0xFFFFFFFF)
    # the panda calls safety_tick and checks the heartbeat at 1 Hz
    if tick is None or t - tick >= 1.:
      tick = t
      safety.safety_heartbeat(enabled)
      safety.safety_tick()
      safety.safety_heartbeat_check()
    for k, frame in enumerate(msg.can):
      if frame.src < BUSES:
        safety.safety_rx_hook(libsafety_py.make_CANPacket(frame.address, frame.src, frame.dat))
      elif frame.src - BLOCKED in range(BUSES) or (frame.address, frame.src - ECHO_BASE) in sent:
        bus = frame.src - (ECHO_BASE if frame.src < BLOCKED else BLOCKED)
        allowed = bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, bus, frame.dat)))
        decisions[(i, k)] = Decision(allowed, state, t - changed)
      now = _safety_state(safety)
      if now != state:
        state = now
        changed = t
  return decisions


def _latest(rec: Recording, i: int, which: str):
  for j in range(i, -1, -1):
    if rec.msgs[j].which() == which:
      return getattr(rec.msgs[j], which)
  return None


def _decoded(dbcs: list[DBC], addr: int, dat: bytes) -> str:
  for dbc in dbcs:
    msg = dbc.addr_to_msg.get(addr)
    if msg is not None:
      values = ", ".join(f"{name} {value:g}" for name, value in signal_values(msg, dat).items())
      return f"{msg.name} {values}"
  return f"{dat.hex()} (no DBC describes it)"


def block_context(rec: Recording, i: int, frame, decision: Decision | None, dbcs: list[DBC]) -> list[str]:
  # the panda's, libsafety's and openpilot's view when the panda blocked a frame
  lines = [f"frame: {_decoded(dbcs, frame.address, bytes(frame.dat))}"]
  panda = _latest(rec, i, 'pandaStates')
  if panda is not None and len(panda):
    p = panda[0]
    checks = f"rx checks invalid {int(p.safetyRxChecksInvalid)}, tx blocked {p.safetyTxBlocked}"
    lines.append(f"panda: controls_allowed {int(p.controlsAllowed)}, rx invalid {p.safetyRxInvalid}, {checks}")
  if decision is not None:
    lines.append(decision.describe())
  CS = _latest(rec, i, 'carState')
  if CS is not None:
    pedals = f"gasPressed {int(CS.gasPressed)}, brakePressed {int(CS.brakePressed)}"
    lines.append(f"carState: {pedals}, standstill {int(CS.standstill)}, cruise enabled {int(CS.cruiseState.enabled)}, vEgo {CS.vEgo:.2f}")
  CC = _latest(rec, i, 'carControl')
  if CC is not None:
    lines.append(f"carControl: enabled {int(CC.enabled)}, latActive {int(CC.latActive)}, longActive {int(CC.longActive)}")
  return lines


def _blocked_while_engaged(rec: Recording, mode: int, param: int, safety, why_not: str, dbcs: list[DBC]) -> Check:
  # a block is a failure unless libsafety, fed the same rx, blocks it too right after its state changed, a transition
  # openpilot reacts to a cycle later. without libsafety every block is
  decisions = safety_decisions(rec, mode, param, safety) if safety is not None else {}
  failures = []
  notes = []
  reported = set()
  commands = 0
  blocked = 0
  transitions = 0
  # frames the panda sent that libsafety blocks, libsafety's replay differs from the panda there
  oracle_only = 0
  for i, msg in enumerate(rec.msgs):
    which = msg.which()
    if which not in ('can', 'sendcan') or not _judged_engaged(rec, i):
      continue
    if which == 'sendcan':
      commands += len(msg.sendcan)
      continue
    for k, frame in enumerate(msg.can):
      decision = decisions.get((i, k))
      if frame.src - ECHO_BASE in range(BUSES):
        oracle_only += decision is not None and not decision.allowed
        continue
      bus = frame.src - BLOCKED
      if bus not in range(BUSES):
        continue
      blocked += 1
      if decision is not None and not decision.allowed and decision.since_change <= TRANSITION * DT:
        transitions += 1
        continue
      key = (rec.span_at(i), frame.address, bus)
      if key in reported:
        continue
      reported.add(key)
      t = rec.t(i)
      if safety is None:
        reason = "not replayed through libsafety"
      elif decision is None:
        reason = "libsafety has no decision on it"
      elif decision.allowed:
        reason = "libsafety allows it"
      else:
        reason = "libsafety blocks it too, not at a transition"
      failures.append((t, f"t={t:.2f} blocked while engaged, addr {frame.address:#x} bus {bus}, {reason}"))
      notes += [f"t={t:.2f} {line}" for line in block_context(rec, i, frame, decision, dbcs)]
  evidence = f"{commands} commands while engaged, {blocked} blocked by the panda"
  if safety is None:
    evidence += f", not replayed through libsafety, {why_not}" if why_not else ""
  else:
    evidence += f", {transitions} of them at a transition libsafety blocks too, {oracle_only} sent that libsafety blocks"
  return Check(evidence, failures, notes)


def _controls_allowed_dropped(rec: Recording) -> Check:
  failures = []
  reported = set()
  held = 0
  for i, state in _panda_states(rec):
    if not rec.enabled_at(i) or not rec.stays_enabled(i):
      continue
    span = rec.span_at(i)
    if not rec.judged_span(span):
      continue
    if state.controlsAllowed:
      held += 1
      continue
    if span in reported:
      continue
    # enabled for longer than the engage limit
    pos = bisect.bisect_right(rec.sd_idx, i) - 1
    while pos > 0 and rec.sd_enabled[pos - 1]:
      pos -= 1
    t = rec.t(i)
    if t - rec.t(rec.sd_idx[pos]) > ENGAGE_LIMIT:
      reported.add(span)
      failures.append((t, f"t={t:.2f} controls_allowed false while engaged"))
  if failures:
    return Check("controls_allowed dropped while engaged", failures)
  return Check(f"controls_allowed held while engaged ({held} panda states)", failures)


def _commands_stopped(rec: Recording, max_idle: int) -> Check:
  # card publishes sendcan every cycle, empty on the cycles the car's controller sends nothing,
  # so commands stopped is a run of more empty sendcan than the car's longest in its route
  failures = []
  # the longest run of empty sendcan between two commands, as the route's max_idle is measured
  longest = None
  judged = False
  sendcan = [i for i, msg in enumerate(rec.msgs) if msg.which() == 'sendcan']
  for first, last in rec.spans:
    if rec.t(rec.sd_idx[first]) < 0.:
      continue
    judged = True
    # card starts sending a cycle after openpilot enables, so only a run after the first command counts
    in_span = [i for i in sendcan if rec.sd_idx[first] < i < rec.sd_idx[last]]
    started = next((n for n, i in enumerate(in_span) if len(rec.msgs[i].sendcan) > 0), None)
    if started is None:
      for pos in range(first + 1, last + 1):
        if rec.stays_enabled(rec.sd_idx[pos]):
          t = rec.t(rec.sd_idx[pos])
          failures.append((t, f"t={t:.2f} no commands while engaged"))
          break
      continue
    run = 0
    failed = False
    for i in in_span[started:]:
      if len(rec.msgs[i].sendcan) > 0:
        longest = max(longest or 0, run)
        run = 0
        continue
      run += 1
      if run <= max_idle or failed:
        continue
      # at a disengage card stops one step before selfdriveState reports it
      pos = rec.next_sd(i)
      if pos is not None and rec.stays_enabled(rec.sd_idx[pos]):
        t = rec.t(rec.sd_idx[pos])
        failures.append((t, f"t={t:.2f} commands stopped while engaged"))
        failed = True
  if failures:
    return Check(f"commands idle more than the route max of {max_idle} cycles while engaged", failures)
  if not judged:
    # after the engage edge failures it explains
    end = rec.t(len(rec.msgs) - 1) if rec.msgs else 0.
    return Check("no engaged span to check the commands of", [(end, "never engaged")])
  if longest is None:
    return Check(f"one command while engaged (route max idle {max_idle} cycles)", failures)
  return Check(f"commands never idle more than {longest} cycles (route max {max_idle})", failures)


def _safety_pair(rec: Recording, mode: int, param: int) -> Check:
  failures = []
  car_mode_seen = False
  reported = None
  for i, state in _panda_states(rec):
    logged = state.safetyModel.raw
    if logged not in NO_CAR_MODES:
      car_mode_seen = True
    pair = (logged, state.safetyParam)
    if car_mode_seen and pair != (mode, param) and pair != reported:
      reported = pair
      t = rec.t(i)
      failures.append((t, f"t={t:.2f} safety pair {pair} != {(mode, param)}"))
  if failures:
    return Check(f"safety mode differs from the scenario's {SAFETY_MODELS.get(mode, mode)} param {param}", failures)
  if not car_mode_seen:
    return Check("panda never in a car safety mode", failures)
  return Check(f"safety mode {SAFETY_MODELS.get(mode, mode)} param {param} = scenario", failures)


def _faults(rec: Recording) -> Check:
  failures = []
  reported = set()
  for i, state in _panda_states(rec):
    for fault in state.faults:
      name = str(fault)
      if name not in reported:
        reported.add(name)
        t = rec.t(i)
        failures.append((t, f"t={t:.2f} new panda fault {name}"))
  return Check("new panda faults" if failures else "no new panda faults", failures)


def car_dbcs(car_model: str) -> list[DBC]:
  # the DBCs the car's frames decode with, none for an unknown car
  platform = PLATFORMS.get(car_model)
  if platform is None:
    return []
  dbcs = []
  for name in dict.fromkeys(platform.config.dbc_dict.values()):
    with suppress(Exception):
      dbcs.append(DBC(name))
  return dbcs


def rule_checks(msgs: list, mode: int, param: int, max_idle: int, start: int | None = None, safety=None, why_not: str = "",
                car_model: str = "") -> list[Check]:
  # safety: libsafety to replay the recording through, why_not: why there is none
  rec = _load(msgs, start)
  return [
    _not_engaged(rec),
    _blocked_while_engaged(rec, mode, param, safety, why_not, car_dbcs(car_model)),
    _commands_stopped(rec, max_idle),
    _controls_allowed_dropped(rec),
    _safety_pair(rec, mode, param),
    _faults(rec),
  ]


def failure_lines(checks: list[Check]) -> list[str]:
  failures = [f for check in checks for f in check.failures]
  return [line for _, line in sorted(failures, key=lambda f: f[0])]


def rule_failures(msgs: list, mode: int, param: int, max_idle: int, start: int | None = None, safety=None, why_not: str = "") -> list[str]:
  return failure_lines(rule_checks(msgs, mode, param, max_idle, start, safety, why_not))


def summary(msgs: list, start: int | None = None) -> str:
  # engage edges openpilot engaged on, and the blocked and sent commands while engaged, from the start
  rec = _load(msgs, start)
  edges = _edges(rec)
  engaged = sum(delay is not None for _, delay in edges)
  blocked = 0
  cmds = 0
  for i, msg in enumerate(rec.msgs):
    if rec.t(i) < 0. or not rec.enabled_at(i):
      continue
    if msg.which() == 'can':
      blocked += sum(c.src - BLOCKED in range(BUSES) for c in msg.can)
    elif msg.which() == 'sendcan':
      cmds += len(msg.sendcan)
  return f"engaged {engaged}/{len(edges)}, {blocked} blocked, {cmds} cmds"
