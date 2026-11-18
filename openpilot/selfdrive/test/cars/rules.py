import bisect
import json
from dataclasses import dataclass
from functools import cached_property

from opendbc.car.structs import CarParams

# seconds after the panda's engage edge by which openpilot must be enabled
ENGAGE_LIMIT = 2.
# the panda marks what it sent with src + 128 and what it blocked with src + 192
BLOCKED = 192
BUSES = 3
LOG_LINE_LIMIT = 200
NO_CAR_MODES = (
  int(CarParams.SafetyModel.noOutput),
  int(CarParams.SafetyModel.silent),
  int(CarParams.SafetyModel.elm327),
)
SAFETY_MODELS = {v: k for k, v in CarParams.SafetyModel.schema.enumerants.items()}


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

  def lines(self) -> list[str]:
    mark = "✗" if self.failures else "✓"
    return [f"{mark} {self.evidence}"] + [f"  {line}" for _, line in self.failures]


def _judged_engaged(rec: Recording, i: int) -> bool:
  # openpilot is enabled at message i and stays so, in an engaged span that starts in the recording window
  return rec.enabled_at(i) and rec.stays_enabled(i) and rec.judged_span(rec.span_at(i))


def _not_engaged(rec: Recording) -> Check:
  failures = []
  evidence = []
  for edge, delay in _edges(rec):
    if delay is None:
      reasons = _openpilot_reasons(rec, edge + ENGAGE_LIMIT)
      failures.append((edge, f"t={edge:.2f} not engaged within {ENGAGE_LIMIT:.0f}s of controls_allowed: {reasons}"))
      evidence.append(f"not engaged at the panda's controls_allowed edge (t={edge:.2f})")
    else:
      evidence.append(f"engaged at the panda's controls_allowed edge (t={edge:.2f}, {delay:.2f} s after)")
  return Check("; ".join(evidence) or "no controls_allowed edge of the panda to engage on", failures)


def _blocked_while_engaged(rec: Recording) -> Check:
  failures = []
  reported = set()
  commands = 0
  blocked = 0
  for i, msg in enumerate(rec.msgs):
    which = msg.which()
    if which not in ('can', 'sendcan') or not _judged_engaged(rec, i):
      continue
    if which == 'sendcan':
      commands += len(msg.sendcan)
      continue
    for frame in msg.can:
      bus = frame.src - BLOCKED
      if bus not in range(BUSES):
        continue
      blocked += 1
      key = (rec.span_at(i), frame.address, bus)
      if key not in reported:
        reported.add(key)
        t = rec.t(i)
        failures.append((t, f"t={t:.2f} blocked while engaged, addr {frame.address:#x} bus {bus}"))
  return Check(f"{commands} commands while engaged, {blocked} blocked by the panda", failures)


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
    return Check("no engaged span to check the commands of", failures)
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


def rule_checks(msgs: list, mode: int, param: int, max_idle: int, start: int | None = None) -> list[Check]:
  rec = _load(msgs, start)
  return [
    _not_engaged(rec),
    _blocked_while_engaged(rec),
    _commands_stopped(rec, max_idle),
    _controls_allowed_dropped(rec),
    _safety_pair(rec, mode, param),
    _faults(rec),
  ]


def failure_lines(checks: list[Check]) -> list[str]:
  failures = [f for check in checks for f in check.failures]
  return [line for _, line in sorted(failures, key=lambda f: f[0])]


def rule_failures(msgs: list, mode: int, param: int, max_idle: int, start: int | None = None) -> list[str]:
  return failure_lines(rule_checks(msgs, mode, param, max_idle, start))


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
