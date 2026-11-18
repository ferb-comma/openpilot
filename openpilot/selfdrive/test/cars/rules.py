import bisect
from dataclasses import dataclass

from opendbc.car.structs import CarParams

# seconds after the panda's engage edge by which openpilot must be enabled
ENGAGE_LIMIT = 2.
# the panda marks what it sent with src + 128 and what it blocked with src + 192
BLOCKED = 192
BUSES = 3
NO_CAR_MODES = (
  int(CarParams.SafetyModel.noOutput),
  int(CarParams.SafetyModel.silent),
  int(CarParams.SafetyModel.elm327),
)


@dataclass
class Recording:
  msgs: list
  start: int
  # message index and enabled of each selfdriveState
  sd_idx: list[int]
  sd_enabled: list[bool]
  # engaged span number of each selfdriveState, counting enable edges
  sd_span: list[int]
  # engage edges and engaged spans that start before this are not judged
  warmup: float = 0.

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
    first, _ = self.spans()[span - 1]
    return self.t(self.sd_idx[first]) >= self.warmup

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


def _load(msgs: list, warmup: float) -> Recording:
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
  return Recording(msgs, msgs[0].logMonoTime if msgs else 0, sd_idx, sd_enabled, sd_span, warmup)


def _panda_states(rec: Recording):
  for i, msg in enumerate(rec.msgs):
    if msg.which() == 'pandaStates' and len(msg.pandaStates) > 0:
      yield i, msg.pandaStates[0]


BLOCKING_FLAGS = ('noEntry', 'softDisable', 'immediateDisable', 'userDisable')


def _openpilot_reasons(rec: Recording, end: float) -> str:
  # latest selfdriveState and onroadEvents at or before end
  state = None
  events = None
  for i, msg in enumerate(rec.msgs):
    if rec.t(i) > end:
      break
    if msg.which() == 'selfdriveState':
      state = msg.selfdriveState
    elif msg.which() == 'onroadEvents':
      events = msg.onroadEvents
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
  return ", ".join(parts)


def _edges(rec: Recording) -> list[tuple[float, bool]]:
  # each judged engage edge of the panda and whether openpilot engaged within the limit
  edges = []
  prev_allowed = None
  for i, state in _panda_states(rec):
    allowed = state.controlsAllowed
    if allowed and prev_allowed is False and rec.t(i) >= rec.warmup:
      edge = rec.t(i)
      engaged = rec.enabled_at(i)
      for pos, idx in enumerate(rec.sd_idx):
        if idx > i and rec.t(idx) <= edge + ENGAGE_LIMIT and rec.sd_enabled[pos]:
          engaged = True
      edges.append((edge, engaged))
    prev_allowed = allowed
  return edges


def _not_engaged(rec: Recording) -> list[tuple[float, str]]:
  failures = []
  for edge, engaged in _edges(rec):
    if not engaged:
      reasons = _openpilot_reasons(rec, edge + ENGAGE_LIMIT)
      failures.append((edge, f"t={edge:.2f} not engaged within {ENGAGE_LIMIT:.0f}s of controls_allowed: {reasons}"))
  return failures


def _blocked_while_engaged(rec: Recording) -> list[tuple[float, str]]:
  failures = []
  reported = set()
  for i, msg in enumerate(rec.msgs):
    if msg.which() != 'can':
      continue
    for frame in msg.can:
      bus = frame.src - BLOCKED
      if bus not in range(BUSES) or not rec.enabled_at(i) or not rec.stays_enabled(i):
        continue
      if not rec.judged_span(rec.span_at(i)):
        continue
      key = (rec.span_at(i), frame.address, bus)
      if key not in reported:
        reported.add(key)
        t = rec.t(i)
        failures.append((t, f"t={t:.2f} blocked while engaged, addr {frame.address:#x} bus {bus}"))
  return failures


def _controls_allowed_dropped(rec: Recording) -> list[tuple[float, str]]:
  failures = []
  reported = set()
  for i, state in _panda_states(rec):
    if state.controlsAllowed or not rec.enabled_at(i) or not rec.stays_enabled(i):
      continue
    span = rec.span_at(i)
    if span in reported or not rec.judged_span(span):
      continue
    # enabled for longer than the engage limit
    pos = bisect.bisect_right(rec.sd_idx, i) - 1
    while pos > 0 and rec.sd_enabled[pos - 1]:
      pos -= 1
    t = rec.t(i)
    if t - rec.t(rec.sd_idx[pos]) > ENGAGE_LIMIT:
      reported.add(span)
      failures.append((t, f"t={t:.2f} controls_allowed false while engaged"))
  return failures


def _commands_stopped(rec: Recording) -> list[tuple[float, str]]:
  failures = []
  sendcan_idx = [i for i, msg in enumerate(rec.msgs) if msg.which() == 'sendcan' and len(msg.sendcan) > 0]
  for first, last in rec.spans():
    if rec.t(rec.sd_idx[first]) < rec.warmup:
      continue
    for pos in range(first + 1, last + 1):
      lo = rec.sd_idx[pos - 1]
      hi = rec.sd_idx[pos]
      # at a disengage card stops one step before selfdriveState reports it
      stopped = bisect.bisect_left(sendcan_idx, hi) - bisect.bisect_right(sendcan_idx, lo) == 0
      if stopped and rec.stays_enabled(hi):
        t = rec.t(hi)
        failures.append((t, f"t={t:.2f} commands stopped while engaged"))
        break
  return failures


def _safety_pair(rec: Recording, mode: int, param: int) -> list[tuple[float, str]]:
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
  return failures


def _faults(rec: Recording) -> list[tuple[float, str]]:
  failures = []
  reported = set()
  for i, state in _panda_states(rec):
    for fault in state.faults:
      name = str(fault)
      if name not in reported:
        reported.add(name)
        t = rec.t(i)
        failures.append((t, f"t={t:.2f} new panda fault {name}"))
  return failures


def rule_failures(msgs: list, mode: int, param: int, warmup: float = 0.) -> list[str]:
  rec = _load(msgs, warmup)
  failures = []
  failures += _not_engaged(rec)
  failures += _blocked_while_engaged(rec)
  failures += _controls_allowed_dropped(rec)
  failures += _commands_stopped(rec)
  failures += _safety_pair(rec, mode, param)
  failures += _faults(rec)
  return [line for _, line in sorted(failures, key=lambda f: f[0])]


def summary(msgs: list, warmup: float = 0.) -> str:
  # engage edges openpilot engaged on, and the blocked and sent commands while engaged, after the warmup
  rec = _load(msgs, warmup)
  edges = _edges(rec)
  engaged = sum(e for _, e in edges)
  blocked = 0
  cmds = 0
  for i, msg in enumerate(rec.msgs):
    if rec.t(i) < warmup or not rec.enabled_at(i):
      continue
    if msg.which() == 'can':
      blocked += sum(c.src - BLOCKED in range(BUSES) for c in msg.can)
    elif msg.which() == 'sendcan':
      cmds += len(msg.sendcan)
  return f"engaged {engaged}/{len(edges)}, {blocked} blocked, {cmds} cmds"
