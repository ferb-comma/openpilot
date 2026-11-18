import json
import unittest

from opendbc.car.structs import CarParams

from openpilot.cereal.messaging import new_message
from openpilot.selfdrive.test.cars.rules import SAFETY_STATE, TRANSITION, is_event, log_line, rule_checks, rule_failures

TOYOTA = int(CarParams.SafetyModel.toyota)


def panda_state(t, controls_allowed):
  msg = new_message('pandaStates', 1)
  msg.logMonoTime = int(t * 1e9)
  msg.pandaStates[0].controlsAllowed = controls_allowed
  msg.pandaStates[0].safetyModel = TOYOTA
  return msg


def selfdrive_state(t, enabled):
  msg = new_message('selfdriveState')
  msg.logMonoTime = int(t * 1e9)
  msg.selfdriveState.enabled = enabled
  return msg


def onroad_events(t, name, **flags):
  msg = new_message('onroadEvents', 1)
  msg.logMonoTime = int(t * 1e9)
  msg.onroadEvents[0].name = name
  for flag, value in flags.items():
    setattr(msg.onroadEvents[0], flag, value)
  return msg


def error_log(t, record):
  msg = new_message(None, errorLogMessage=json.dumps(record))
  msg.logMonoTime = int(t * 1e9)
  return msg


def can_msg(which, t, frames, dat=bytes(8)):
  msg = new_message(which, len(frames))
  msg.logMonoTime = int(t * 1e9)
  for c, (addr, src) in zip(getattr(msg, which), frames, strict=True):
    c.address = addr
    c.dat = dat
    c.src = src
  return msg


def blocked_command(t):
  # card's command and the panda's block of it in the next can message
  return [can_msg('sendcan', t - 0.005, [(0x100, 0)]), can_msg('can', t, [(0x100, 192)])]


def clean_drive(duration=5.):
  # panda allows at 1 s, openpilot enables 0.5 s later, one command a second after that
  msgs = [panda_state(0., False), selfdrive_state(0., False)]
  msgs += [panda_state(1., True), selfdrive_state(1.5, True)]
  for i in range(int(duration)):
    t = 2. + i
    msgs.append(can_msg('sendcan', t, [(0x100, 0)]))
    msgs.append(can_msg('can', t + 0.01, [(0x100, 128)]))
    msgs.append(selfdrive_state(t + 0.02, True))
    msgs.append(panda_state(t + 0.5, True))
  return msgs


def idle(msgs, since=0., until=float('inf')):
  # card's commands in [since, until) replaced by empty sendcan, as card publishes on a cycle with nothing to send
  out = []
  for m in msgs:
    if m.which() == 'sendcan' and since <= m.logMonoTime / 1e9 < until:
      m = can_msg('sendcan', m.logMonoTime / 1e9, [])
    out.append(m)
  return out


def cadence_drive(every, duration=3.):
  # openpilot enabled at 1.5 s, card publishes sendcan each 10 ms cycle with a command every `every` cycles
  msgs = [panda_state(0., False), selfdrive_state(0., False), panda_state(1., True), selfdrive_state(1.5, True)]
  for i in range(int(duration / 0.01)):
    t = 2. + i * 0.01
    frames = [(0x100, 0)] if i % every == 0 else []
    msgs.append(can_msg('sendcan', t, frames))
    msgs.append(selfdrive_state(t + 0.005, True))
  return msgs


def failures(msgs, max_idle=0):
  return rule_failures(msgs, TOYOTA, 0, max_idle)


GAS = 0x200


class FakeSafety:
  # libsafety that blocks every command once a gas frame came, or every command at all
  def __init__(self, blocks_all=False):
    self.blocks_all = blocks_all
    self.gas = False

  def set_safety_hooks(self, mode, param):
    return 0

  def safety_rx_hook(self, packet):
    self.gas |= packet[0].addr == GAS

  def safety_tx_hook(self, packet):
    return not (self.gas or self.blocks_all)

  def __getattr__(self, name):
    if name == 'get_gas_pressed_prev':
      return lambda: self.gas
    if name.startswith('get_') and name[4:] in SAFETY_STATE:
      return lambda: False
    return lambda *args: None


def oracle_failures(msgs, safety):
  return rule_failures(msgs, TOYOTA, 0, 0, safety=safety)


class TestRules(unittest.TestCase):
  def test_clean_drive(self):
    assert failures(clean_drive()) == []

  def test_clean_drive_evidence(self):
    lines = [line for check in rule_checks(clean_drive(), TOYOTA, 0, 0) for line in check.lines()]
    assert lines == [
      "✓ engaged at the panda's controls_allowed edge (t=1.00, 0.50 s after)",
      "✓ 5 commands while engaged, 0 blocked by the panda",
      "✓ commands never idle more than 0 cycles (route max 0, 1 of jitter)",
      "✓ controls_allowed held while engaged (5 panda states)",
      "✓ safety mode toyota param 0 = scenario",
      "✓ no new panda faults",
    ]

  def test_blocked_evidence(self):
    msgs = clean_drive()
    msgs.append(can_msg('can', 3.05, [(0x100, 192)]))
    blocked = rule_checks(msgs, TOYOTA, 0, 0)[1]
    assert blocked.lines()[:2] == ["✗ 5 commands while engaged, 1 blocked by the panda",
                                   "  t=3.05 blocked while engaged, addr 0x100 bus 0, not replayed through libsafety"]
    assert "    t=3.05 frame: 0000000000000000 (no DBC describes it)" in blocked.lines()

  def test_block_at_transition(self):
    # the gas press reaches the panda in the cycle before the block, openpilot's command is from before it, after the drive's commands
    msgs = clean_drive()
    msgs.append(can_msg('can', 6.5, [(GAS, 0)]))
    msgs += blocked_command(6.5 + (TRANSITION - 1) * 0.01)
    assert oracle_failures(msgs, FakeSafety()) == []
    blocked = rule_checks(msgs, TOYOTA, 0, 0, safety=FakeSafety())[1]
    assert "1 of them at a transition libsafety blocks too" in blocked.evidence

  def test_block_long_after_transition(self):
    msgs = clean_drive()
    msgs.append(can_msg('can', 6.5, [(GAS, 0)]))
    msgs += blocked_command(6.5 + (TRANSITION + 1) * 0.01)
    result = oracle_failures(msgs, FakeSafety())
    assert result == ["t=6.54 blocked while engaged, addr 0x100 bus 0, libsafety blocks it too, not at a transition"]

  def test_block_libsafety_allows(self):
    msgs = clean_drive()
    msgs += blocked_command(3.05)
    result = oracle_failures(msgs, FakeSafety())
    assert result == ["t=3.05 blocked while engaged, addr 0x100 bus 0, libsafety allows it"]
    notes = rule_checks(msgs, TOYOTA, 0, 0, safety=FakeSafety())[1].notes
    assert any("libsafety allows it: controls_allowed 0" in note for note in notes)

  def test_sent_libsafety_blocks(self):
    msgs = clean_drive()
    blocked = rule_checks(msgs, TOYOTA, 0, 0, safety=FakeSafety(blocks_all=True))[1]
    assert [line for _, line in blocked.failures] == ["t=2.01 sent while engaged, addr 0x100 bus 0, libsafety blocks it, not at a transition"]
    assert blocked.evidence.endswith("5 sent that libsafety blocks, 0 of them at a transition")
    assert any("libsafety blocks it: controls_allowed 0" in note for note in blocked.notes)

  def test_sent_at_transition(self):
    # the gas press reached the panda right after it took the command, its echo came after the press
    msgs = clean_drive()
    msgs.append(can_msg('sendcan', 6.5, [(0x100, 0)]))
    msgs.append(can_msg('can', 6.505, [(GAS, 0), (0x100, 128)]))
    blocked = rule_checks(msgs, TOYOTA, 0, 0, safety=FakeSafety())[1]
    assert blocked.failures == []
    assert blocked.evidence.endswith("1 sent that libsafety blocks, 1 of them at a transition")

  def test_tx_in_echo_order(self):
    # the panda queues its echo of a command with the rx as it takes it, after the drive's commands
    for frames, blocked in (([(0x100, 128), (GAS, 0)], 0), ([(GAS, 0), (0x100, 128)], 1)):
      msgs = clean_drive()
      msgs.append(can_msg('sendcan', 6.5, [(0x100, 0)]))
      msgs.append(can_msg('can', 6.505, frames))
      check = rule_checks(msgs, TOYOTA, 0, 0, safety=FakeSafety())[1]
      assert f"{blocked} sent that libsafety blocks" in check.evidence

  def test_forwarded_echo(self):
    # the panda's echo of a frame it forwards isn't a command, though openpilot sends the same message
    msgs = clean_drive()
    msgs.append(can_msg('can', 6.5, [(GAS, 0)]))
    msgs.append(can_msg('can', 6.51, [(0x100, 128)], dat=bytes([1] * 8)))
    blocked = rule_checks(msgs, TOYOTA, 0, 0, safety=FakeSafety())[1]
    assert blocked.evidence.endswith("0 sent that libsafety blocks")

  def test_brake_disengage(self):
    # the panda drops controls and blocks commands, openpilot disengages right after
    msgs = clean_drive(3.)
    msgs.append(panda_state(5., False))
    msgs.append(can_msg('can', 5., [(0x100, 192)]))
    msgs.append(selfdrive_state(5.02, False))
    assert failures(msgs) == []

  def test_stuck_engaged_after_drop(self):
    msgs = clean_drive(3.)
    msgs.append(panda_state(5., False))
    msgs.append(can_msg('can', 5., [(0x100, 192)]))
    for i in range(3):
      msgs.append(can_msg('sendcan', 5.01 + i, [(0x100, 0)]))
      msgs.append(selfdrive_state(5.02 + i, True))
    assert any("controls_allowed false while engaged" in f for f in failures(msgs))

  def test_not_engaged_names_reasons(self):
    msgs = [panda_state(0., False), selfdrive_state(0., False)]
    msgs += [onroad_events(0.5, 'calibrationIncomplete', noEntry=True), panda_state(1., True), selfdrive_state(3.5, False)]
    result = failures(msgs)
    assert result[1:] == ["never engaged"]
    assert "calibrationIncomplete (noEntry)" in result[0]
    assert "state disabled, engageable False" in result[0]

  def test_no_engage_edge(self):
    # the panda never allows controls, so openpilot never engages and nothing is judged
    msgs = [panda_state(0., False), selfdrive_state(0., False), panda_state(5., False), selfdrive_state(5., False)]
    assert failures(msgs) == ["no engage edge in the recording", "never engaged"]

  def test_not_engaged_no_events(self):
    msgs = [panda_state(0., False), selfdrive_state(0., False), panda_state(1., True), selfdrive_state(3.5, False)]
    assert "no onroadEvents recorded" in failures(msgs)[0]

  def test_not_engaged_comm_issue_services(self):
    msgs = [panda_state(0., False), selfdrive_state(0., False)]
    msgs.append(error_log(0.4, {'msg': {'event': 'commIssue', 'invalid': ['carState'], 'not_alive': ['radarState', 'modelV2'], 'not_freq_ok': []}}))
    msgs += [onroad_events(0.5, 'commIssue', noEntry=True, softDisable=True), panda_state(1., True), selfdrive_state(3.5, False)]
    result = failures(msgs)
    assert result[1:] == ["never engaged"]
    assert result[0].endswith("commIssue: invalid [carState], not alive [radarState, modelV2]")

  def test_not_engaged_comm_issue_before_window(self):
    # the commIssue record was logged before the recording window starts
    msgs = [error_log(1., {'msg': {'event': 'commIssue', 'invalid': ['carState'], 'not_alive': [], 'not_freq_ok': []}})]
    msgs += [panda_state(5., False), selfdrive_state(5., False), onroad_events(5.5, 'commIssue', noEntry=True)]
    msgs += [panda_state(6., True), selfdrive_state(8.5, False)]
    result = rule_failures(msgs, TOYOTA, 0, 0, start=int(5e9))
    assert result[1:] == ["never engaged"]
    assert result[0].endswith("commIssue: invalid [carState], not alive []")

  def test_log_line(self):
    record = {'msg': 'card crashed', 'ctx': {'daemon': 'card'}, 'exc_info': 'Traceback\n  File "card.py"\nKeyError: 1\n'}
    assert log_line(json.dumps(record)) == "card: KeyError: 1"
    record = {'msg': {'event': 'fingerprinted', 'car_fingerprint': 'MAZDA_6'}, 'ctx': {'daemon': 'card'}}
    assert log_line(json.dumps(record)) == 'card: {"event": "fingerprinted", "car_fingerprint": "MAZDA_6"}'
    assert log_line("not json\nline 2") == "?: not json line 2"

  def test_is_event(self):
    event = {'msg': {'event': 'fingerprinted', 'car_fingerprint': 'MAZDA_6'}, 'ctx': {'daemon': 'card'}}
    assert is_event(json.dumps(event))
    assert not is_event(json.dumps({'msg': 'dashcamOnly: SecOC is unsupported', 'ctx': {'daemon': 'card'}}))
    assert not is_event(json.dumps({'msg': {'event': 'x'}, 'exc_info': 'Traceback\nKeyError: 1\n'}))
    assert not is_event("not json")

  def test_start(self):
    # openpilot doesn't engage on either edge, only the one after the recording window's start is judged
    msgs = [panda_state(0., False), selfdrive_state(0., False), panda_state(1., True), panda_state(2., False)]
    msgs += [panda_state(5., True), selfdrive_state(7.5, False)]
    result = rule_failures(msgs, TOYOTA, 0, 0, start=int(4e9))
    assert result[1:] == ["never engaged"]
    assert result[0].startswith("t=1.00 not engaged")

  def test_blocked_while_engaged(self):
    msgs = clean_drive()
    msgs.append(can_msg('can', 3.05, [(0x100, 192)]))
    result = failures(msgs)
    assert len(result) == 1
    assert "blocked while engaged" in result[0]

  def test_commands_stop(self):
    result = failures(idle(clean_drive(8.), since=5.))
    assert len(result) == 1
    assert "commands stopped" in result[0]

  def test_commands_stop_before_disengage(self):
    msgs = clean_drive(3.)
    msgs += [can_msg('sendcan', 5., []), selfdrive_state(5.02, True), can_msg('sendcan', 6., []), selfdrive_state(6.02, False)]
    assert failures(msgs) == []

  def test_commands_stop_stays_engaged(self):
    msgs = clean_drive(3.)
    for i in range(3):
      msgs += [can_msg('sendcan', 5. + i, []), selfdrive_state(5.02 + i, True)]
    msgs.append(selfdrive_state(8.02, False))
    result = failures(msgs)
    assert len(result) == 1
    assert result[0].startswith("t=6.02 commands stopped")

  def test_commands_start_late(self):
    # no command in the first two samples of the span
    msgs = idle(clean_drive(6.), until=3.)
    msgs.append(selfdrive_state(2.02, True))
    assert failures(msgs) == []

  def test_commands_never_start(self):
    msgs = [m for m in clean_drive(6.) if m.which() != 'sendcan']
    result = failures(msgs)
    assert len(result) == 1
    assert "no commands while engaged" in result[0]

  def test_commands_never_start_empty(self):
    result = failures(idle(clean_drive(6.)))
    assert len(result) == 1
    assert "no commands while engaged" in result[0]

  def test_commands_start_then_stop(self):
    result = failures(idle(clean_drive(8.), since=5.))
    assert len(result) == 1
    assert "commands stopped while engaged" in result[0]

  def test_commands_50hz_cadence(self):
    # every other sendcan is empty, the car's cadence in its route
    assert failures(cadence_drive(2), max_idle=1) == []

  def test_commands_50hz_cadence_at_100hz(self):
    # one cycle more than the route is jitter
    assert failures(cadence_drive(2), max_idle=0) == []

  def test_commands_33hz_cadence_at_100hz(self):
    result = failures(cadence_drive(3), max_idle=0)
    assert len(result) == 1
    assert "commands stopped while engaged, 2 cycles idle" in result[0]

  def test_commands_stop_context(self):
    msgs = clean_drive(3.)
    for i in range(3):
      msgs += [can_msg('sendcan', 5. + i, []), selfdrive_state(5.02 + i, True)]
    msgs.append(selfdrive_state(8.02, False))
    notes = rule_checks(msgs, TOYOTA, 0, 0)[2].notes
    assert notes[0] == "t=6.02 idle 2000 ms since the command at t=4.00"
    assert notes[1] == "t=6.02 sendcan: 3 in it, gaps 1000-1000 ms"

  def test_commands_50hz_cadence_stop(self):
    result = failures(idle(cadence_drive(2, 4.), since=3.), max_idle=1)
    assert len(result) == 1
    assert result[0].startswith("t=3.01 commands stopped while engaged")


if __name__ == "__main__":
  unittest.main()
