import json
import unittest

from opendbc.car.structs import CarParams

from openpilot.cereal.messaging import new_message
from openpilot.selfdrive.test.cars.rules import log_line, rule_failures

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


def can_msg(which, t, frames):
  msg = new_message(which, len(frames))
  msg.logMonoTime = int(t * 1e9)
  for c, (addr, src) in zip(getattr(msg, which), frames, strict=True):
    c.address = addr
    c.dat = bytes(8)
    c.src = src
  return msg


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


def failures(msgs):
  return rule_failures(msgs, TOYOTA, 0)


class TestRules(unittest.TestCase):
  def test_clean_drive(self):
    assert failures(clean_drive()) == []

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
    assert len(result) == 1
    assert "calibrationIncomplete (noEntry)" in result[0]
    assert "state disabled, engageable False" in result[0]

  def test_not_engaged_no_events(self):
    msgs = [panda_state(0., False), selfdrive_state(0., False), panda_state(1., True), selfdrive_state(3.5, False)]
    assert "no onroadEvents recorded" in failures(msgs)[0]

  def test_not_engaged_comm_issue_services(self):
    msgs = [panda_state(0., False), selfdrive_state(0., False)]
    msgs.append(error_log(0.4, {'msg': {'event': 'commIssue', 'invalid': ['carState'], 'not_alive': ['radarState', 'modelV2'], 'not_freq_ok': []}}))
    msgs += [onroad_events(0.5, 'commIssue', noEntry=True, softDisable=True), panda_state(1., True), selfdrive_state(3.5, False)]
    result = failures(msgs)
    assert len(result) == 1
    assert result[0].endswith("commIssue: invalid [carState], not alive [radarState, modelV2]")

  def test_not_engaged_comm_issue_before_window(self):
    # the commIssue record was logged before the recording window starts
    msgs = [error_log(1., {'msg': {'event': 'commIssue', 'invalid': ['carState'], 'not_alive': [], 'not_freq_ok': []}})]
    msgs += [panda_state(5., False), selfdrive_state(5., False), onroad_events(5.5, 'commIssue', noEntry=True)]
    msgs += [panda_state(6., True), selfdrive_state(8.5, False)]
    result = rule_failures(msgs, TOYOTA, 0, start=int(5e9))
    assert len(result) == 1
    assert result[0].endswith("commIssue: invalid [carState], not alive []")

  def test_log_line(self):
    record = {'msg': 'card crashed', 'ctx': {'daemon': 'card'}, 'exc_info': 'Traceback\n  File "card.py"\nKeyError: 1\n'}
    assert log_line(json.dumps(record)) == "card: KeyError: 1"
    record = {'msg': {'event': 'fingerprinted', 'car_fingerprint': 'MAZDA_6'}, 'ctx': {'daemon': 'card'}}
    assert log_line(json.dumps(record)) == 'card: {"event": "fingerprinted", "car_fingerprint": "MAZDA_6"}'
    assert log_line("not json\nline 2") == "?: not json line 2"

  def test_start(self):
    # openpilot doesn't engage on either edge, only the one after the recording window's start is judged
    msgs = [panda_state(0., False), selfdrive_state(0., False), panda_state(1., True), panda_state(2., False)]
    msgs += [panda_state(5., True), selfdrive_state(7.5, False)]
    result = rule_failures(msgs, TOYOTA, 0, start=int(4e9))
    assert len(result) == 1
    assert result[0].startswith("t=1.00 not engaged")

  def test_blocked_while_engaged(self):
    msgs = clean_drive()
    msgs.append(can_msg('can', 3.05, [(0x100, 192)]))
    result = failures(msgs)
    assert len(result) == 1
    assert "blocked while engaged" in result[0]

  def test_commands_stop(self):
    msgs = [m for m in clean_drive(8.) if not (m.which() == 'sendcan' and m.logMonoTime >= 5e9)]
    result = failures(msgs)
    assert len(result) == 1
    assert "commands stopped" in result[0]

  def test_commands_stop_before_disengage(self):
    msgs = clean_drive(3.)
    msgs += [selfdrive_state(5.02, True), selfdrive_state(6.02, False)]
    assert failures(msgs) == []

  def test_commands_stop_stays_engaged(self):
    msgs = clean_drive(3.)
    msgs += [selfdrive_state(5.02 + i, True) for i in range(3)]
    msgs.append(selfdrive_state(8.02, False))
    result = failures(msgs)
    assert len(result) == 1
    assert result[0].startswith("t=5.02 commands stopped")


if __name__ == "__main__":
  unittest.main()
