import time
from unittest import mock

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import CarParams, car
from openpilot.cereal import log
from openpilot.common.parameterized import parameterized
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.pandad import pandad
from openpilot.selfdrive.pandad.panda_safety import PandaSafety
from openpilot.selfdrive.pandad.pandad import PeripheralControl, can_recv, forward_logs, send_panda_states, send_peripheral_state
from openpilot.selfdrive.pandad.tests.helpers import TRES, FakePanda, FakePubMaster

SafetyModel = CarParams.SafetyModel
FaultType = log.PandaState.FaultType


def published(pm: FakePubMaster, service: str):
  s, dat = pm.sent[-1]
  assert s == service
  return messaging.log_from_bytes(dat)


# pandaStates field: Panda.health() key, for the ones copied as is
PANDA_STATE_KEYS = {
  "voltage": "voltage", "current": "current", "uptime": "uptime", "safetyTxBlocked": "safety_tx_blocked",
  "safetyRxInvalid": "safety_rx_invalid", "ignitionLine": "ignition_line", "ignitionCan": "ignition_can",
  "controlsAllowed": "controls_allowed", "txBufferOverflow": "tx_buffer_overflow", "rxBufferOverflow": "rx_buffer_overflow",
  "safetyParam": "safety_param", "powerSaveEnabled": "power_save_enabled", "heartbeatLost": "heartbeat_lost",
  "interruptLoad": "interrupt_load", "fanPower": "fan_power", "safetyRxChecksInvalid": "safety_rx_checks_invalid",
  "nmiReset": "nmi_reset", "hardfaultReset": "hardfault_reset", "spiErrorCount": "spi_error_count",
  "soundOutputLevel": "sound_output_level",
}
CAN_STATE_KEYS = {
  "busOff": "bus_off", "busOffCnt": "bus_off_cnt", "errorWarning": "error_warning", "errorPassive": "error_passive",
  "receiveErrorCnt": "receive_error_cnt", "transmitErrorCnt": "transmit_error_cnt", "totalErrorCnt": "total_error_cnt",
  "totalTxLostCnt": "total_tx_lost_cnt", "totalRxLostCnt": "total_rx_lost_cnt", "totalTxCnt": "total_tx_cnt",
  "totalRxCnt": "total_rx_cnt", "totalFwdCnt": "total_fwd_cnt", "canSpeed": "can_speed", "canDataSpeed": "can_data_speed",
  "canfdEnabled": "canfd_enabled", "brsEnabled": "brs_enabled", "canfdNonIso": "canfd_non_iso",
  "irq0CallRate": "irq0_call_rate", "irq1CallRate": "irq1_call_rate", "irq2CallRate": "irq2_call_rate",
  "canCoreResetCnt": "can_core_reset_count",
}
LEC_NAMES = ['noError', 'stuffError', 'formError', 'ackError', 'bit1Error', 'bit0Error', 'crcError', 'noChange']


class TestPandaState(OpenpilotTestCase):
  def test_fields(self):
    faults = (1 << int(FaultType.relayMalfunction)) | (1 << int(FaultType.safetyRedundancy)) | (1 << int(FaultType.heartbeatLoopWatchdog)) | (1 << 31)
    panda, pm = FakePanda(faults=faults, alternative_experience=0xFFFF, ignition_line=True), FakePubMaster()
    send_panda_states(pm, panda, TRES, is_onroad=True, spoofing_started=False)

    evt = published(pm, 'pandaStates')
    assert evt.valid
    assert len(evt.pandaStates) == 1
    ps = evt.pandaStates[0].to_dict()
    for k in ('deprecated', 'controlsAllowedRESERVED1', 'controlsAllowedRESERVED2'):
      ps.pop(k)
    health = panda.health_dict
    expected: dict = {f: health[k] for f, k in PANDA_STATE_KEYS.items()}
    expected |= {
      'pandaType': 'tres', 'safetyModel': 'noOutput', 'faultStatus': 'faultPerm', 'harnessStatus': 'normal',
      'alternativeExperience': -1,  # Int16
      'sbu1Voltage': 1.0, 'sbu2Voltage': 2.0,
      # ascending, and bits past the last fault type dropped
      'faults': ['relayMalfunction', 'heartbeatLoopWatchdog', 'safetyRedundancy'],
    }
    for i in range(3):
      can_health = panda.can_health(i)
      expected[f'canState{i}'] = {
        **{f: can_health[k] for f, k in CAN_STATE_KEYS.items()},
        # the error codes FakePanda.can_health() reports as strings
        'lastError': LEC_NAMES[(i + 1) % 8], 'lastStoredError': LEC_NAMES[(i + 3) % 8],
        'lastDataError': LEC_NAMES[(i + 5) % 8], 'lastDataStoredError': LEC_NAMES[(i + 7) % 8],
      }
    assert ps == expected

  @parameterized.expand([
    # SILENT goes to NO_OUTPUT twice: to make the bus live, and because we're offroad
    ("silent_offroad", {"safety_mode": SafetyModel.silent, "power_save_enabled": False}, False, False,
     [("set_safety_mode", SafetyModel.noOutput, 0), ("set_power_save", True), ("set_safety_mode", SafetyModel.noOutput, 0)]),
    # car safety mode stays, power save goes off
    ("onroad_ignition", {"safety_mode": SafetyModel.hondaNidec, "ignition_can": True}, True, False, [("set_power_save", False)]),
    # ignition gone while still onroad: relay closes
    ("ignition_lost", {"safety_mode": SafetyModel.hondaNidec, "power_save_enabled": False}, True, False,
     [("set_power_save", True), ("set_safety_mode", SafetyModel.noOutput, 0)]),
    # offroad with ignition on, e.g. too hot: relay closes
    ("offroad_ignition", {"safety_mode": SafetyModel.hondaNidec, "ignition_can": True, "power_save_enabled": False}, False, False,
     [("set_safety_mode", SafetyModel.noOutput, 0)]),
    ("spoofing_started", {"safety_mode": SafetyModel.allOutput, "power_save_enabled": False}, True, True, []),
  ])
  def test_safety_writes(self, name, health, is_onroad, spoofing_started, writes):
    panda, pm = FakePanda(**health), FakePubMaster()
    assert send_panda_states(pm, panda, TRES, is_onroad, spoofing_started)['ignition_line'] == spoofing_started
    assert panda.writes() == writes
    # published as it was read, before the writes
    ps = published(pm, 'pandaStates').pandaStates[0]
    assert ps.safetyModel.raw == health['safety_mode']
    assert ps.ignitionLine == spoofing_started

  def test_peripheral_state(self):
    panda, pm = FakePanda(), FakePubMaster()
    send_peripheral_state(pm, panda, TRES, panda.health())
    evt = published(pm, 'peripheralState')
    assert evt.valid
    ps = evt.peripheralState
    assert (ps.pandaType, ps.voltage, ps.current, ps.fanSpeedRpm) == ('tres', 12000, 500, 4321)

  def test_forward_logs(self):
    panda = FakePanda()
    with mock.patch.object(pandad, 'cloudlog') as log, \
         mock.patch.object(panda, 'debug_read', side_effect=[b"Register 0x1234 divergent\n", b"hello \xff\n", b""]):
      for _ in range(3):
        forward_logs(panda)
    # register divergent faults are errors, the rest debug. nothing for an empty read
    log.error.assert_called_once_with("Register 0x1234 divergent\n")
    log.debug.assert_called_once_with("hello \ufffd\n")

  def test_can_recv(self):
    # published every time, even when there's nothing. invalid when the panda library found the stream out of sync
    frames = [(1, b'12345678', 0), (0x800, b'1' * 64, 2)]
    panda, pm = FakePanda(), FakePubMaster()
    with mock.patch.object(panda, 'can_recv', side_effect=[frames, [], AssertionError, frames]):
      for valid, expected in ((True, frames), (True, []), (False, []), (True, frames)):
        can_recv(panda, pm)
        evt = published(pm, 'can')
        assert evt.valid == valid and [(c.address, c.dat, c.src) for c in evt.can] == expected


def car_params(safety_model=SafetyModel.allOutput, safety_param=0, alternative_experience=0, not_car=False, configs=1) -> bytes:
  cp = car.CarParams.new_message()
  cp.notCar = not_car
  cp.alternativeExperience = alternative_experience
  safety_configs = cp.init('safetyConfigs', configs)
  for sc in safety_configs:
    sc.safetyModel = safety_model
    sc.safetyParam = safety_param
  return cp.to_bytes()


class TestPandaSafety(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = Params()
    self.panda = FakePanda()
    self.safety = PandaSafety(self.panda)

  def test_onroad_sequence(self):
    s, p = self.safety, self.params

    s.configure_safety_mode(False)
    assert self.panda.writes() == []

    # ELM327 without OBD multiplexing to fingerprint
    s.configure_safety_mode(True)
    assert self.panda.writes() == [("set_safety_mode", SafetyModel.elm327, 1)]

    p.put_bool("ObdMultiplexingEnabled", True, block=True)
    s.configure_safety_mode(True)
    assert self.panda.writes()[-1] == ("set_safety_mode", SafetyModel.elm327, 0)
    # written before it returns, card waits on it
    assert p.get_bool("ObdMultiplexingChanged")

    # nothing until card is done
    p.put_bool("FirmwareQueryDone", True, block=True)
    p.put("CarParams", car_params(SafetyModel.hondaBosch, 5, alternative_experience=-1), block=True)
    s.configure_safety_mode(True)
    assert len(self.panda.writes()) == 2

    # alternative experience first, the panda only takes it outside of a car safety mode
    p.put_bool("ControlsReady", True, block=True)
    s.configure_safety_mode(True)
    assert self.panda.writes()[2:] == [("set_alternative_experience", 0xFFFF), ("set_safety_mode", SafetyModel.hondaBosch, 5)]

    # set once per drive
    n = len(self.panda.writes())
    s.configure_safety_mode(True)
    assert len(self.panda.writes()) == n

    # and again on the next drive
    p.remove("ObdMultiplexingChanged")
    s.configure_safety_mode(False)
    s.configure_safety_mode(True)
    assert self.panda.writes()[n:] == [("set_safety_mode", SafetyModel.elm327, 1), ("set_safety_mode", SafetyModel.elm327, 0),
                                       ("set_alternative_experience", 0xFFFF), ("set_safety_mode", SafetyModel.hondaBosch, 5)]
    assert p.get_bool("ObdMultiplexingChanged")

  @parameterized.expand([("no_safety_config", car_params(configs=0)), ("invalid", b"\x01" * 27)])
  def test_bad_car_params(self, name, cp):
    # noOutput, once
    self.params.put_bool("FirmwareQueryDone", True, block=True)
    self.params.put_bool("ControlsReady", True, block=True)
    self.params.put("CarParams", cp, block=True)
    self.safety.configure_safety_mode(True)
    assert self.panda.writes()[-1] == ("set_safety_mode", SafetyModel.noOutput, 0)
    n = len(self.panda.writes())
    self.safety.configure_safety_mode(True)
    assert len(self.panda.writes()) == n


class TestPeripheralControl(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.panda = FakePanda()
    self.pc = PeripheralControl(self.panda, no_fan_control=False)
    self.sm = messaging.SubMaster(['deviceState', 'cabinCameraState'])
    self.hw = mock.patch.object(pandad.HARDWARE, 'set_ir_power').start()
    self.addCleanup(mock.patch.stopall)

  def update(self, is_onroad, *msgs, pc=None):
    self.sm.update_msgs(time.monotonic(), list(msgs))
    (pc or self.pc).update(self.sm, is_onroad)

  def camera(self, integ_lines, frame_id):
    msg = messaging.new_message('cabinCameraState')
    msg.cabinCameraState.integLines = integ_lines
    msg.cabinCameraState.frameId = frame_id
    return msg

  def ramp(self, n=200, is_onroad=True):
    for i in range(n):
      self.update(is_onroad, self.camera(3000, i))

  def ir_writes(self):
    return [c[1] for c in self.panda.calls if c[0] == "set_ir_power"]

  def test_fan(self):
    # written when it changes, and again every 100 updates
    msg = messaging.new_message('deviceState')
    msg.deviceState.fanSpeedPercentDesired = 40
    for _ in range(101):
      self.update(False, msg)
    assert self.panda.calls.count(("set_fan_power", 40)) == 2

    self.update(False, msg, pc=PeripheralControl(self.panda, no_fan_control=True))
    assert self.panda.count("set_fan_power") == 2

  def test_ir(self):
    # off without the camera, sent once
    self.update(True)
    self.update(True)
    assert self.ir_writes() == [0]

    # the filter ramps up, written when the integer value changes
    self.ramp()
    writes = self.ir_writes()
    ir_pwr = [c.args[0] for c in self.hw.call_args_list]
    # (plus the refresh every 100 updates)
    assert ir_pwr == sorted(ir_pwr) and len(ir_pwr) <= len(set(ir_pwr)) + 2
    assert 0 < ir_pwr[-1] < 100
    assert writes == [x * pandad.MAX_IR_PANDA_VAL // 100 for x in ir_pwr]

    # requested off, offroad only
    Params().put_bool("DisableDriverCameraIR", True, block=True)
    self.update(True, self.camera(3000, 200))
    assert self.ir_writes()[-1] >= writes[-1]
    self.update(False, self.camera(3000, 201))
    assert self.ir_writes()[-1] == 0
    Params().put_bool("DisableDriverCameraIR", False, block=True)
    self.update(False, self.camera(3000, 202))
    assert self.ir_writes()[-1] >= writes[-1]

    # camerad restart resets the filter
    self.update(True, self.camera(1000, 0))
    assert self.ir_writes()[-1] == 0

  def test_ir_driver_view(self):
    # the driver camera preview ramps faster. read when camerad (re)starts
    self.ramp(20)
    assert self.ir_writes()[-1] == 0
    Params().put_bool("IsDriverViewEnabled", True, block=True)
    self.ramp(20)
    assert self.ir_writes()[-1] > 0

  def test_ir_camera_timeout(self):
    # off once camerad has been quiet for a second, e.g. it crashed
    self.ramp()
    on = self.ir_writes()[-1]
    assert on > 0
    t = self.sm.recv_time['cabinCameraState']
    with mock.patch.object(pandad.time, 'monotonic', return_value=t + 0.5):
      self.pc.update(self.sm, True)
    assert self.ir_writes()[-1] == on
    with mock.patch.object(pandad.time, 'monotonic', return_value=t + 1.1):
      self.pc.update(self.sm, True)
    assert self.ir_writes()[-1] == 0 and self.hw.call_args == mock.call(0)

  def test_ir_write_fails(self):
    # logged, and retried at the refresh rate, not on every update
    self.hw.side_effect = OSError
    with mock.patch.object(pandad, 'cloudlog') as log:
      for _ in range(101):
        self.update(True)
    assert self.hw.call_count == 2 and self.ir_writes() == [0, 0]
    assert log.exception.call_count == 2

  def test_ir_off_for_body(self):
    Params().put("CarParams", car_params(not_car=True), block=True)
    self.ramp()
    assert set(self.ir_writes()) == {0}

  def test_invalid_car_params(self):
    # logged once, and IR control carries on
    Params().put("CarParams", b"\x01" * 27, block=True)
    with mock.patch.object(pandad, 'cloudlog') as log:
      self.ramp()
    assert self.ir_writes()[-1] > 0
    log.exception.assert_called_once_with("pandad.invalid_car_params")
