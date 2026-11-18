#!/usr/bin/env python3
# pandad: keeps the panda's firmware up to date, then runs the panda <-> openpilot loop in a forked daemon
import collections
import logging
import multiprocessing
import os
import signal
import sys
import threading
import time

from setproctitle import setproctitle, setthreadtitle

from opendbc.car.structs import CarParams, car
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.hardware import HARDWARE
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, Ratekeeper, config_realtime_process, set_realtime_priority
from openpilot.common.swaglog import ForwardingHandler, cloudlog
from openpilot.selfdrive.pandad.panda_safety import PandaSafety
from openpilot.selfdrive.pandad.pandad_api_impl import can_capnp_to_list, can_list_to_can_capnp
from panda import Panda, PandaDFU
from panda.python import LEN_TO_DLC, PANDA_CAN_CNT
from panda.python.spi import PandaSpiHandle, PandaSpiNackResponse

SafetyModel = CarParams.SafetyModel

PANDA_BUS_OFFSET = 4  # sendcan frames from this bus up are for another panda
CAN_SEND_MAX_AGE_NS = 1_000_000_000
# NACK retries for a sendcan batch, from the start of its send. a started batch goes out whole:
# cutting it off once it's stale would drop the frames for the higher buses
CAN_SEND_TIMEOUT_MS = 1000
# for every SPI request: a silent panda raises PandaSpiMissingAck after this, instead of stalling for the default 15 s
PANDA_TIMEOUT_MS = 1000
# a sendcan burst goes out in batches of about this many frames or this many messages, whichever comes first,
# so the main loop gets the SPI bus and the GIL in between. messages count too: some have no frames to send
SEND_BATCH_FRAMES = 256
SEND_BATCH_MSGS = 64

STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM) + ((signal.SIGPWR,) if hasattr(signal, "SIGPWR") else ())  # no SIGPWR on macOS

MAX_IR_PANDA_VAL = 50
CUTOFF_IL = 400
SATURATE_IL = 1000

# capnp lists of enums only take enumerant names. bits above the last fault type are dropped
FAULT_NAMES = [name for name, _ in sorted(log.PandaState.FaultType.schema.enumerants.items(), key=lambda kv: kv[1])]
# Panda.can_health() reports the last error codes as strings
LEC_ERROR_CODE = {name: code for code, name in Panda.LEC_ERROR_CODE.items()}

# set by a stop signal (STOP_SIGNALS)
do_exit: bool = False


# *** CAN ***

def can_send_thread(panda: Panda, stop: threading.Event, fake_send: bool) -> None:
  # the log context is per thread
  cloudlog.bind(daemon="pandad")
  setthreadtitle("pandad_can_send")

  # messages that didn't go out, by reason: too old, invalid, or the panda's CAN TX buffers stayed full. at most a line a second
  dropped: collections.Counter[str] = collections.Counter()
  last_log = 0.0
  def drop(count: int, reason: str) -> None:
    nonlocal last_log
    dropped[reason] += count
    if time.monotonic() - last_log > 1.0:
      cloudlog.error("sendcan: " + "; ".join(f"dropped {n} messages, {r}" for r, n in dropped.items()))
      dropped.clear()
      last_log = time.monotonic()

  try:
    # below the main loop, so heavy sendcan can't starve CAN receive and the heartbeat
    set_realtime_priority(Priority.PANDAD_CAN_SEND)
    sock = messaging.sub_sock('sendcan', timeout=100)

    # run as fast as messages come in
    while not stop.is_set():
      # what's queued goes out together, up to a batch: under a burst that's a few SPI requests for many messages
      nread, msgs, frames = 0, 0, []
      dat = sock.receive()
      while dat is not None:
        nread += 1
        try:
          (log_mono_time, msg_frames), = can_capnp_to_list([dat], 'sendcan')
          msg_frames = [f for f in msg_frames if f[2] < PANDA_BUS_OFFSET]
          if any(len(f[1]) not in LEN_TO_DLC for f in msg_frames):
            raise ValueError("invalid CAN frame length")
        except Exception as e:
          # dropped alone, not with the ones queued with it
          drop(1, f"invalid: {e!r}")
        else:
          # Don't send if older than 1 second or timestamped in the future
          if 0 <= time.monotonic_ns() - log_mono_time < CAN_SEND_MAX_AGE_NS and not fake_send:
            msgs += 1
            frames += msg_frames
          else:
            drop(1, "too old to send")
        if len(frames) >= SEND_BATCH_FRAMES or nread >= SEND_BATCH_MSGS:
          break
        dat = sock.receive(non_blocking=True)

      if frames:
        try:
          # the panda NACKs while its CAN TX buffers are full. that's backpressure, it retries until the timeout
          panda.can_send_many(frames, timeout=CAN_SEND_TIMEOUT_MS)
        except PandaSpiNackResponse:
          drop(msgs, "panda isn't taking CAN frames")
      # give the main loop the SPI bus and the GIL before the next batch. the SPI lock isn't fair
      if frames or nread >= SEND_BATCH_MSGS:
        time.sleep(0.001)
  except Exception:
    # e.g. the panda's gone. the main loop sees the thread is gone and exits, so main() resets the panda
    cloudlog.exception("pandad.can_send_thread_failed")


def can_recv(panda: Panda, pm: messaging.PubMaster) -> None:
  valid = True
  try:
    frames = panda.can_recv()
  except AssertionError:
    # bad checksum. the panda library dropped its buffer and reset the panda's CAN comms
    cloudlog.error("Panda CAN checksum failed")
    frames, valid = [], False
  # published every frame, even when empty
  pm.send('can', can_list_to_can_capnp(frames, 'can', valid))


# *** panda state ***

def fill_panda_state(ps, hw_type: int, health: dict) -> None:
  ps.voltage = health['voltage']
  ps.current = health['current']
  ps.uptime = health['uptime']
  ps.safetyTxBlocked = health['safety_tx_blocked']
  ps.safetyRxInvalid = health['safety_rx_invalid']
  ps.ignitionLine = health['ignition_line']
  ps.ignitionCan = health['ignition_can']
  ps.controlsAllowed = health['controls_allowed']
  ps.txBufferOverflow = health['tx_buffer_overflow']
  ps.rxBufferOverflow = health['rx_buffer_overflow']
  ps.pandaType = hw_type
  ps.safetyModel = health['safety_mode']
  ps.safetyParam = health['safety_param']
  ps.faultStatus = health['fault_status']
  ps.powerSaveEnabled = health['power_save_enabled']
  ps.heartbeatLost = health['heartbeat_lost']
  # uint16 on the panda, Int16 in the schema
  alternative_experience = health['alternative_experience']
  ps.alternativeExperience = alternative_experience - 0x10000 if alternative_experience >= 0x8000 else alternative_experience
  ps.harnessStatus = health['car_harness_status']
  ps.interruptLoad = health['interrupt_load']
  ps.fanPower = health['fan_power']
  ps.safetyRxChecksInvalid = health['safety_rx_checks_invalid']
  ps.nmiReset = health['nmi_reset']
  ps.hardfaultReset = health['hardfault_reset']
  ps.spiErrorCount = health['spi_error_count']
  ps.sbu1Voltage = health['sbu1_voltage_mV'] / 1000.0
  ps.sbu2Voltage = health['sbu2_voltage_mV'] / 1000.0
  ps.soundOutputLevel = health['sound_output_level']

  faults = health['faults']
  ps.faults = [name for bit, name in enumerate(FAULT_NAMES) if faults & (1 << bit)]


def fill_panda_can_state(cs, can_health: dict) -> None:
  cs.busOff = bool(can_health['bus_off'])
  cs.busOffCnt = can_health['bus_off_cnt']
  cs.errorWarning = bool(can_health['error_warning'])
  cs.errorPassive = bool(can_health['error_passive'])
  cs.lastError = LEC_ERROR_CODE[can_health['last_error']]
  cs.lastStoredError = LEC_ERROR_CODE[can_health['last_stored_error']]
  cs.lastDataError = LEC_ERROR_CODE[can_health['last_data_error']]
  cs.lastDataStoredError = LEC_ERROR_CODE[can_health['last_data_stored_error']]
  cs.receiveErrorCnt = can_health['receive_error_cnt']
  cs.transmitErrorCnt = can_health['transmit_error_cnt']
  cs.totalErrorCnt = can_health['total_error_cnt']
  cs.totalTxLostCnt = can_health['total_tx_lost_cnt']
  cs.totalRxLostCnt = can_health['total_rx_lost_cnt']
  cs.totalTxCnt = can_health['total_tx_cnt']
  cs.totalRxCnt = can_health['total_rx_cnt']
  cs.totalFwdCnt = can_health['total_fwd_cnt']
  cs.canSpeed = can_health['can_speed']
  cs.canDataSpeed = can_health['can_data_speed']
  cs.canfdEnabled = bool(can_health['canfd_enabled'])
  cs.brsEnabled = bool(can_health['brs_enabled'])
  cs.canfdNonIso = bool(can_health['canfd_non_iso'])
  cs.irq0CallRate = can_health['irq0_call_rate']
  cs.irq1CallRate = can_health['irq1_call_rate']
  cs.irq2CallRate = can_health['irq2_call_rate']
  cs.canCoreResetCnt = can_health['can_core_reset_count']


def send_panda_states(pm: messaging.PubMaster, panda: Panda, hw_type: int, is_onroad: bool, spoofing_started: bool) -> dict:
  """Publishes pandaStates and keeps the panda's safety mode and power save in line with ignition. Returns the health."""
  health = panda.health()
  can_healths = [panda.can_health(i) for i in range(PANDA_CAN_CNT)]

  if spoofing_started:
    health['ignition_line'] = True
  ignition = health['ignition_line'] or health['ignition_can']

  # Make sure CAN buses are live: openpilot can't talk to the car while the panda is SILENT
  if health['safety_mode'] == SafetyModel.silent:
    panda.set_safety_mode(SafetyModel.noOutput)

  power_save_desired = not ignition
  if health['power_save_enabled'] != power_save_desired:
    panda.set_power_save(power_save_desired)

  # set safety mode to NO_OUTPUT when car is off or we're not onroad. ELM327 is an alternative if we want to leverage athenad/connect
  should_close_relay = not ignition or not is_onroad
  if should_close_relay and health['safety_mode'] != SafetyModel.noOutput:
    panda.set_safety_mode(SafetyModel.noOutput)

  msg = messaging.new_message('pandaStates', 1, valid=True)
  ps = msg.pandaStates[0]
  fill_panda_state(ps, hw_type, health)
  for i, can_health in enumerate(can_healths):
    fill_panda_can_state(ps.init(f'canState{i}'), can_health)
  pm.send('pandaStates', msg)
  return health


def send_peripheral_state(pm: messaging.PubMaster, panda: Panda, hw_type: int, health: dict) -> None:
  msg = messaging.new_message('peripheralState', valid=True)
  ps = msg.peripheralState
  ps.pandaType = hw_type
  ps.voltage = health['voltage']
  ps.current = health['current']
  ps.fanSpeedRpm = panda.get_fan_rpm()
  pm.send('peripheralState', msg)


class PeripheralControl:
  """Fan and IR LEDs for the driver camera."""

  def __init__(self, panda: Panda, no_fan_control: bool):
    self.panda = panda
    self.no_fan_control = no_fan_control
    self.params = Params()

    self.prev_fan_speed = 999
    self.ir_pwr = 0
    self.prev_ir_pwr = 999
    self.prev_frame_id: int | None = None
    self.driver_view = False
    self.not_car = False
    self.not_car_checked = False

    self.integ_lines_filter = FirstOrderFilter(0, 30.0, 0.05)
    self.integ_lines_filter_driver_view = FirstOrderFilter(0, 5.0, 0.05)

  def update(self, sm: messaging.SubMaster, is_onroad: bool) -> None:
    if sm.updated['deviceState'] and not self.no_fan_control:
      fan_speed = sm['deviceState'].fanSpeedPercentDesired
      if fan_speed != self.prev_fan_speed or sm.frame % 100 == 0:
        self.panda.set_fan_power(fan_speed)
        self.prev_fan_speed = fan_speed

    if sm.updated['cabinCameraState']:
      cs = sm['cabinCameraState']

      # reset the filters when camerad restarts
      if self.prev_frame_id is None or cs.frameId < self.prev_frame_id:
        self.integ_lines_filter.x = 0
        self.integ_lines_filter_driver_view.x = 0
        self.driver_view = self.params.get_bool("IsDriverViewEnabled")
      self.prev_frame_id = cs.frameId

      f = self.integ_lines_filter_driver_view if self.driver_view else self.integ_lines_filter
      cur_integ_lines = int(f.update(cs.integLines))

      # integer math, so a steady input gives a steady value
      if cur_integ_lines <= CUTOFF_IL:
        self.ir_pwr = 0
      elif cur_integ_lines > SATURATE_IL:
        self.ir_pwr = 100
      else:
        self.ir_pwr = 100 * (cur_integ_lines - CUTOFF_IL) // (SATURATE_IL - CUTOFF_IL)

    # Disable IR on input timeout or when requested offroad
    if time.monotonic() - sm.recv_time['cabinCameraState'] > 1.0 or (not is_onroad and self.params.get_bool("DisableDriverCameraIR")):
      self.ir_pwr = 0

    # turn off IR leds if body
    if not self.not_car_checked and is_onroad:
      cp_bytes = self.params.get("CarParams")
      if cp_bytes:
        # checked once, even if it doesn't parse
        self.not_car_checked = True
        try:
          with car.CarParams.from_bytes(cp_bytes) as CP:
            self.not_car = CP.notCar
        except Exception:
          cloudlog.exception("pandad.invalid_car_params")
    if self.not_car:
      self.ir_pwr = 0

    if self.ir_pwr != self.prev_ir_pwr or sm.frame % 100 == 0:
      # set first, so a failing write is retried at the refresh rate, not every update
      self.prev_ir_pwr = self.ir_pwr
      self.panda.set_ir_power(self.ir_pwr * MAX_IR_PANDA_VAL // 100)
      try:
        HARDWARE.set_ir_power(self.ir_pwr)
      except OSError:
        cloudlog.exception("pandad.set_ir_power_failed")


def forward_logs(panda: Panda) -> None:
  dbg = panda.debug_read().decode('utf-8', errors='replace')
  if dbg:
    # Log register divergent faults as errors
    if "Register 0x" in dbg:
      cloudlog.error(dbg)
    else:
      cloudlog.debug(dbg)


# *** main loop ***

def pandad_run(panda: Panda) -> None:
  """Runs until a stop signal. Any exception, e.g. the panda going silent, ends the daemon and main() resets the panda."""
  no_fan_control = "NO_FAN_CONTROL" in os.environ
  spoofing_started = "STARTED" in os.environ
  fake_send = "FAKESEND" in os.environ

  # read once at connect. unknown (0) if the panda doesn't say
  hw_type = int.from_bytes(panda.get_type()[:1], 'little')

  rk = Ratekeeper(100, print_delay_threshold=None)
  sm = messaging.SubMaster(['selfdriveState', 'deviceState', 'cabinCameraState'])
  pm = messaging.PubMaster(['can', 'pandaStates', 'peripheralState'])
  panda_safety = PandaSafety(panda)
  peripherals = PeripheralControl(panda, no_fan_control)
  engaged = False
  is_onroad = False

  # sendcan is event driven, the moment it's published
  stop = threading.Event()
  send_thread = threading.Thread(target=can_send_thread, args=(panda, stop, fake_send), name="pandad_can_send", daemon=True)
  send_thread.start()

  try:
    # receive CAN first, then the lower priority panda and peripheral state
    while not do_exit:
      # it only stops on an error it couldn't handle
      if not send_thread.is_alive():
        raise RuntimeError("can send thread died")

      can_recv(panda, pm)

      # Process peripheral state at 20 Hz
      if rk.frame % 5 == 0:
        sm.update(0)
        engaged = sm.all_alive(['selfdriveState']) and sm.all_valid(['selfdriveState']) and sm['selfdriveState'].enabled
        if sm.updated['deviceState']:
          is_onroad = sm['deviceState'].started
        peripherals.update(sm, is_onroad)

      # Process panda state at 10 Hz
      if rk.frame % 10 == 0:
        health = send_panda_states(pm, panda, hw_type, is_onroad, spoofing_started)
        # only after a publish: no heartbeat when the panda can't be read
        panda.send_heartbeat(engaged)
        panda_safety.configure_safety_mode(is_onroad)

        # Send out peripheralState at 2 Hz
        if rk.frame % 50 == 0:
          send_peripheral_state(pm, panda, hw_type, health)

      # Forward logs from panda to cloudlog at 10 Hz, offset from the panda state
      if rk.frame % 10 == 5:
        forward_logs(panda)

      rk.keep_time()

    # Close relay on exit to prevent a fault
    if is_onroad and not engaged:
      panda.set_safety_mode(SafetyModel.noOutput)
  finally:
    stop.set()
    # it's out within the 100 ms receive timeout, or once the send in progress finishes or hits CAN_SEND_TIMEOUT_MS
    send_thread.join(timeout=0.1 + CAN_SEND_TIMEOUT_MS / 1000 + 0.4)


# *** firmware and connection ***

def panda_ready() -> bool:
  """Is the panda up and running the firmware we ship?"""
  try:
    # a healthy panda answers on SPI within ~200ms of reset
    if not Panda.wait_for_panda(None, timeout=1):
      cloudlog.warning("no panda found")
      return False
    with Panda(cli=False) as p:
      if p.bootstub:
        cloudlog.warning(f"panda stuck in bootstub, version {p.get_version()}")
        return False
      if not p.up_to_date():
        cloudlog.warning(f"panda firmware out of date, version {p.get_version()}")
        return False
      return True
  except Exception:
    # e.g. SPI protocol mismatch with old firmware
    cloudlog.exception("pandad.panda_check_failed")
    return False


def flash_panda() -> None:
  """Reflash bootstub + app from the ST bootloader. Works from any state the panda can be in."""
  cloudlog.warning("flashing panda")
  HARDWARE.recover_internal_panda()
  assert Panda.wait_for_dfu(None, timeout=5), "panda didn't enter DFU"
  with PandaDFU(None) as dfu:
    dfu.recover()

  assert Panda.wait_for_panda(None, timeout=5), "panda didn't boot into bootstub"
  with Panda(cli=False) as p:
    p.flash()
  cloudlog.warning("done flashing panda")


def connect(serial: str) -> Panda:
  # pandad sends the heartbeat, so the checks stay enabled
  panda = Panda(serial, cli=False, disable_checks=False)

  # common panda config
  if "BOARDD_LOOPBACK" in os.environ:
    panda.set_can_loopback(True)

  for i in range(PANDA_CAN_CNT):
    panda.set_canfd_auto(i, True)

  if not panda.up_to_date() and "BOARDD_SKIP_FW_CHECK" not in os.environ:
    raise RuntimeError("panda firmware out of date")

  return panda


def run() -> None:
  """The daemon, in its own process: connects to the panda and runs until a stop signal, or an exception."""
  cloudlog.warning("starting pandad")

  # for the SPI requests that don't set their own, from the Panda constructor's to health and the heartbeat
  PandaSpiHandle.timeout = PANDA_TIMEOUT_MS

  # only the internal (SPI) panda
  serials = Panda.spi_list()
  if not serials:
    cloudlog.warning("no pandas found, exiting")
    return

  # a failure raises, and main() resets the panda before the next try
  cloudlog.warning(f"connecting to panda: {serials[0]}")
  panda = connect(serials[0])
  cloudlog.warning("connected to panda")

  # only this process is realtime: main() resets and flashes the panda, which busy-polls SPI for seconds.
  # set before the send thread starts, which inherits it
  config_realtime_process(3, Priority.PANDAD)
  # a thread waiting for the GIL gets it within 0.5 ms, instead of the default 5 ms: the main loop when it wakes up
  # while the send thread is busy, the send thread when sendcan arrives. a lower RT priority doesn't give up the GIL
  sys.setswitchinterval(0.0005)
  pandad_run(panda)


def daemon() -> None:
  # for pidof, and so test_onroad tells its CPU usage apart from main()'s
  setproctitle("pandad")
  try:
    run()
  except Exception:
    cloudlog.exception("pandad.uncaught_exception")
    # main() logs the exit code and resets the panda
    sys.exit(1)


def main() -> None:
  # signal pandad to close the relay and exit. the forked daemon has this handler too
  def signal_handler(signum, frame):
    global do_exit
    cloudlog.info(f"Caught signal {signum}, exiting")
    do_exit = True
  for sig in STOP_SIGNALS:
    signal.signal(sig, signal_handler)

  # the panda library's SPI errors go to swaglog
  logging.getLogger('panda').handlers = [ForwardingHandler(cloudlog)]

  # check health for lost heartbeat
  try:
    for s in Panda.list():
      with Panda(s) as p:
        health = p.health()
        if p.is_internal() and health["heartbeat_lost"]:
          Params().put_bool("PandaHeartbeatLost", True, block=True)
          cloudlog.event("heartbeat lost", deviceState=health)
  except Exception:
    cloudlog.exception("pandad.uncaught_exception")

  count = 0
  while not do_exit:
    try:
      cloudlog.event("pandad.flash_and_connect", count=count)
      count += 1

      HARDWARE.reset_internal_panda()
      if not panda_ready():
        flash_panda()
        assert panda_ready(), "panda not ready after flashing"

      # run real pandad, in its own process so it's restarted after any crash, even a native abort or a SIGKILL
      proc = multiprocessing.get_context('fork').Process(target=daemon, name="pandad")
      proc.start()
      # pass a stop signal on, again until it exits: a signal that lands before the new process
      # finishes its after-fork setup is dropped. exitcode reaps it first, so a reused pid isn't signaled
      while proc.exitcode is None:
        if do_exit:
          os.kill(proc.pid, signal.SIGINT)
        proc.join(0.1)
      if proc.exitcode != 0:
        cloudlog.error(f"pandad exited with {proc.exitcode}")
    except Exception:
      cloudlog.exception("pandad.uncaught_exception")


if __name__ == "__main__":
  main()
