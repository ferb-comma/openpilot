# one scenario on a DUT in an ultimate provisioning (UP) station, the UP plays the scenario's car
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass

import openpilot.cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.selfdrive.test.cars.replay import DT, Frame
from openpilot.selfdrive.test.cars.rules import NO_CAR_MODES, log_line
from openpilot.selfdrive.test.cars.scenario import Scenario

# seconds from manager start, ~2x the measured normal onroad time (20-25 s)
ONROAD_LIMIT = 45.
# long enough for ENGAGE_LIMIT plus a stretch of engaged commands
ENGAGED_RECORD = 10.
# subscribed this long before the recording window, so the states before its start are known
PREROLL = 0.5
RECORD = ('can', 'sendcan', 'pandaStates', 'selfdriveState', 'carState', 'onroadEvents', 'managerState', 'errorLogMessage')
# the panda marks its own tx echoes with src + 128
ECHO = 128


class RigProblem(Exception):
  pass


class DriveFailed(Exception):
  pass


@dataclass
class DriveLog:
  msgs: list
  # seconds from manager start to onroad
  onroad: float
  # logMonoTime and seconds of the recording window
  start: int
  length: float


def onroad_missing(sm) -> list[str]:
  # what keeps openpilot from being onroad, empty once it is
  missing = []
  if not sm['deviceState'].started:
    missing.append("not started")
  if not sm['carState'].canValid:
    missing.append("carState not canValid")
  stopped = [p.name for p in sm['managerState'].processes if p.shouldBeRunning and not p.running]
  if stopped:
    missing.append(f"not running: {', '.join(stopped)}")
  if not len(sm['pandaStates']):
    missing.append("no panda")
  elif sm['pandaStates'][0].safetyModel.raw in NO_CAR_MODES:
    missing.append(f"panda in {sm['pandaStates'][0].safetyModel}")
  return missing


def fd_buses(frames: list[Frame]) -> list[int]:
  # the buses with a frame longer than classic CAN's 8 bytes
  return sorted({bus for frame in frames for _, dat, bus in frame if len(dat) > 8})


def manager_died(rc: int) -> DriveFailed:
  return DriveFailed(f"manager died: rc {rc}")


def _signal_group(pgid: int, sig: signal.Signals) -> None:
  try:
    os.killpg(pgid, sig)
  except ProcessLookupError:
    pass


class Drive:
  def __init__(self, up, log_dir: str):
    self.up = up
    self.log_dir = log_dir
    self._manager: subprocess.Popen | None = None
    self._started = 0.

  def _up(self, call: str, *args):
    # the UP not answering is the rack's problem
    try:
      return getattr(self.up, call)(*args)
    except Exception as e:
      raise RigProblem(f"rig problem: UP {call} failed: {e!r}") from e

  def log_path(self, scenario: Scenario) -> str:
    return os.path.join(self.log_dir, f"{scenario.car_model}_{scenario.mode}_{scenario.param}.log")

  def _start_manager(self, scenario: Scenario) -> None:
    set_params_enabled()
    # calibrationd drops the seeded calibration when the previous route was another car, so no previous car
    params = Params()
    params.remove("CarParamsPersistent")
    params.remove("CarParamsPrevRoute")
    env = os.environ | {
      'FINGERPRINT': scenario.car_model,
      'SKIP_FW_QUERY': '1',
      'TESTING_CLOSET': '1',
      'BLOCK': 'soundd',
    }
    # its own process group, so stopping it leaves no processes for the next scenario
    self._started = time.monotonic()
    with open(self.log_path(scenario), "w") as log:
      self._manager = subprocess.Popen(["python", os.path.join(BASEDIR, "openpilot/system/manager/manager.py")], env=env, start_new_session=True,
                                       stdout=log, stderr=subprocess.STDOUT)

  def _stop_manager(self) -> None:
    if self._manager is None:
      return
    manager, self._manager = self._manager, None
    rc = manager.poll()
    _signal_group(manager.pid, signal.SIGINT)
    try:
      manager.wait(60)
    except subprocess.TimeoutExpired:
      pass
    _signal_group(manager.pid, signal.SIGKILL)
    manager.wait()
    if rc is not None:
      raise manager_died(rc)

  def _check_manager(self) -> None:
    assert self._manager is not None
    rc = self._manager.poll()
    if rc is not None:
      raise manager_died(rc)

  def _wait_panda(self, sm):
    end = time.monotonic() + ONROAD_LIMIT
    while not len(sm['pandaStates']):
      self._check_manager()
      if time.monotonic() > end:
        raise RigProblem("rig problem: panda not answering")
      sm.update(10)
    return sm['pandaStates'][0]

  def _wait_onroad(self, sm, can_sock, error_sock, buses: set[int]) -> float:
    seen: set[int] = set()
    # the last error any process logged, and the last card logged
    last_error = ""
    card_error = ""
    end = self._started + ONROAD_LIMIT
    while True:
      self._check_manager()
      sm.update(10)
      for m in messaging.drain_sock(can_sock):
        seen |= {c.src for c in m.can if c.src < ECHO}
      for m in messaging.drain_sock(error_sock):
        last_error = log_line(m.errorLogMessage)
        if last_error.startswith("card: "):
          card_error = last_error
      missing = onroad_missing(sm)
      if buses - seen:
        missing.append(f"no CAN on bus {sorted(buses - seen)}")
      if not missing:
        return time.monotonic() - self._started
      if time.monotonic() > end:
        error = card_error or last_error
        if error:
          missing.append(f"last error {error}")
        raise DriveFailed(f"not onroad {ONROAD_LIMIT:g} s after manager start: {', '.join(missing)}")

  def _record(self, start: float, end: float) -> list:
    while time.monotonic() < start - PREROLL:
      self._check_manager()
      time.sleep(DT)
    socks = [messaging.sub_sock(s, conflate=False) for s in RECORD]
    for sock in socks:
      messaging.drain_sock_raw(sock)
    msgs = []
    while time.monotonic() < end:
      self._check_manager()
      for sock in socks:
        msgs += messaging.drain_sock(sock)
      time.sleep(DT)
    for sock in socks:
      msgs += messaging.drain_sock(sock)
    self._check_manager()
    return msgs

  def run(self, scenario: Scenario, frames: list[Frame]) -> DriveLog:
    # the UP loops the window from its call until reset, each loop a lead-in, the engage edge, the engaged stretch and the wrap
    buses = {bus for frame in frames for _, _, bus in frame}
    sm = messaging.SubMaster(['deviceState', 'carState', 'managerState', 'pandaStates'])
    can_sock = messaging.sub_sock('can', conflate=False)
    error_sock = messaging.sub_sock('errorLogMessage', conflate=False)
    self._start_manager(scenario)
    try:
      self._up('reset')
      self._up('set_ignition', True)
      # the station's lanes start classic, a CAN FD bus gets the data speed the DUT's panda is set to
      panda = self._wait_panda(sm)
      can_states = (panda.canState0, panda.canState1, panda.canState2)
      for bus in fd_buses(frames):
        self._up('set_can_data_speed_kbps', bus, can_states[bus].canDataSpeed)
      self._up('can_replay', frames)
      # the loop's 10 ms clock starts as can_replay returns
      t0 = time.monotonic()
      period = len(frames) * DT
      onroad = self._wait_onroad(sm, can_sock, error_sock, buses)
      # record from the next loop start through its engage edge and the engaged stretch after it
      start = t0 + math.ceil((time.monotonic() - t0) / period) * period
      end = start + (scenario.rise - scenario.window.start) + ENGAGED_RECORD
      return DriveLog(self._record(start, end), onroad, int(start * 1e9), end - start)
    finally:
      try:
        self._up('reset')
      finally:
        self._stop_manager()

  def close(self) -> None:
    try:
      self._stop_manager()
    finally:
      self._up('reset')
