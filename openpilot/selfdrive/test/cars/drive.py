# one scenario on a DUT in an ultimate provisioning (UP) station, the UP plays the scenario's car
import os
import signal
import subprocess
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.selfdrive.test.cars.replay import DT, Frame
from openpilot.selfdrive.test.cars.rules import NO_CAR_MODES
from openpilot.selfdrive.test.cars.scenario import LENGTH, Scenario

ONROAD_LIMIT = 120.
# after onroad, a car that doesn't get engageable fails, the rig is fine
ENGAGEABLE_LIMIT = 30.
# one full loop of the window, wherever in the loop the recording starts
RECORD_TIME = 2 * LENGTH
RECORD = ('can', 'sendcan', 'pandaStates', 'selfdriveState', 'carState', 'onroadEvents')
# the panda marks its own tx echoes with src + 128
ECHO = 128


class RigProblem(Exception):
  pass


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


def manager_died(rc: int) -> RuntimeError:
  return RuntimeError(f"manager died: rc {rc}")


def onroad_timeout(rc: int | None, panda_seen: bool, quiet_buses: set[int], missing: list[str]) -> Exception:
  # never onroad is the rack's problem only while openpilot is still running
  if rc is not None:
    return manager_died(rc)
  if not panda_seen:
    return RigProblem("rig problem: panda not answering")
  if quiet_buses:
    return RigProblem(f"rig problem: no CAN on bus {sorted(quiet_buses)}")
  return RigProblem(f"rig problem: not onroad after {ONROAD_LIMIT:g} s: {', '.join(missing)}")


def _signal_group(pgid: int, sig: signal.Signals) -> None:
  try:
    os.killpg(pgid, sig)
  except ProcessLookupError:
    pass


class Drive:
  def __init__(self, up):
    self.up = up
    self._manager: subprocess.Popen | None = None
    # how long the last up.can_replay call took, the UP gets every frame in it
    self.replay_call_s = 0.

  def _up(self, call: str, *args):
    # the UP not answering is the rack's problem
    try:
      return getattr(self.up, call)(*args)
    except Exception as e:
      raise RigProblem(f"rig problem: UP {call} failed: {e!r}") from e

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
    self._manager = subprocess.Popen(["python", os.path.join(BASEDIR, "openpilot/system/manager/manager.py")], env=env, start_new_session=True)

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

  def _wait_onroad(self, sm, can_sock, buses: set[int]) -> None:
    seen: set[int] = set()
    panda_seen = False
    end = time.monotonic() + ONROAD_LIMIT
    while True:
      self._check_manager()
      sm.update(10)
      for m in messaging.drain_sock(can_sock):
        seen |= {c.src for c in m.can if c.src < ECHO}
      panda_seen = panda_seen or len(sm['pandaStates']) > 0
      missing = onroad_missing(sm)
      if not missing:
        break
      if time.monotonic() > end:
        assert self._manager is not None
        raise onroad_timeout(self._manager.poll(), panda_seen, buses - seen, missing)
    if buses - seen:
      raise RigProblem(f"rig problem: no CAN on bus {sorted(buses - seen)}")

  def _wait_engageable(self, sm) -> None:
    end = time.monotonic() + ENGAGEABLE_LIMIT
    while not sm['selfdriveState'].engageable:
      self._check_manager()
      if time.monotonic() > end:
        events = [str(e.name) for e in sm['onroadEvents']]
        raise TimeoutError(f"not engageable {ENGAGEABLE_LIMIT:g} s after onroad, events: {events}")
      sm.update(10)
    self._check_manager()

  def _record(self) -> list:
    socks = [messaging.sub_sock(s, conflate=False) for s in RECORD]
    for sock in socks:
      messaging.drain_sock_raw(sock)
    msgs = []
    end = time.monotonic() + RECORD_TIME
    while time.monotonic() < end:
      self._check_manager()
      for sock in socks:
        msgs += messaging.drain_sock(sock)
      time.sleep(DT)
    for sock in socks:
      msgs += messaging.drain_sock(sock)
    self._check_manager()
    return msgs

  def run(self, scenario: Scenario, frames: list[Frame]) -> list:
    # the UP loops the window from its call until reset, each loop a lead-in, the engage edge, the engaged stretch and the wrap
    buses = {bus for frame in frames for _, _, bus in frame}
    sm = messaging.SubMaster(['deviceState', 'carState', 'managerState', 'pandaStates', 'selfdriveState', 'onroadEvents'])
    can_sock = messaging.sub_sock('can', conflate=False)
    self._start_manager(scenario)
    try:
      self._up('reset')
      self._up('set_ignition', True)
      called = time.monotonic()
      self._up('can_replay', frames)
      self.replay_call_s = time.monotonic() - called
      print(f"up.can_replay took {self.replay_call_s:.2f} s")
      self._wait_onroad(sm, can_sock, buses)
      self._wait_engageable(sm)
      return self._record()
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
