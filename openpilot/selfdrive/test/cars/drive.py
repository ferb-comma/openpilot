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
from openpilot.selfdrive.test.cars.replay import DT, Frame, Replay
from openpilot.selfdrive.test.cars.rules import NO_CAR_MODES, is_event, log_line
from openpilot.selfdrive.test.cars.scenario import LEAD, TAIL, Scenario

# seconds from manager start, ~2x the measured normal onroad time (20-25 s)
ONROAD_LIMIT = 45.
# subscribed this long before the recording window, so the states before its start are known
PREROLL = 0.5
RECORD = ('can', 'sendcan', 'pandaStates', 'selfdriveState', 'carState', 'onroadEvents', 'managerState')
# the panda marks its own tx echoes with src + 128
ECHO = 128


class RigProblem(Exception):
  pass


class DriveFailed(Exception):
  pass


@dataclass
class DriveLog:
  msgs: list
  # logMonoTime the recording window starts at
  start: int


def onroad_missing(sm) -> list[str]:
  # what keeps openpilot from being onroad with selfdrived initialized, empty once it is
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
  # selfdrived blocks engaging until its services are valid, or for up to its init timeout
  if not sm.recv_frame['onroadEvents']:
    missing.append("no onroadEvents")
  elif any(str(event.name) == 'selfdriveInitializing' for event in sm['onroadEvents']):
    missing.append("selfdrived initializing")
  return missing


def lead_in_start(scenario: Scenario) -> float:
  # seconds from a loop's start to the lead-in before its engage edge, LEAD before the edge or the loop's start
  edge = scenario.rise - scenario.window.start
  return edge - min(LEAD, edge)


def recorded_loop(scenario: Scenario, t0: float, period: float, onroad: float) -> float:
  # the start of the first loop of the clock started at t0 whose engage edge has its whole lead-in after onroad,
  # subscribed PREROLL before the lead-in
  return t0 + max(0, math.ceil((onroad + PREROLL - t0 - lead_in_start(scenario)) / period)) * period


def fd_buses(frames: list[Frame]) -> list[int]:
  # the buses with a frame longer than classic CAN's 8 bytes
  return sorted({bus for frame in frames for _, dat, bus in frame if len(dat) > 8})


def manager_died(rc: int) -> DriveFailed:
  return DriveFailed(f"manager died: rc {rc}")


def group_processes(pgid: int) -> dict[int, str]:
  # pid -> name of each live process in a process group
  procs = {}
  for pid in os.listdir('/proc'):
    if not pid.isdigit():
      continue
    try:
      with open(f'/proc/{pid}/stat') as f:
        stat = f.read()
    except OSError:
      continue
    # the name is in parentheses and may hold spaces, then state, ppid and pgrp
    state, _, pgrp = stat[stat.rindex(')') + 2:].split()[:3]
    if int(pgrp) == pgid and state != 'Z':
      procs[int(pid)] = stat[stat.index('(') + 1:stat.rindex(')')]
  return procs


def _signal_group(pgid: int, sig: signal.Signals) -> None:
  try:
    os.killpg(pgid, sig)
  except ProcessLookupError:
    pass


class Drive:
  def __init__(self, up):
    self.up = up
    self._manager: subprocess.Popen | None = None
    self._started = 0.
    # seconds of each phase of the last run, in order
    self.timing: dict[str, float] = {}
    # the processes that took longest to exit in the last stop, and their seconds
    self.slowest_exits: list[tuple[str, float]] = []

  def _up(self, call: str, *args):
    # the UP not answering is the rack's problem
    try:
      return getattr(self.up, call)(*args)
    except Exception as e:
      raise RigProblem(f"rig problem: UP {call} failed: {e!r}") from e

  def _start_manager(self, scenario: Scenario, replay: Replay) -> None:
    set_params_enabled()
    # calibrationd drops the seeded calibration when the previous route was another car, so no previous car
    params = Params()
    params.remove("CarParamsPersistent")
    params.remove("CarParamsPrevRoute")
    # card derives the scenario's car params with the toggle the route drove with
    params.put_bool("AlphaLongitudinalEnabled", scenario.alpha_long)
    # FINGERPRINT also wins over a cached firmware match, so the car model doesn't depend on fuzzy firmware matching
    env = os.environ | {
      'FINGERPRINT': scenario.car_model,
      'TESTING_CLOSET': '1',
      'BLOCK': 'soundd',
    }
    # without the route's firmware card would query the UP, which doesn't answer
    if replay.cache is None:
      env['SKIP_FW_QUERY'] = '1'
    # its own process group, so stopping it leaves no processes for the next scenario, its output goes to ours
    self._started = time.monotonic()
    self._manager = subprocess.Popen(["python", os.path.join(BASEDIR, "openpilot/system/manager/manager.py")], env=env, start_new_session=True)

  def _stop_manager(self) -> None:
    if self._manager is None:
      return
    manager, self._manager = self._manager, None
    rc = manager.poll()
    alive = group_processes(manager.pid)
    interrupted = time.monotonic()
    _signal_group(manager.pid, signal.SIGINT)
    # when each process exits, the manager gives each 5 s after SIGINT before SIGKILL
    exits: dict[str, float] = {}
    while manager.poll() is None and time.monotonic() < interrupted + 60:
      now = group_processes(manager.pid)
      for pid in alive.keys() - now.keys():
        exits[alive[pid]] = time.monotonic() - interrupted
      alive = now
      time.sleep(DT * 5)
    self.slowest_exits = sorted(exits.items(), key=lambda e: -e[1])[:3]
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

  def _wait_onroad(self, sm, can_sock, error_sock, errors: list, buses: set[int]) -> float:
    seen: set[int] = set()
    # the last real error any process logged (not an informational event), and the last card logged
    last_error = ""
    card_error = ""
    end = self._started + ONROAD_LIMIT
    while True:
      self._check_manager()
      sm.update(10)
      for m in messaging.drain_sock(can_sock):
        seen |= {c.src for c in m.can if c.src < ECHO}
      for m in messaging.drain_sock(error_sock):
        errors.append(m)
        if is_event(m.errorLogMessage):
          continue
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

  def _wait(self, until: float, error_sock, errors: list) -> None:
    while time.monotonic() < until:
      self._check_manager()
      errors += messaging.drain_sock(error_sock)
      time.sleep(DT)

  def _record(self, socks: list, end: float, error_sock, errors: list) -> list:
    msgs = []
    while time.monotonic() < end:
      self._check_manager()
      errors += messaging.drain_sock(error_sock)
      for sock in socks:
        msgs += messaging.drain_sock(sock)
      time.sleep(DT)
    errors += messaging.drain_sock(error_sock)
    for sock in socks:
      msgs += messaging.drain_sock(sock)
    self._check_manager()
    return msgs

  def run(self, scenario: Scenario, replay: Replay) -> DriveLog:
    # the UP loops the window from its can_replay call until reset, each loop a lead-in, the engage edge, the engaged stretch and the wrap
    self.timing = {}
    self.slowest_exits = []
    frames = replay.frames
    buses = {bus for frame in frames for _, _, bus in frame}
    sm = messaging.SubMaster(['deviceState', 'carState', 'managerState', 'pandaStates', 'onroadEvents'])
    can_sock = messaging.sub_sock('can', conflate=False)
    # every error since openpilot started, not just the recording window's
    error_sock = messaging.sub_sock('errorLogMessage', conflate=False)
    errors: list = []
    self._start_manager(scenario, replay)
    try:
      self._up('reset')
      self._up('set_ignition', True)
      # the station's lanes start classic, a CAN FD bus gets the data speed the DUT's panda is set to
      panda = self._wait_panda(sm)
      # pandad runs after manager start cleared CarParamsCache, and card reads it on the first CAN, which comes with can_replay
      if replay.cache is not None:
        Params().put("CarParamsCache", replay.cache)
      can_states = (panda.canState0, panda.canState1, panda.canState2)
      for bus in fd_buses(frames):
        self._up('set_can_data_speed_kbps', bus, can_states[bus].canDataSpeed)
      # only once per car, a second can_replay call runs a second loop interleaved with the first, only reset stops them
      self._up('can_replay', frames)
      # the loop's 10 ms clock starts as can_replay returns
      t0 = time.monotonic()
      period = len(frames) * DT
      self.timing['manager start → onroad'] = self._wait_onroad(sm, can_sock, error_sock, errors, buses)
      onroad = time.monotonic()
      loop = recorded_loop(scenario, t0, period, onroad)
      start = loop + lead_in_start(scenario)
      self._wait(start - PREROLL, error_sock, errors)
      socks = [messaging.sub_sock(s, conflate=False) for s in RECORD]
      for sock in socks:
        messaging.drain_sock_raw(sock)
      self.timing['wait for lead-in'] = start - onroad
      # through the window's engaged stretch, ending TAIL before the loop wraps
      end = loop + (scenario.window.end - TAIL - scenario.window.start)
      msgs = self._record(socks, end, error_sock, errors)
      self.timing['record'] = time.monotonic() - start
      return DriveLog(msgs + errors, int(start * 1e9))
    finally:
      stopping = time.monotonic()
      try:
        self._up('reset')
      finally:
        try:
          self._stop_manager()
        finally:
          self.timing['stop'] = time.monotonic() - stopping

  def close(self) -> None:
    try:
      self._stop_manager()
    finally:
      self._up('reset')
