#!/usr/bin/env python3
import multiprocessing
import os
import re
import subprocess
import sys
import time
import unittest
from concurrent.futures import Future, ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from dataclasses import dataclass, field

from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.test.cars.drive import Drive, DriveFailed, RigProblem
from openpilot.selfdrive.test.cars.replay import BUDGET, Replay, scenario_replay
from openpilot.selfdrive.test.cars.rules import SAFETY_MODELS, Check, failure_lines, rule_checks, summary
from openpilot.selfdrive.test.cars.scenario import Scenario, load_scenarios

NOT_DRIVEN = "not driven"
RIG_PROBLEM = "rig problem"


def commit() -> str:
  return os.getenv("GIT_COMMIT") or subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=BASEDIR, encoding="utf-8").strip()


def gap_report(scenarios: list[Scenario], gaps: list[str]) -> list[str]:
  pair_gaps = [g for g in gaps if g.startswith("mode ")]
  other = [g for g in gaps if not g.startswith("mode ")]
  pairs = {(s.mode, s.param) for s in scenarios}
  lines = [f"not covered: {len(pair_gaps)} of {len(pairs) + len(pair_gaps)} safety mode/param pairs have no usable route, so no car drives them:"]
  for gap in pair_gaps:
    mode, rest = re.fullmatch(r"mode (\d+) (.*)", gap).groups()
    lines.append(f"  {SAFETY_MODELS[int(mode)]} {rest}")
  return lines + [f"  {gap}" for gap in other]


@dataclass
class DriveResult:
  scenario: Scenario
  # the car model alone does not pick this scenario out
  shared_model: bool = False
  status: str = NOT_DRIVEN
  error: str = ""
  failures: list[str] | None = None
  checks: list[Check] | None = None
  replay: Replay | None = None
  stats: str = ""
  took: float = 0.
  # seconds of each phase of the drive
  timing: dict[str, float] = field(default_factory=dict)
  slowest_exits: list[tuple[str, float]] = field(default_factory=list)

  @property
  def name(self) -> str:
    s = self.scenario
    return f"{s.car_model} {SAFETY_MODELS[s.mode]} {s.param}"

  @property
  def first_failure(self) -> str:
    if self.error:
      return self.error.splitlines()[0]
    return self.failures[0] if self.failures else ""

  @property
  def repeat(self) -> str:
    s = self.scenario
    selector = f"{s.car_model}:{s.param}" if self.shared_model else s.car_model
    return f"repeat: SCENARIOS={selector} ./openpilot/selfdrive/test/test_cars.py   (commit {commit()})"

  @property
  def times(self) -> str:
    onroad = self.timing.get('manager start → onroad')
    recorded = self.timing.get('record')
    onroad_text = f"onroad {onroad:.0f} s, " if onroad is not None else ""
    recorded_text = f"recorded {recorded:.0f} s, " if recorded is not None else ""
    return f"{onroad_text}{recorded_text}{self.took:.0f} s"

  def line(self) -> str:
    if self.status == "passed":
      return f"✅ {self.name}  {self.stats}  {self.times}"
    return f"❌ {self.name}  {self.first_failure}  {self.times}"

  def details(self) -> list[str]:
    # what was driven and the evidence for each rule, under its line
    s = self.scenario
    lines = [f"route {s.route} segment {s.segment}, window {s.window.start:.1f}-{s.window.end:.1f} s, panda engage edge at {s.rise:.1f} s"]
    if self.replay is not None:
      lines.append(self.replay.describe(BUDGET))
    if self.error:
      lines += self.error.splitlines()
    for check in self.checks or []:
      lines += check.lines()
    if self.timing:
      phases = ", ".join(f"{name} {seconds:.1f} s" for name, seconds in self.timing.items())
      lines.append(f"time: {phases}, total {self.took:.0f} s")
    if self.slowest_exits:
      lines.append(f"slowest to exit at stop: {', '.join(f'{name} {seconds:.1f} s' for name, seconds in self.slowest_exits)}")
    return [f"    {line}" for line in lines]


# every car's engaged route window on a DUT in an ultimate provisioning (UP) station, the UP plays the car.
# setup_class drives each scenario in scenarios.json once and records it, the tests check the recordings.
# SCENARIOS=TOYOTA_COROLLA_TSS2,HONDA_CIVIC only drives those car models, HONDA_HRV_3G:8 one param of a car model
class TestCars(OpenpilotTestCase):
  COMMA_HARDWARE_TEST = True

  def setUp(self):
    # the hardware is set up once for the whole run in setup_class
    unittest.TestCase.setUp(self)

  @classmethod
  def setup_class(cls):
    from ultimate_provisioning import UP  # only on a DUT, from its UP's up run or up ssh
    scenarios, gaps = load_scenarios()
    # alfred runs a job with shards = N as N jobs, each drives every Nth scenario
    shard, shards = int(os.getenv("ALFRED_SHARD", "0")), int(os.getenv("ALFRED_SHARDS", "1"))
    cls.shard_label = f"shard {shard}/{shards}: " if "ALFRED_SHARDS" in os.environ else ""
    cls.job = f"cars{shard}" if "ALFRED_SHARDS" in os.environ else "cars"
    if shard == 0:
      print("\n".join(gap_report(scenarios, gaps)), flush=True)
    models = [m.strip() for m in os.getenv("SCENARIOS", "").split(",") if m.strip()]
    cls.unmatched = [m for m in models if not any(m in (s.car_model, f"{s.car_model}:{s.param}") for s in scenarios)]
    car_models = [s.car_model for s in scenarios]
    if models:
      scenarios = [s for s in scenarios if s.car_model in models or f"{s.car_model}:{s.param}" in models]
    cls.nothing_to_drive = bool(scenarios) and not scenarios[shard::shards]
    scenarios = scenarios[shard::shards]
    if cls.shard_label:
      print(f"{cls.shard_label}drives {', '.join(s.car_model for s in scenarios) or 'nothing'}", flush=True)

    cls.results = [DriveResult(s, shared_model=car_models.count(s.car_model) > 1) for s in scenarios]
    drive = Drive(UP.remote())
    start = time.monotonic()
    # builds the next car's replay while this one drives, at the lowest priority so openpilot keeps its cpu
    def new_pool() -> ProcessPoolExecutor:
      return ProcessPoolExecutor(1, mp_context=multiprocessing.get_context("spawn"), initializer=os.nice, initargs=(19,))

    pool = new_pool()
    try:
      replay = None
      for i, result in enumerate(cls.results):
        current = replay or pool.submit(scenario_replay, result.scenario)
        # waits for this build, a worker that died (segfault, oom) breaks the pool and the next build too
        crash = current.exception()
        if isinstance(crash, BrokenProcessPool):
          pool.shutdown(cancel_futures=True)
          pool = new_pool()
        if i + 1 < len(cls.results):
          replay = pool.submit(scenario_replay, cls.results[i + 1].scenario)
        if isinstance(crash, BrokenProcessPool):
          result.status = "failed"
          result.error = f"replay build crashed: {crash}"
        else:
          print(f"=== {result.name} ===", flush=True)
          cls.drive_one(drive, result, current)
        print(result.line(), flush=True)
        print("\n".join(result.details()), flush=True)
        if result.status == "failed":
          print(f"    {result.repeat}", flush=True)
        if result.status == RIG_PROBLEM:
          break
    finally:
      pool.shutdown(cancel_futures=True)
      drive.close()

    passed = sum(r.status == "passed" for r in cls.results)
    not_driven = sum(r.status == NOT_DRIVEN for r in cls.results)
    failed = len(cls.results) - passed - not_driven
    rig = f", {not_driven} not driven after a rig problem" if not_driven else ""
    cls.summary = f"{cls.job}: {passed} passed, {failed} failed{rig}, {(time.monotonic() - start) / 60:.1f} min"

  @staticmethod
  def drive_one(drive: Drive, result: DriveResult, replay: Future) -> None:
    s = result.scenario
    start = time.monotonic()
    drive.timing = {}
    drive.slowest_exits = []
    try:
      result.replay = replay.result()
      log = drive.run(s, result.replay)
      result.checks = rule_checks(log.msgs, s.mode, s.param, s.max_idle, start=log.start)
      result.failures = failure_lines(result.checks)
      result.stats = summary(log.msgs, start=log.start)
      result.status = "passed" if not result.failures else "failed"
    except RigProblem as e:
      result.status = RIG_PROBLEM
      result.error = str(e)
    except DriveFailed as e:
      result.status = "failed"
      result.error = str(e)
    except Exception as e:
      result.status = "failed"
      result.error = repr(e)
    result.took = time.monotonic() - start
    result.timing = dict(drive.timing)
    result.slowest_exits = list(drive.slowest_exits)

  def test_something_driven(self):
    assert not self.unmatched, f"SCENARIOS matched nothing for: {', '.join(self.unmatched)}"
    if self.nothing_to_drive:
      print(f"{self.shard_label}nothing to drive, more shards than scenarios")
      return
    assert self.results, "no scenarios were driven"

  def test_driven(self, subtests):
    for result in self.results:
      with subtests.test(scenario=result.name):
        assert result.status != NOT_DRIVEN, "an earlier scenario hit a rig problem"
        assert not result.error, result.first_failure

  def test_rules(self, subtests):
    for result in self.results:
      if result.failures is None:
        continue
      with subtests.test(scenario=result.name):
        assert not result.failures, result.first_failure


if __name__ == "__main__":
  # unittest's tracebacks scroll the results away, so the results are printed once, last, after its footer (also on stdout)
  program = unittest.main(testRunner=unittest.TextTestRunner(stream=sys.stdout), exit=False)
  sys.stdout.flush()
  sys.stderr.flush()
  for result in getattr(TestCars, "results", []):
    print(result.line())
    if result.status != "passed":
      print(f"    {result.repeat}")
  print(getattr(TestCars, "summary", ""), flush=True)
  raise SystemExit(not program.result.wasSuccessful())
