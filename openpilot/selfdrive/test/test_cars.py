#!/usr/bin/env python3
import os
import subprocess
import tempfile
import time
import unittest
from dataclasses import dataclass

from opendbc.car.structs import CarParams

from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.test.cars.drive import Drive, DriveFailed, RigProblem
from openpilot.selfdrive.test.cars.replay import BUDGET, build_replay
from openpilot.selfdrive.test.cars.rules import rule_failures, summary
from openpilot.selfdrive.test.cars.scenario import LENGTH, Scenario, load_scenario, load_scenarios

NOT_DRIVEN = "not driven"
RIG_PROBLEM = "rig problem"
SAFETY_MODELS = {v: k for k, v in CarParams.SafetyModel.schema.enumerants.items()}


def commit() -> str:
  return os.getenv("GIT_COMMIT") or subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=BASEDIR, encoding="utf-8").strip()


@dataclass
class DriveResult:
  scenario: Scenario
  # the car model alone does not pick this scenario out
  shared_model: bool = False
  status: str = NOT_DRIVEN
  error: str = ""
  failures: list[str] | None = None
  stats: str = ""
  took: float = 0.
  # seconds from manager start to onroad
  onroad: float | None = None

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
    onroad = f"onroad {self.onroad:.0f} s, " if self.onroad is not None else ""
    return f"{onroad}{self.took:.0f} s"

  def line(self) -> str:
    if self.status == "passed":
      return f"✅ {self.name}  {self.stats}  {self.times}"
    return f"❌ {self.name}  {self.first_failure}  {self.times}"


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
    print(f"{len(gaps)} gaps (not driven):", flush=True)
    for gap in gaps:
      print(f"  {gap}", flush=True)
    models = [m.strip() for m in os.getenv("SCENARIOS", "").split(",") if m.strip()]
    cls.unmatched = [m for m in models if not any(m in (s.car_model, f"{s.car_model}:{s.param}") for s in scenarios)]
    car_models = [s.car_model for s in scenarios]
    if models:
      scenarios = [s for s in scenarios if s.car_model in models or f"{s.car_model}:{s.param}" in models]

    cls.results = [DriveResult(s, shared_model=car_models.count(s.car_model) > 1) for s in scenarios]
    log_dir = tempfile.mkdtemp(prefix="test_cars_")
    print(f"manager logs in {log_dir}", flush=True)
    drive = Drive(UP.remote(), log_dir)
    start = time.monotonic()
    try:
      for result in cls.results:
        cls.drive_one(drive, result)
        print(result.line(), flush=True)
        if result.status == "failed":
          print(f"    {result.repeat}", flush=True)
        if result.status == RIG_PROBLEM:
          break
    finally:
      drive.close()

    passed = sum(r.status == "passed" for r in cls.results)
    not_driven = sum(r.status == NOT_DRIVEN for r in cls.results)
    failed = len(cls.results) - passed - not_driven
    rig = f", {not_driven} not driven after a rig problem" if not_driven else ""
    cls.summary = f"{passed} passed, {failed} failed{rig}, {len(gaps)} gaps (not driven), {(time.monotonic() - start) / 60:.1f} min"
    print(cls.summary, flush=True)

  @staticmethod
  def drive_one(drive: Drive, result: DriveResult) -> None:
    s = result.scenario
    start = time.monotonic()
    try:
      CP, msgs = load_scenario(s)
      frames = build_replay(s, CP, msgs, BUDGET)
      log = drive.run(s, frames)
      result.onroad = log.onroad
      result.failures = rule_failures(log.msgs, s.mode, s.param, warmup=LENGTH)
      result.stats = summary(log.msgs, warmup=LENGTH)
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

  def test_something_driven(self):
    assert not self.unmatched, f"SCENARIOS matched nothing for: {', '.join(self.unmatched)}"
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
  # unittest's tracebacks scroll the results away, so print them again last
  program = unittest.main(exit=False)
  for result in getattr(TestCars, "results", []):
    print(result.line())
    if result.status != "passed":
      print(f"    {result.repeat}")
  print(getattr(TestCars, "summary", ""), flush=True)
  raise SystemExit(not program.result.wasSuccessful())
