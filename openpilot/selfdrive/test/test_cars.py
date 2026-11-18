#!/usr/bin/env python3
import os
import tempfile
import time
import unittest
from dataclasses import dataclass

from opendbc.car.structs import CarParams

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.test.cars.drive import Drive, DriveFailed, RigProblem
from openpilot.selfdrive.test.cars.replay import BUDGET, build_replay
from openpilot.selfdrive.test.cars.rules import rule_failures, summary
from openpilot.selfdrive.test.cars.scenario import LENGTH, Scenario, load_scenario, load_scenarios

NOT_DRIVEN = "not driven"
RIG_PROBLEM = "rig problem"
SAFETY_MODELS = {v: k for k, v in CarParams.SafetyModel.schema.enumerants.items()}


@dataclass
class DriveResult:
  scenario: Scenario
  status: str = NOT_DRIVEN
  error: str = ""
  failures: list[str] | None = None
  stats: str = ""
  took: float = 0.

  @property
  def name(self) -> str:
    s = self.scenario
    return f"{s.car_model} {SAFETY_MODELS[s.mode]} {s.param}"

  @property
  def first_failure(self) -> str:
    if self.error:
      return self.error.splitlines()[0]
    return self.failures[0] if self.failures else ""

  def line(self) -> str:
    if self.status == "passed":
      return f"✅ {self.name}  {self.stats}  {self.took:.0f} s"
    return f"❌ {self.name}  {self.first_failure}  {self.took:.0f} s"


# every car's engaged route window on a DUT in an ultimate provisioning (UP) station, the UP plays the car.
# setup_class drives each scenario in scenarios.json once and records it, the tests check the recordings.
# SCENARIOS=TOYOTA_COROLLA_TSS2,HONDA_CIVIC only drives those car models
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
    cls.unmatched = [m for m in models if not any(s.car_model == m for s in scenarios)]
    if models:
      scenarios = [s for s in scenarios if s.car_model in models]

    cls.results = [DriveResult(s) for s in scenarios]
    log_dir = tempfile.mkdtemp(prefix="test_cars_")
    print(f"manager logs in {log_dir}", flush=True)
    drive = Drive(UP.remote(), log_dir)
    start = time.monotonic()
    try:
      for result in cls.results:
        cls.drive_one(drive, result)
        print(result.line(), flush=True)
        if result.status == "failed":
          for line in drive.log_tail(result.scenario):
            print(f"    {line}", flush=True)
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
      msgs = drive.run(s, frames)
      result.failures = rule_failures(msgs, s.mode, s.param, warmup=LENGTH)
      result.stats = summary(msgs, warmup=LENGTH)
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
  print(getattr(TestCars, "summary", ""), flush=True)
  raise SystemExit(not program.result.wasSuccessful())
