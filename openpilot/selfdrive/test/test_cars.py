#!/usr/bin/env python3
import os
import time
import unittest
from dataclasses import dataclass

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.utils import tabulate
from openpilot.selfdrive.test.cars.drive import Drive, DriveFailed, RigProblem
from openpilot.selfdrive.test.cars.replay import BUDGET, build_replay
from openpilot.selfdrive.test.cars.rules import rule_failures
from openpilot.selfdrive.test.cars.scenario import LENGTH, Scenario, load_scenario, load_scenarios

NOT_DRIVEN = "not driven"
RIG_PROBLEM = "rig problem"


@dataclass
class DriveResult:
  scenario: Scenario
  status: str = NOT_DRIVEN
  error: str = ""
  failures: list[str] | None = None
  took: float = 0.
  replay_call_s: float = 0.

  @property
  def name(self) -> str:
    s = self.scenario
    return f"{s.car_model} mode {s.mode} param {s.param}"

  @property
  def first_failure(self) -> str:
    if self.error:
      return self.error.splitlines()[0]
    return self.failures[0] if self.failures else ""


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
    for gap in gaps:
      print(f"gap: {gap}", flush=True)
    models = os.getenv("SCENARIOS")
    if models is not None:
      scenarios = [s for s in scenarios if s.car_model in models.split(",")]

    cls.results = [DriveResult(s) for s in scenarios]
    drive = Drive(UP.remote())
    start = time.monotonic()
    try:
      for i, result in enumerate(cls.results):
        cls.drive_one(drive, result)
        timing = f"took {result.took:.0f} s, up.can_replay {result.replay_call_s:.1f} s"
        print(f"[{i + 1}/{len(cls.results)}] {result.name}: {result.status} {result.first_failure[:100]} {timing}", flush=True)
        if result.status == RIG_PROBLEM:
          break
    finally:
      drive.close()
    total = time.monotonic() - start

    rows = [(r.name, r.status, r.first_failure[:100], f"{r.took:.0f}", f"{r.replay_call_s:.1f}") for r in cls.results]
    print(tabulate(rows, ("scenario", "result", "first failure", "took (s)", "up.can_replay (s)"), tablefmt="simple_grid"))
    print(f"total drive time {total:.0f} s")

  @staticmethod
  def drive_one(drive: Drive, result: DriveResult) -> None:
    s = result.scenario
    start = time.monotonic()
    try:
      CP, msgs = load_scenario(s)
      frames = build_replay(s, CP, msgs, BUDGET)
      msgs = drive.run(s, frames)
      result.failures = rule_failures(msgs, s.mode, s.param, warmup=LENGTH)
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
    result.replay_call_s = drive.replay_call_s

  def test_driven(self, subtests):
    for result in self.results:
      with subtests.test(scenario=result.name):
        assert result.status != NOT_DRIVEN, "an earlier scenario hit a rig problem"
        assert result.status != RIG_PROBLEM, result.error
        assert not result.error, result.error

  def test_rules(self, subtests):
    for result in self.results:
      if result.failures is None:
        continue
      with subtests.test(scenario=result.name):
        assert not result.failures, "\n".join(result.failures)


if __name__ == "__main__":
  unittest.main()
