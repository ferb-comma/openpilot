import unittest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces

from openpilot.selfdrive.test.cars.fidelity import LOOPS, SLACK, Engage, diverging, engage_problems, selfdrive

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


def car_state(**fields) -> structs.CarState:
  CS = structs.CarState(canValid=True, gearShifter=GearShifter.drive)
  CS.cruiseState.available = True
  for name, value in fields.items():
    setattr(CS, name, value)
  return CS


def press(t: ButtonType, pressed: bool) -> list:
  return [structs.CarState.ButtonEvent(pressed=pressed, type=t)]


def release(t: ButtonType, **fields) -> structs.CarState:
  # the car interface sets buttonEnable on the release of set or resume
  return car_state(buttonEvents=press(t, False), buttonEnable=True, **fields)


def looped(states: list) -> list:
  return states * LOOPS


def op_long_params() -> structs.CarParams:
  # a car where openpilot longitudinal enables on the set and resume buttons
  CP = interfaces["HONDA_CIVIC_2022"].get_non_essential_params("HONDA_CIVIC_2022")
  CP.pcmCruise = False
  return CP


class TestDiverging(unittest.TestCase):
  def test_button_within_slack(self):
    route = [car_state() for _ in range(100)]
    replay = [car_state() for _ in range(100)]
    route[50] = car_state(buttonEvents=press(ButtonType.decelCruise, False))
    replay[50 + SLACK] = car_state(buttonEvents=press(ButtonType.decelCruise, False))
    assert diverging(looped(route), looped(replay)) == {}

  def test_button_moved(self):
    route = [car_state() for _ in range(100)]
    replay = [car_state() for _ in range(100)]
    route[50] = car_state(buttonEvents=press(ButtonType.decelCruise, False))
    replay[50 + SLACK + 1] = car_state(buttonEvents=press(ButtonType.decelCruise, False))
    assert list(diverging(looped(route), looped(replay))) == ['buttonEvents']

  def test_field_lost(self):
    route = [car_state(seatbeltUnlatched=40 <= i < 60) for i in range(100)]
    replay = [car_state() for _ in range(100)]
    assert list(diverging(looped(route), looped(replay))) == ['seatbeltUnlatched']


class TestSelfdrive(unittest.TestCase):
  def test_set_enables(self):
    states = [car_state() for _ in range(20)]
    states[10] = release(ButtonType.decelCruise)
    steps = selfdrive(op_long_params(), states)
    assert not steps[9].enabled and steps[10].enabled

  def test_resume_without_set_speed_blocked(self):
    states = [car_state() for _ in range(20)]
    states[10] = release(ButtonType.accelCruise)
    steps = selfdrive(op_long_params(), states)
    assert not steps[10].enabled
    assert steps[10].enable and 'resumeBlocked' in steps[10].no_entry

  def test_seatbelt_blocks(self):
    states = [car_state(seatbeltUnlatched=True) for _ in range(20)]
    states[10] = release(ButtonType.decelCruise, seatbeltUnlatched=True)
    steps = selfdrive(op_long_params(), states)
    assert not any(s.enabled for s in steps)
    assert 'seatbeltNotLatched' in steps[10].no_entry


class TestEngageProblems(unittest.TestCase):
  def test_engaged(self):
    assert engage_problems(Engage([500], [500], [], False, False, None), 15.) == []

  def test_stays_allowed(self):
    problems = engage_problems(Engage([], [], [], True, True, None), 15.)
    assert problems == ["the panda stays allowed across the loop's wrap, nothing before the edge drops it"]

  def test_carried(self):
    assert engage_problems(Engage([500], [500], [], False, True, None), 15.) == ["openpilot stays engaged across the loop's wrap"]

  def test_blocked(self):
    problems = engage_problems(Engage([500], [None], ["resumeBlocked at 5.00 s"], False, False, None), 15.)
    assert problems == ["openpilot doesn't engage at the panda's edge: resumeBlocked at 5.00 s"]

  def test_late_edge(self):
    assert engage_problems(Engage([1200], [1200], [], False, False, None), 15.)[0].startswith("the panda's edge at 12.00 s leaves less than")


if __name__ == "__main__":
  unittest.main()
