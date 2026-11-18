import unittest

from openpilot.selfdrive.test.cars.drive import PREROLL, lead_in_start, recorded_loop
from openpilot.selfdrive.test.cars.scenario import LEAD, Scenario, Window


def scenario(start: float, rise: float, end: float) -> Scenario:
  return Scenario("route", 0, "CAR", 0, 0, Window(start, end), rise, {}, 1, False)


class TestRecordedLoop(unittest.TestCase):
  def test_lead_in_at_loop_start(self):
    s = scenario(10., 10. + LEAD, 26.)
    assert lead_in_start(s) == 0.
    # onroad mid loop, the next loop's lead-in
    assert recorded_loop(s, 100., 16., 108.) == 116.
    # onroad PREROLL before a loop start is still in time for it
    assert recorded_loop(s, 100., 16., 116. - PREROLL) == 116.
    assert recorded_loop(s, 100., 16., 116. - PREROLL + 0.01) == 132.

  def test_longer_lead(self):
    # the edge 12 s into the loop, its lead-in starts 7 s in, so onroad 3 s in still makes this loop
    s = scenario(10., 22., 32.)
    assert lead_in_start(s) == 12. - LEAD
    assert recorded_loop(s, 100., 22., 103.) == 100.
    assert recorded_loop(s, 100., 22., 107.) == 122.

  def test_onroad_before_clock(self):
    s = scenario(10., 10. + LEAD, 26.)
    assert recorded_loop(s, 100., 16., 90.) == 100.


if __name__ == "__main__":
  unittest.main()
