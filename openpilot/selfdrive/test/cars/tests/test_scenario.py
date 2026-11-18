import unittest

from opendbc.car.structs import car
from opendbc.car.vin import VIN_UNKNOWN
from openpilot.selfdrive.test.test_cars import gap_report
from openpilot.selfdrive.test.cars.scenario import DASHCAM_ONLY, DROP_LEAD, LEAD, MAX_ENGAGED, TAIL, Window, cached_fw, candidate_windows, pick_window


def logged(vin: str = "1HGCM82633A004352", fw: int = 1, brand: str = "toyota"):
  CP = car.CarParams.new_message()
  CP.brand = brand
  CP.carVin = vin
  CP.init('carFw', fw)
  return CP.as_reader()


class TestScenario(unittest.TestCase):
  def test_cached_fw(self):
    fw, reason = cached_fw(logged())
    assert len(fw) == 1 and reason == ""

  def test_no_cached_fw(self):
    assert cached_fw(None)[0] == []
    assert "VIN is unknown" in cached_fw(logged(vin=VIN_UNKNOWN))[1]
    assert "no firmware" in cached_fw(logged(fw=0))[1]
    assert "mock" in cached_fw(logged(brand="mock"))[1]


def engaged_between(rise: float, fall: float, end: float = 30.) -> list[tuple[float, bool]]:
  return [(i / 2, rise <= i / 2 < fall) for i in range(int(end * 2))]


class TestPickWindow(unittest.TestCase):
  def test_short_stretch(self):
    # a 6 s stretch is enough for the engage limit and the commands after it, the window ends TAIL after it
    assert pick_window(engaged_between(6., 12.), 30.) == (Window(1., 12. + TAIL), 6.)

  def test_too_short(self):
    assert pick_window(engaged_between(6., 10.), 30.) is None

  def test_long_stretch_capped(self):
    assert pick_window(engaged_between(6., 25.), 30.) == (Window(1., 6. + MAX_ENGAGED + TAIL), 6.)

  def test_no_lead_in(self):
    assert pick_window(engaged_between(3., 12.), 30.) is None
    assert pick_window(engaged_between(3., 12.), 30., lead_in=False) is not None


class TestCandidateWindows(unittest.TestCase):
  def test_led_in_first_then_after_drop(self):
    # engaged 2-4 s, then from 12 s: the led-in window first, then the one with the drop at 4 s in its loop
    samples = [(i / 2, 2. <= i / 2 < 4. or 12. <= i / 2 < 20.) for i in range(60)]
    end = 20. + TAIL
    assert candidate_windows(samples, 30.) == [(Window(12. - LEAD, end), 12.), (Window(3.5 - DROP_LEAD, end), 12.)]

  def test_short_lead_in_after_drop(self):
    # the drop at 10 s is less than LEAD before the edge, only the window with the drop in its loop, still LEAD before the edge
    samples = [(i / 2, 2. <= i / 2 < 10. or 12. <= i / 2 < 20.) for i in range(60)]
    assert candidate_windows(samples, 30.) == [(Window(12. - LEAD, 20. + TAIL), 12.)]


class TestGapReport(unittest.TestCase):
  def test_dashcam_only_not_supported(self):
    gaps = [f"mode 1 param 0: {DASHCAM_ONLY}", "mode 2 param 1: no engage edge in any segment"]
    assert gap_report([], gaps)[0].startswith("not covered: covered 0 of 1 supported safety mode/param pairs, 1 more pairs are dashcam only")


if __name__ == "__main__":
  unittest.main()
