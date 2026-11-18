import unittest

from opendbc.car.structs import car
from opendbc.car.vin import VIN_UNKNOWN
from openpilot.selfdrive.test.cars.scenario import MAX_ENGAGED, TAIL, Window, cached_fw, pick_window


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


if __name__ == "__main__":
  unittest.main()
