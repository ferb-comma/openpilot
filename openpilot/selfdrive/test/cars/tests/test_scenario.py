import unittest

from opendbc.car.structs import car
from opendbc.car.vin import VIN_UNKNOWN
from openpilot.selfdrive.test.cars.scenario import cached_fw


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


if __name__ == "__main__":
  unittest.main()
