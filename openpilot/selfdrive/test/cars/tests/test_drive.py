import unittest

from openpilot.cereal.services import SERVICE_LIST
from openpilot.selfdrive.test.cars.drive import CAMERAS, PREROLL, RECORD, recorded_loop


class TestServices(unittest.TestCase):
  def test_services_exist(self):
    for service in (*CAMERAS, *RECORD):
      assert service in SERVICE_LIST, service


class TestRecordedLoop(unittest.TestCase):
  def test_next_loop(self):
    # onroad mid loop, the next loop
    assert recorded_loop(100., 16., 108.) == 116.
    # onroad PREROLL before a loop start is still in time for it
    assert recorded_loop(100., 16., 116. - PREROLL) == 116.
    assert recorded_loop(100., 16., 116. - PREROLL + 0.01) == 132.

  def test_onroad_before_clock(self):
    assert recorded_loop(100., 16., 90.) == 100.


if __name__ == "__main__":
  unittest.main()
