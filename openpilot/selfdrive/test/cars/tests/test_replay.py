import unittest

from opendbc.car.car_helpers import FRAME_FINGERPRINT

from openpilot.selfdrive.test.cars.replay import fingerprint_frames, presented


def frames_with(n: int, every: int, addr: int = 0x100, bus: int = 0) -> list:
  return [[(addr, b'\x00' * 8, bus)] if i % every == 0 else [] for i in range(n)]


class TestReplay(unittest.TestCase):
  def test_presented_needs_a_frame_every_interval(self):
    assert presented(frames_with(1000, FRAME_FINGERPRINT))[0] == {0x100: 8}
    assert presented(frames_with(1000, FRAME_FINGERPRINT + 1))[0] == {}

  def test_presented_wraps(self):
    frames = frames_with(1000, FRAME_FINGERPRINT)
    frames[0] = []
    assert presented(frames)[0] == {}

  def test_fingerprint_frames(self):
    frames = frames_with(3000, 1) + [[(0x200, b'\x00', 1)]]
    sparse = fingerprint_frames(frames, [(0, 0x100)])
    kept = [i for i, f in enumerate(sparse) if f]
    assert kept[0] == 0 and kept[-1] == 2999
    assert all(b - a <= FRAME_FINGERPRINT for a, b in zip(kept, kept[1:], strict=False))
    assert len(kept) <= 3000 // FRAME_FINGERPRINT + 2
    assert presented(sparse[:3000])[0] == {0x100: 8}


if __name__ == "__main__":
  unittest.main()
