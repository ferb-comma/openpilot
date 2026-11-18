import unittest

from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.car.car_helpers import FRAME_FINGERPRINT

from openpilot.selfdrive.test.cars.replay import fingerprint_frames, presented, value_changes

GM_DBC = "gm_global_a_powertrain_generated"


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

  def test_value_changes_keep_a_short_press(self):
    # a button pressed for two frames changes rarely, the rolling counter every frame
    packer = CANPacker(GM_DBC)
    frames = []
    for i in range(1000):
      buttons = 2 if i in (500, 501) else 1
      frames.append([packer.make_can_msg("ASCMSteeringButton", 0, {"ACCButtons": buttons, "RollingCounter": i % 4})])
    key = (0, 0x1E1)
    changes = value_changes(frames, {key: GM_DBC}, {GM_DBC: DBC(GM_DBC)}, set())
    assert changes == {key: {500, 502}}


if __name__ == "__main__":
  unittest.main()
