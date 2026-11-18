import unittest

from openpilot.selfdrive.test.cars.comment import comment_body, shard_block

SHARD0 = shard_block(["✅ A x 1  ok  3 s", "❌ B y 2  boom  4 s", "    repeat: SCENARIOS=B ./openpilot/selfdrive/test/test_cars.py   (commit abcdef1234)"],
                     "shard 0/3: 1 passed, 1 failed, 7 gaps (not driven), 1.0 min")
SHARD2 = shard_block(["✅ C z 3  ok  5 s"], "shard 2/3: 1 passed, 0 failed, 7 gaps (not driven), 0.5 min")


class TestComment(unittest.TestCase):
  def test_partial(self):
    body = comment_body({0: SHARD0, 2: SHARD2}, 3, "abcdef1234567")
    assert "cars: 2 passed, 1 failed (1 of 3 shards still running)" in body
    assert "commit abcdef1, 7 gaps (not driven)" in body
    assert "_cars_" in body
    assert body.index("✅ A") < body.index("❌ B") < body.index("repeat: SCENARIOS=B") < body.index("✅ C")
    assert "shard 0/3" not in body

  def test_complete(self):
    body = comment_body({0: SHARD0, 1: "shard 1/3: 0 passed, 0 failed, 7 gaps (not driven), 0.0 min\n", 2: SHARD2}, 3, "abcdef1234567")
    assert "still running" not in body


if __name__ == "__main__":
  unittest.main()
