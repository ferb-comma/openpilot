import numpy as np
from openpilot.common.params import Params

LEARN_RATE = 1 / 20  # taps, about ten seconds of typing to adapt to a new way of holding the device
# per axis: mean key position, mean touch, mean key * touch, mean key ** 2, starting with taps landing on the keys
DEFAULT_STATS = [[268, 268, 268 ** 2 + 140 ** 2, 268 ** 2 + 140 ** 2], [165, 165, 165 ** 2 + 42 ** 2, 165 ** 2 + 42 ** 2]]


class TouchModel:
  """Learns how this user's taps are shifted and stretched from the keys, with running averages per axis."""
  def __init__(self):
    self._params = Params()
    self._stats = np.array(self._params.get("KeyboardTouchStats") or DEFAULT_STATS, dtype=float)
    self._pending = None

  def target(self, key_x: float, key_y: float) -> tuple[float, float]:
    key_mean, touch_mean, key_touch, key_sq = self._stats.T
    stretch = (key_touch - key_mean * touch_mean) / (key_sq - key_mean ** 2)
    return tuple(touch_mean + stretch * (np.array([key_x, key_y]) - key_mean))

  def add_tap(self, key_pos, touch_pos):
    # learn from the previous tap once the next key is typed, so backspaced taps are never learned
    if self._pending is not None:
      key, touch = self._pending
      self._stats += LEARN_RATE * (np.stack([key, touch, key * touch, key ** 2], axis=1) - self._stats)
      self._params.put("KeyboardTouchStats", self._stats.tolist())
    self._pending = np.array(key_pos, dtype=float), np.array(touch_pos, dtype=float)

  def erase_tap(self):
    self._pending = None
