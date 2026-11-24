from collections import deque
from functools import cache
import numpy as np
from openpilot.common.params import Params

MAX_TAPS = 1000
HALF_LIFE = 80  # taps, about half a minute of typing, so a new way of holding the device takes over quickly
PRIOR_TAPS = 6  # a key trusts its own taps once it has about this many
CORRECTION_MARGIN = 30  # px, a backspaced tap about as close to the runner-up key as to its own was probably meant for the runner-up


class TouchModel:
  """Learns where this user taps each key. Taps are stored as [key_x, key_y, touch_x, touch_y]."""
  def __init__(self):
    self._params = Params()
    self._taps = deque(self._params.get("KeyboardTouchTaps") or [], maxlen=MAX_TAPS)
    self._typed: list = []  # taps in the current text with their runner-up key, for backspace
    self._fit()

  def target(self, key_x: float, key_y: float) -> tuple[float, float]:
    return self._targets.get((round(key_x), round(key_y))) or tuple(self._layout_target(np.array([key_x, key_y])))

  def add_tap(self, key_pos, touch_pos, runner_up_pos, margin: float):
    tap = [round(key_pos[0]), round(key_pos[1]), round(touch_pos[0]), round(touch_pos[1])]
    self._taps.append(tap)
    self._typed.append((tap, runner_up_pos, margin))
    self._update()

  def erase_tap(self):
    if self._typed:
      tap, runner_up_pos, margin = self._typed.pop()
      if margin < CORRECTION_MARGIN:
        tap[:2] = round(runner_up_pos[0]), round(runner_up_pos[1])
      elif tap in self._taps:
        self._taps.remove(tap)
      self._update()

  def clear_typed(self):
    self._typed = []

  def _update(self):
    self._fit()
    self._params.put("KeyboardTouchTaps", list(self._taps))

  def _layout_target(self, key):
    return key + self._shift + self._stretch * (key - self._center)

  def _fit(self):
    # the whole layout shifts and stretches to fit all taps, then each key moves to the recency weighted mean of its own
    taps = np.array(self._taps, dtype=float).reshape(-1, 4)
    w = 0.5 ** (np.arange(len(taps))[::-1, None] / HALF_LIFE)
    keys, offsets = taps[:, :2], taps[:, 2:] - taps[:, :2]
    self._center = np.sum(w * keys, axis=0) / max(np.sum(w), 1e-3)
    self._shift = np.sum(w * offsets, axis=0) / (np.sum(w) + PRIOR_TAPS)
    rel = keys - self._center
    self._stretch = np.sum(w * rel * (offsets - self._shift), axis=0) / (np.sum(w * rel ** 2, axis=0) + PRIOR_TAPS * 50 ** 2)
    self._targets = {}
    for key in {(k[0], k[1]) for k in self._taps}:
      mask = np.all(keys == key, axis=1)
      target = (PRIOR_TAPS * self._layout_target(np.array(key)) + np.sum(w[mask] * taps[mask, 2:], axis=0)) / (PRIOR_TAPS + np.sum(w[mask]))
      self._targets[key] = tuple(target)


@cache
def get_touch_model() -> TouchModel:
  # one model shared by every keyboard
  return TouchModel()
