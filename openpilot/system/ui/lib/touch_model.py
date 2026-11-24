from collections import deque
import numpy as np
from openpilot.common.params import Params

TAP_BUFFER_SIZE = 1000  # taps, older ones weigh nothing at this half-life
TAP_HALF_LIFE = 80  # taps, about half a minute of typing, so a new way of holding the device takes over quickly
PRIOR_TAPS = 3  # a key trusts its own taps once it has about this many
CORRECTION_WEIGHT = 2.0  # a backspaced tap retyped as another key says where that key really is
CORRECTION_MAX_DIST = 70  # px, retyped key must be about one key away from the erased tap, else the user changed their mind


class TouchModel:
  """Learns where this user taps for each key. Taps are stored as [key_x, key_y, touch_x, touch_y, weight]."""
  def __init__(self):
    self._params = Params()
    self._taps = deque(self._params.get("KeyboardTouchTaps") or [], maxlen=TAP_BUFFER_SIZE)
    self._fit()
    self.start_session()

  def start_session(self):
    self._session_taps = 0  # taps typed into the current text, the only ones backspace can take back
    self._erased: list[list[float]] = []

  def target(self, key_x: float, key_y: float) -> tuple[float, float]:
    return self._targets.get((round(key_x), round(key_y))) or (self._fit_x(key_x), self._fit_y(key_y))

  def add_tap(self, key_x: float, key_y: float, touch_x: float, touch_y: float):
    if self._erased:
      erased = self._erased.pop()
      if abs(erased[2] - key_x) + abs(erased[3] - key_y) < CORRECTION_MAX_DIST:
        self._taps.append([round(key_x), round(key_y), erased[2], erased[3], CORRECTION_WEIGHT])
    self._taps.append([round(key_x), round(key_y), round(touch_x), round(touch_y), 1.0])
    self._session_taps += 1
    self._fit()
    self._params.put("KeyboardTouchTaps", list(self._taps))

  def erase_tap(self):
    if self._session_taps > 0:
      self._session_taps -= 1
      tap = self._taps.pop()
      while tap[4] != 1.0:  # drop a correction made by the tap being erased along with it
        tap = self._taps.pop()
      self._erased.append(tap)
      self._fit()
      self._params.put("KeyboardTouchTaps", list(self._taps))

  def _fit(self):
    self._targets = {}
    self._fit_x = self._fit_y = lambda k: k
    if not self._taps:
      return

    taps = np.array(self._taps, dtype=float)
    weights = taps[:, 4] * 0.5 ** (np.arange(len(taps))[::-1] / TAP_HALF_LIFE)

    # how the whole layout is shifted and stretched, so keys with few taps still move
    self._fit_x = self._fit_axis(taps[:, 0], taps[:, 2], weights)
    self._fit_y = self._fit_axis(taps[:, 1], taps[:, 3], weights)

    # each key then moves to the weighted mean of its own taps
    for key_x, key_y in {(k[0], k[1]) for k in self._taps}:
      mask = (taps[:, 0] == key_x) & (taps[:, 1] == key_y)
      w = weights[mask]
      x = (PRIOR_TAPS * self._fit_x(key_x) + np.sum(w * taps[mask, 2])) / (PRIOR_TAPS + np.sum(w))
      y = (PRIOR_TAPS * self._fit_y(key_y) + np.sum(w * taps[mask, 3])) / (PRIOR_TAPS + np.sum(w))
      self._targets[(int(key_x), int(key_y))] = (float(x), float(y))

  @staticmethod
  def _fit_axis(keys, touches, weights):
    # weighted line touch = scale * key + offset, pulled towards no change until there are enough taps
    total = np.sum(weights)
    key_mean = np.sum(weights * keys) / total
    touch_mean = np.sum(weights * touches) / total
    prior = PRIOR_TAPS * 50 ** 2
    scale = (np.sum(weights * (keys - key_mean) * (touches - touch_mean)) + prior) / (np.sum(weights * (keys - key_mean) ** 2) + prior)
    shift = (touch_mean - key_mean) * total / (total + PRIOR_TAPS)
    return lambda k: float(key_mean + shift + scale * (k - key_mean))


touch_model: TouchModel | None = None


def get_touch_model() -> TouchModel:
  # one model shared by every keyboard, so they all learn and save the same taps
  global touch_model
  if touch_model is None:
    touch_model = TouchModel()
  return touch_model
