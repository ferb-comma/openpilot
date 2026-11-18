import json
import math
import multiprocessing
import os
from collections.abc import Iterable
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from pathlib import Path

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.logreader import LogReader as OpendbcLogReader
from opendbc.car.tests.routes import CarTestRoute, routes as test_routes
from opendbc.car.tests.test_models import TestCarModelBase, get_cached_segment
from opendbc.car.vin import VIN_UNKNOWN
from opendbc.safety.tests.libsafety import libsafety_py

from openpilot.tools.lib.logreader import LogReader

LEAD = 5.
LENGTH = 30.
MIN_ENGAGED = 10.
SCENARIOS = Path(__file__).parent / "scenarios.json"


@dataclass(frozen=True)
class Window:
  start: float
  end: float


@dataclass(frozen=True)
class Scenario:
  route: str
  segment: int
  car_model: str
  mode: int
  param: int
  window: Window
  # the panda's engage edge, in segment seconds
  rise: float
  # (bus, addr) of every message the panda checks -> whether it checks its counter, read when the list is built
  rx_checks: dict[tuple[int, int], bool]
  # the longest run of empty sendcan the car's controller has while engaged in the route, its command cadence
  max_idle: int


def _first_can(msgs: list) -> int | None:
  for m in msgs:
    if m.which() == 'can':
      return m.logMonoTime
  return None


def engaged_samples(msgs: Iterable) -> list[tuple[float, bool]]:
  samples = []
  msgs = list(msgs)
  first = _first_can(msgs)
  if first is None:
    return []
  for m in msgs:
    which = m.which()
    if which == 'pandaStates' and len(m.pandaStates):
      allowed = m.pandaStates[0].controlsAllowed
    elif which == 'pandaStateDEPRECATED':
      allowed = m.pandaStateDEPRECATED.controlsAllowed
    else:
      continue
    samples.append(((m.logMonoTime - first) / 1e9, bool(allowed)))
  return samples


def sendcan_samples(msgs: list) -> list[tuple[float, bool]]:
  # each logged sendcan and whether it carries a command, in the time base of engaged_samples
  first = _first_can(msgs)
  if first is None:
    return []
  return [((m.logMonoTime - first) / 1e9, len(m.sendcan) > 0) for m in msgs if m.which() == 'sendcan']


def max_idle(sendcan: list[tuple[float, bool]], samples: list[tuple[float, bool]], window: Window, rise: float) -> int | None:
  # the longest run of empty sendcan between two commands in the window's engaged stretch, None without two commands
  stop = window.end
  for t, allowed in samples:
    if t > rise and not allowed:
      stop = min(stop, t)
      break
  longest = None
  run = None
  for t, sent in sendcan:
    if t < rise or t > stop:
      continue
    if sent:
      if run is not None:
        longest = max(longest or 0, run)
      run = 0
    elif run is not None:
      run += 1
  return longest


def can_span(msgs: Iterable) -> float:
  times = [m.logMonoTime for m in msgs if m.which() == 'can']
  return (max(times) - min(times)) / 1e9 if times else 0.


def pick_window(samples: list[tuple[float, bool]], can_end: float, lead_in: bool = True) -> tuple[Window, float] | None:
  # the window and its engage edge, the window starts LEAD before the edge with the panda not allowed throughout,
  # so the UP's loop of it is a lead-in, the edge, the engaged stretch and the wrap
  # lead_in=False skips the lead-in check, to tell a missing lead-in from a missing stretch
  best = None
  best_len = 0.
  rise = None
  # the last allowed sample before the current stretch's rise, and before now
  allowed_before_rise = -math.inf
  last_allowed = -math.inf
  seen_down = False
  # a stretch starts where the panda goes from not allowed to allowed, the rack's panda starts disengaged
  # and has to engage from the replay, so one engaged since the first sample has no usable start
  # a stretch ends at the first sample that is not engaged, or at the last sample
  for i, (t, allowed) in enumerate(samples):
    if allowed and rise is None and seen_down:
      rise, allowed_before_rise = t, last_allowed
    if allowed:
      last_allowed = t
    else:
      seen_down = True
    last = i == len(samples) - 1
    if rise is not None and (not allowed or last):
      length = (can_end if allowed else t) - rise
      start = rise - LEAD
      fits = length >= MIN_ENGAGED and start + LENGTH <= can_end
      led_in = start >= 0. and allowed_before_rise < start
      if fits and (led_in or not lead_in) and length > best_len:
        best, best_len = (start, rise), length
      rise = None
  if best is None:
    return None
  start, rise = best
  return Window(start, start + LENGTH), rise


def safety_pair(CP: structs.CarParams) -> tuple[int, int]:
  cfg = CP.safetyConfigs[-1]
  return int(cfg.safetyModel.raw), int(cfg.safetyParam)


def rx_checks(mode: int, param: int) -> dict[tuple[int, int], bool]:
  safety = libsafety_py.libsafety
  if safety.set_safety_hooks(mode, param) != 0:
    raise ValueError(f"libsafety has no safety mode {mode} param {param}")
  cfg = safety.safety_a.setup.current_safety_config
  checks = {}
  for i in range(cfg.rx_checks_len):
    for m in cfg.rx_checks[i].msg:
      if m.addr:
        checks[(m.bus, m.addr)] = m.max_counter > 0 and not m.ignore_counter
  return checks


Fingerprint = dict[int, dict[int, int]]


def dut_params(car_model: str, fingerprint: Fingerprint, car_fw: list) -> structs.CarParams:
  # as card derives them with FINGERPRINT set, the alpha long toggle unset and not a release branch
  return interfaces[car_model].get_params(car_model, fingerprint, car_fw, False, False, docs=False)


def logged_car_params(msgs: list):
  # the route's last logged car params, as the opendbc tests take its firmware
  return next((m.carParams for m in reversed(msgs) if m.which() == 'carParams'), None)


def cached_fw(logged) -> tuple[list, str]:
  # the firmware card takes from a CarParamsCache of the route's car params instead of querying (car_helpers fingerprint()),
  # and why it takes none. REPLAY would lift the VIN check, but it also changes how selfdrived engages and card's clock
  if logged is None:
    return [], "route logged no car params"
  if logged.brand == "mock":
    return [], "route's car params are mock"
  if not len(logged.carFw):
    return [], "route logged no firmware"
  if logged.carVin == VIN_UNKNOWN:
    return [], "route's VIN is unknown, card doesn't use cached firmware without one"
  return list(logged.carFw), ""


@dataclass
class RouteCar:
  car_model: str
  # the route's car params, and the DUT's on the route's fingerprint and the firmware it takes from the cache
  CP: structs.CarParams
  dut_CP: structs.CarParams
  fingerprint: Fingerprint
  dut_fw: list
  # why the DUT takes no firmware from the cache, empty if it does
  no_fw: str
  # the route's logged car params for CarParamsCache, None if card wouldn't use them
  cache: bytes | None
  # the route drove with openpilot longitudinal
  alpha_long: bool

  def dut(self, fingerprint: Fingerprint) -> structs.CarParams:
    # the DUT's car params on another fingerprint
    return dut_params(self.car_model, fingerprint, self.dut_fw)

  def gap(self) -> str:
    # why the DUT can't drive the route's safety pair, empty if it can
    pair = safety_pair(self.CP)
    dut_pair = safety_pair(self.dut_CP)
    if self.CP.dashcamOnly:
      return "dashcam only in opendbc"
    if dut_pair == pair and not self.dut_CP.dashcamOnly:
      return ""
    # what the DUT's car params need besides its firmware: the alpha long toggle the route drove with
    with_long = interfaces[self.car_model].get_params(self.car_model, self.fingerprint, self.dut_fw, self.alpha_long, False, docs=False)
    if self.alpha_long and safety_pair(with_long) == pair and not with_long.dashcamOnly:
      what = "the route drove with openpilot longitudinal, the DUT has AlphaLongitudinalEnabled off"
    elif self.no_fw:
      what = f"depends on firmware, {self.no_fw}"
    else:
      what = "the DUT's car params differ with the route's firmware"
    if self.dut_CP.dashcamOnly:
      return f"dashcam only on the DUT, {what}"
    return f"param {what}: {dut_pair} != {pair}"


def car_params(car_model: str, path: Path, msgs: list) -> RouteCar:
  cls = type("R", (TestCarModelBase,), {"platform": car_model})
  car_fw, _, alpha_long = cls.get_testing_data_from_logreader(OpendbcLogReader(str(path), only_union_types=True, sort_by_time=True))
  route_CP = interfaces[cls.platform].get_params(cls.platform, cls.fingerprint, car_fw, alpha_long, False, docs=False)
  logged = logged_car_params(msgs)
  dut_fw, no_fw = cached_fw(logged)
  cache = logged.as_builder().to_bytes() if dut_fw else None
  return RouteCar(car_model, route_CP, dut_params(cls.platform, cls.fingerprint, dut_fw), cls.fingerprint, dut_fw, no_fw, cache, alpha_long)


def read_log(path: Path) -> list:
  # a log from the comma API is cached under a name cut from its signed url, so read it by content
  return sorted(LogReader.from_bytes(path.read_bytes()), key=lambda m: m.logMonoTime)


def load_scenario(s: Scenario) -> tuple[RouteCar, list]:
  # the route's car, the replay is built for its DUT car params, and the log
  path = get_cached_segment(s.route, s.segment)
  msgs = read_log(path)
  return car_params(s.car_model, path, msgs), msgs


def load_route(route: CarTestRoute) -> tuple[int, RouteCar, list]:
  segments = (2, 1, 0) if route.segment is None else (route.segment,)
  errors = []
  for segment in segments:
    try:
      path = get_cached_segment(route.route, segment)
      msgs = read_log(path)
    except (OSError, AssertionError) as e:
      errors.append(f"segment {segment}: {e!r}")
      continue
    try:
      return segment, car_params(str(route.car_model), path, msgs), msgs
    except Exception as e:
      errors.append(f"segment {segment}: can't derive safety config: {e!r}")
  raise OSError("; ".join(errors))


RouteData = tuple[int, tuple[int, int], str, float, list[tuple[float, bool]], list[tuple[float, bool]]]


def read_route(route: CarTestRoute) -> RouteData | Exception:
  # the picklable part of a route's load, run in a worker, a failure comes back as the exception
  try:
    segment, car, msgs = load_route(route)
    return segment, safety_pair(car.CP), car.gap(), can_span(msgs), engaged_samples(msgs), sendcan_samples(msgs)
  except Exception as e:
    return e


def build_candidates(routes) -> tuple[dict[tuple[int, int], list[Scenario]], list[str]]:
  # every route's scenario per pair, in routes order, so a pair can fall back to its next route
  scenarios: dict[tuple[int, int], list[Scenario]] = {}
  seen_pairs: set[tuple[int, int]] = set()
  engaged_from_start: set[tuple[int, int]] = set()
  no_lead_in: set[tuple[int, int]] = set()
  # pair -> why the DUT can't drive it
  dut_gaps: dict[tuple[int, int], str] = {}
  # pairs with an engaged window whose route logged no commands in it
  no_commands: set[tuple[int, int]] = set()
  gaps = []
  with ProcessPoolExecutor(os.cpu_count(), mp_context=multiprocessing.get_context("spawn")) as pool:
    loaded = list(pool.map(read_route, routes))
  for route, result in zip(routes, loaded, strict=True):
    who = f"{route.car_model} {route.route}"
    if isinstance(result, Exception):
      gaps.append(f"{who}: can't load ({result})")
      continue

    segment, mode_param, dut_gap, span, samples, sendcan = result
    if span < LENGTH:
      gaps.append(f"{who}: segment shorter than {LENGTH:g} s")
      continue
    try:
      checks = rx_checks(*mode_param)
    except ValueError as e:
      gaps.append(f"{who}: {e}")
      continue
    seen_pairs.add(mode_param)
    if dut_gap:
      dut_gaps[mode_param] = dut_gap
      continue

    if not samples:
      gaps.append(f"{who}: no panda state logged")
      continue
    picked = pick_window(samples, span)
    if picked is not None:
      window, rise = picked
      idle = max_idle(sendcan, samples, window, rise)
      if idle is None:
        no_commands.add(mode_param)
        continue
      scenario = Scenario(route.route, segment, str(route.car_model), mode_param[0], mode_param[1], window, rise, checks, idle)
      scenarios.setdefault(mode_param, []).append(scenario)
    elif pick_window(samples, span, lead_in=False) is not None:
      no_lead_in.add(mode_param)
    elif samples[0][1]:
      engaged_from_start.add(mode_param)

  for mode, param in sorted(seen_pairs - scenarios.keys()):
    if (mode, param) in dut_gaps:
      gaps.append(f"mode {mode} param {param}: {dut_gaps[(mode, param)]}")
    elif (mode, param) in no_commands:
      gaps.append(f"mode {mode} param {param}: no logged commands in the engaged window")
    elif (mode, param) in no_lead_in:
      gaps.append(f"mode {mode} param {param}: no {LEAD:g} s disengaged lead-in before the engage edge")
    elif (mode, param) in engaged_from_start:
      gaps.append(f"mode {mode} param {param}: no engage edge in segment")
    else:
      gaps.append(f"mode {mode} param {param}: no engaged stretch of {MIN_ENGAGED:g} s in any route")
  return scenarios, gaps


def dumps(scenarios: list[Scenario], gaps: list[str]) -> str:
  # one scenario or gap per line, so a change reads as a diff of the cars it touches
  rows = []
  for s in sorted(scenarios, key=lambda s: (s.mode, s.param)):
    row = asdict(s)
    row["rx_checks"] = sorted([bus, addr, counter] for (bus, addr), counter in s.rx_checks.items())
    rows.append(json.dumps(row, sort_keys=True))
  gap_lines = ",\n  ".join(json.dumps(gap) for gap in sorted(gaps))
  scenario_lines = ",\n  ".join(rows)
  return f'{{\n "gaps": [\n  {gap_lines}\n ],\n "scenarios": [\n  {scenario_lines}\n ]\n}}\n'


def load_scenarios() -> tuple[list[Scenario], list[str]]:
  data = json.loads(SCENARIOS.read_text())
  scenarios = []
  for row in data["scenarios"]:
    row["window"] = Window(**row["window"])
    row["rx_checks"] = {(bus, addr): counter for bus, addr, counter in row["rx_checks"]}
    scenarios.append(Scenario(**row))
  return scenarios, data["gaps"]


def main() -> None:
  # replay imports this module
  from openpilot.selfdrive.test.cars.replay import BUDGET, pick_scenarios
  SCENARIOS.write_text(dumps(*pick_scenarios(test_routes, BUDGET)))


if __name__ == "__main__":
  main()
