import math
from collections import Counter
from collections.abc import Iterable

from opendbc.can.dbc import DBC, Msg, SignalType
from opendbc.can.packer import CANPacker
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces

from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.selfdrive.test.cars.rules import BUSES
from openpilot.selfdrive.test.cars.scenario import Scenario, Window, build_candidates, load_scenario

# the UP's can_replay frame period
DT = 0.01
# (addr, data, bus) of each message in a 10 ms frame
Frame = list[tuple[int, bytes, int]]
# frames/s of a station's lanes for buses 0-2, measured on AGNOS 20.1
BUDGET = {0: 230., 1: 130., 2: 60.}


def _bytes(msg) -> bytes:
  return (msg.as_builder() if hasattr(msg, 'as_builder') else msg).to_bytes()


def window_frames(msgs: Iterable, window: Window) -> list[Frame]:
  msgs = [m for m in msgs if m.which() == 'can']
  if not msgs:
    return []
  first = min(m.logMonoTime for m in msgs)
  frames: list[Frame] = [[] for _ in range(round((window.end - window.start) / DT))]
  parsed = can_capnp_to_list([_bytes(m) for m in msgs])
  for m, (_, can) in zip(msgs, parsed, strict=True):
    i = int(((m.logMonoTime - first) / 1e9 - window.start) // DT)
    if not 0 <= i < len(frames):
      continue
    for addr, dat, src in can:
      # the first panda's buses 0-2, not its tx echoes (src + 128) or another panda's buses (src + 4)
      if src < BUSES:
        frames[i].append((addr, dat, src))
  return frames


def _parsers(CP: structs.CarParams) -> list:
  interface = interfaces[CP.carFingerprint]
  car = interface(CP)
  radar = interface.RadarInterface(CP)
  return [car, radar]


def _packets(frames: list[Frame]) -> list[tuple[int, list[CanData]]]:
  return [(round(i * DT * 1e9), [CanData(a, d, b) for a, d, b in f]) for i, f in enumerate(frames)]


def needed_messages(CP: structs.CarParams, rx_checks: dict[tuple[int, int], bool], frames: list[Frame]) -> set[tuple[int, int]]:
  car, radar = _parsers(CP)
  for packet in _packets(frames):
    car.update([packet])
    radar.update([packet])
  # the parsers only add the messages they read once they run
  parsers = [p for p in car.can_parsers.values() if p is not None]
  if radar.rcp is not None:
    parsers.append(radar.rcp)
  needed = set(rx_checks)
  for p in parsers:
    for addr in p.message_states:
      needed.add((p.bus, addr))
  # keep each read bus alive, the parser times out a bus with no frame for 500 ms
  counts = Counter((bus, addr) for frame in frames for addr, _, bus in frame)
  for bus in {p.bus for p in parsers}:
    on_bus = {k: n for k, n in counts.items() if k[0] == bus}
    if on_bus:
      needed.add(max(on_bus, key=on_bus.get))
  return needed


def needed_frames(CP: structs.CarParams, rx_checks: dict[tuple[int, int], bool], frames: list[Frame]) -> list[Frame]:
  needed = needed_messages(CP, rx_checks, frames)
  return [[f for f in frame if (f[2], f[0]) in needed] for frame in frames]


def _dbcs(CP: structs.CarParams) -> dict[tuple[int, int], str]:
  # (bus, addr) -> name of the parser DBC that describes it
  car, radar = _parsers(CP)
  parsers = [p for p in car.can_parsers.values() if p is not None]
  if radar.rcp is not None:
    parsers.append(radar.rcp)
  dbcs: dict[tuple[int, int], str] = {}
  for p in parsers:
    for addr in p.dbc.addr_to_msg:
      dbcs.setdefault((p.bus, addr), p.dbc_name)
  return dbcs


def _is_counter(sig) -> bool:
  return sig.type == SignalType.COUNTER or sig.name == "COUNTER"


def _checked(msg: Msg) -> bool:
  return any(_is_counter(sig) or sig.calc_checksum is not None for sig in msg.sigs.values())


def _values(msg: Msg, dat: bytes) -> dict[str, float]:
  values = {}
  for sig in msg.sigs.values():
    if _is_counter(sig) or sig.calc_checksum is not None:
      continue
    raw = sig.get_raw_value(dat)
    if sig.is_signed:
      raw -= ((raw >> (sig.size - 1)) & 0x1) * (1 << sig.size)
    values[sig.name] = raw * sig.factor + sig.offset
  return values


def thin(CP: structs.CarParams, rx_checks: dict[tuple[int, int], bool], frames: list[Frame], budget: dict[int, float]) -> list[Frame]:
  seconds = len(frames) * DT
  load = Counter(bus for f in frames for _, _, bus in f)
  # every k-th frame of each message on a bus
  keep = {}
  for bus, rate in budget.items():
    if load[bus]:
      keep[bus] = max(1, math.ceil(load[bus] / seconds / rate))
  dbcs = _dbcs(CP)
  parsed = {name: DBC(name) for name in set(dbcs.values())}
  # one packer per bus and DBC, so each bus continues its own counters
  packers: dict[tuple[int, str], CANPacker] = {}
  seen: Counter[tuple[int, int]] = Counter()
  out: list[Frame] = []
  for frame in frames:
    kept: Frame = []
    for addr, dat, bus in frame:
      n = seen[(bus, addr)]
      seen[(bus, addr)] += 1
      k = keep.get(bus, 1)
      dbc = dbcs.get((bus, addr))
      msg = parsed[dbc].addr_to_msg[addr] if dbc is not None else None
      # the panda checks this message and no DBC can repack it or its counter, so it keeps every frame
      if (bus, addr) in rx_checks:
        if msg is None:
          k = 1
        elif rx_checks[(bus, addr)] and not any(_is_counter(sig) for sig in msg.sigs.values()):
          k = 1
      if n % k:
        continue
      # a kept frame skips k - 1 counter steps, repack it so the counter and checksum stay consecutive
      if k > 1 and msg is not None and dbc is not None and _checked(msg):
        packer = packers.setdefault((bus, dbc), CANPacker(dbc))
        addr, dat, bus = packer.make_can_msg(addr, bus, _values(msg, dat))
      kept.append((addr, dat, bus))
    out.append(kept)
  return out


def over_budget(frames: list[Frame], budget: dict[int, float]) -> list[str]:
  seconds = len(frames) * DT
  load = Counter(bus for f in frames for _, _, bus in f)
  over = []
  for bus, rate in sorted(budget.items()):
    if load[bus] / seconds > rate:
      over.append(f"bus {bus} {load[bus] / seconds:.0f} frames/s > {rate:.0f}")
  return over


def build_replay(s: Scenario, CP: structs.CarParams, msgs: list, budget: dict[int, float]) -> list[Frame]:
  # what a station sends, one frame per DT
  needed = needed_frames(CP, s.rx_checks, window_frames(msgs, s.window))
  return thin(CP, s.rx_checks, needed, budget)


def pick_replay(candidates: list[Scenario], budget: dict[int, float]) -> tuple[Scenario, list[str]]:
  # a pair's scenarios in routes order, the first whose replay fits the station's lanes, or else the last
  for s in candidates:
    CP, msgs = load_scenario(s)
    lane_problems = over_budget(build_replay(s, CP, msgs, budget), budget)
    if not lane_problems:
      break
  return s, lane_problems


def pick_scenarios(routes, budget: dict[int, float]) -> tuple[list[Scenario], list[str]]:
  # what a rack run drives: each pair's scenario after the lane budget fallback, and the gaps
  candidates, gaps = build_candidates(routes)
  scenarios = []
  for pair_candidates in candidates.values():
    try:
      s, lane_problems = pick_replay(pair_candidates, budget)
    except Exception as e:
      first = pair_candidates[0]
      gaps.append(f"{first.car_model} {first.route}: {e!r}")
      continue
    if lane_problems:
      gaps.append(f"mode {s.mode} param {s.param}: lane budget: {', '.join(lane_problems)}")
    else:
      scenarios.append(s)
  return scenarios, gaps
