import logging
import math
from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass

from opendbc.can.dbc import DBC, Msg, SignalType
from opendbc.can.packer import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import FRAME_FINGERPRINT, interfaces

from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.selfdrive.test.cars.rules import BUSES
from openpilot.selfdrive.test.cars.scenario import Fingerprint, RouteCar, Scenario, Window, build_candidates, load_scenario, safety_pair

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
  # the test's own parsers see a bus time out whenever the window has a gap, openpilot's logs stay
  carlog = logging.getLogger("carlog")
  level, carlog.level = carlog.level, logging.CRITICAL
  try:
    for packet in _packets(frames):
      car.update([packet])
      radar.update([packet])
  finally:
    carlog.setLevel(level)
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
      needed.add(max(on_bus, key=lambda k: on_bus[k]))
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


def _radar_trigger(CP: structs.CarParams) -> tuple[int, int] | None:
  # (bus, addr) of the message the radar interface publishes its tracks on, radard needs it at the car's rate
  _, radar = _parsers(CP)
  trigger = getattr(radar, 'trigger_msg', None)
  if radar.rcp is None or trigger is None:
    return None
  return radar.rcp.bus, trigger


def _full_rate(dbcs: dict[tuple[int, int], str], parsed: dict[str, DBC], rx_checks: dict[tuple[int, int], bool],
               radar_trigger: tuple[int, int] | None, key: tuple[int, int]) -> bool:
  # the messages that keep every frame
  if key == radar_trigger:
    return True
  if key not in rx_checks:
    return False
  # the panda checks this message and no DBC can repack it or its counter
  dbc = dbcs.get(key)
  if dbc is None:
    return True
  msg = parsed[dbc].addr_to_msg[key[1]]
  return rx_checks[key] and not any(_is_counter(sig) for sig in msg.sigs.values())


def thin(CP: structs.CarParams, rx_checks: dict[tuple[int, int], bool], frames: list[Frame], budget: dict[int, float],
         keep_all: set[tuple[int, int]] | None = None) -> list[Frame]:
  # keep_all: messages that keep every frame they have
  seconds = len(frames) * DT
  dbcs = _dbcs(CP)
  parsed = {name: DBC(name) for name in set(dbcs.values())}
  radar_trigger = _radar_trigger(CP)
  counts = Counter((bus, addr) for f in frames for addr, _, bus in f)
  full = {key for key in counts if key in (keep_all or set()) or _full_rate(dbcs, parsed, rx_checks, radar_trigger, key)}
  # every k-th frame of each other message on a bus, in what the full rate messages leave of the budget
  keep = {}
  for bus, rate in budget.items():
    fixed = sum(n for key, n in counts.items() if key[0] == bus and key in full)
    rest = [n for key, n in counts.items() if key[0] == bus and key not in full]
    if not rest:
      continue
    # each message keeps its first frame and every k-th after it
    k = 1
    while k < len(frames) and fixed + sum(math.ceil(n / k) for n in rest) > rate * seconds:
      k += 1
    keep[bus] = k
  # one packer per bus and DBC, so each bus continues its own counters
  packers: dict[tuple[int, str], CANPacker] = {}
  seen: Counter[tuple[int, int]] = Counter()
  out: list[Frame] = []
  for frame in frames:
    kept: Frame = []
    for addr, dat, bus in frame:
      n = seen[(bus, addr)]
      seen[(bus, addr)] += 1
      k = 1 if (bus, addr) in full else keep.get(bus, 1)
      dbc = dbcs.get((bus, addr))
      msg = parsed[dbc].addr_to_msg[addr] if dbc is not None else None
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


def _radar_output(CP: structs.CarParams, frames: list[Frame]) -> tuple[int, set[str]]:
  # tracks published and the errors raised by the radar interface on the second of two loops, as the UP plays it
  _, radar = _parsers(CP)
  packets = _packets(frames)
  loop = round(len(frames) * DT * 1e9)
  for t, packet in packets:
    radar.update([(t, packet)])
  published = 0
  errors: set[str] = set()
  for t, packet in packets:
    out = radar.update([(t + loop, packet)])
    if out is None:
      continue
    published += 1
    errors |= {name for name, raised in out.errors.to_dict().items() if raised}
  return published, errors


def radar_problems(CP: structs.CarParams, window: list[Frame], replay: list[Frame]) -> list[str]:
  # the radar interface has to publish as often and with no more errors on the replay as on the route's window
  if _radar_trigger(CP) is None:
    return []
  route_published, route_errors = _radar_output(CP, window)
  published, errors = _radar_output(CP, replay)
  seconds = len(window) * DT
  problems = []
  if published < route_published:
    problems.append(f"radar tracks {published / seconds:.1f} Hz < {route_published / seconds:.1f} Hz on the route")
  if errors - route_errors:
    problems.append(f"radar errors {', '.join(sorted(errors - route_errors))}")
  return problems


def _differs(car: RouteCar, fingerprint: Fingerprint) -> list[str]:
  # the car params fields the DUT derives differently from this fingerprint
  dut = car.dut(fingerprint).to_dict()
  return sorted(k for k, v in car.dut_CP.to_dict().items() if dut.get(k) != v)


def _can_valid(CP: structs.CarParams, frames: list[Frame]) -> bool:
  # carState is canValid throughout the second of two loops, as the UP plays it
  car = interfaces[CP.carFingerprint](CP)
  packets = _packets(frames)
  loop = round(len(frames) * DT * 1e9)
  for t, packet in packets:
    car.update([(t, packet)])
  return all(car.update([(t + loop, packet)]).canValid for t, packet in packets)


def _dut_problem(car: RouteCar, fingerprint: Fingerprint, replay: list[Frame]) -> str | None:
  # what keeps the DUT, deriving its car params from this fingerprint, from driving the replay built for its route fingerprint
  dut = car.dut(fingerprint)
  if safety_pair(dut) != safety_pair(car.dut_CP):
    return f"safety pair {safety_pair(dut)} != {safety_pair(car.dut_CP)}"
  if dut.dashcamOnly:
    return "dashcam only"
  if not _can_valid(dut, replay):
    return "carState not canValid"
  return None


def presented(replay: list[Frame]) -> Fingerprint:
  # the fingerprint card collects wherever in the loop it starts: the messages with a frame in every FRAME_FINGERPRINT frames
  at: dict[tuple[int, int], list[int]] = {}
  sizes = {}
  for i, frame in enumerate(replay):
    for addr, dat, bus in frame:
      at.setdefault((bus, addr), []).append(i)
      sizes[(bus, addr)] = len(dat)
  fingerprint = gen_empty_fingerprint()
  for (bus, addr), frames in at.items():
    gaps = [b - a for a, b in zip(frames, frames[1:], strict=False)]
    gaps.append(frames[0] + len(replay) - frames[-1])
    if max(gaps) <= FRAME_FINGERPRINT:
      fingerprint[bus][addr] = sizes[(bus, addr)]
  return fingerprint


def fingerprint_additions(car: RouteCar, replay: list[Frame]) -> list[tuple[int, int]]:
  # the fewest of the route's fingerprint messages the replay drops that the DUT needs to derive its car params, none if they can't
  fingerprint = car.fingerprint
  shown = presented(replay)
  if _dut_problem(car, shown, replay) is None:
    return []
  in_replay = {(bus, addr) for frame in replay for addr, _, bus in frame}
  dropped = sorted((bus, addr) for bus, addrs in fingerprint.items() for addr in addrs if bus < BUSES and (bus, addr) not in in_replay)
  # add them until the car params match, then take back the ones not needed
  restored = {bus: dict(addrs) for bus, addrs in shown.items()}
  added = []
  for bus, addr in dropped:
    restored[bus][addr] = fingerprint[bus][addr]
    added.append((bus, addr))
    if not _differs(car, restored):
      break
  else:
    return []
  for bus, addr in list(added):
    size = restored[bus].pop(addr)
    if _differs(car, restored):
      restored[bus][addr] = size
    else:
      added.remove((bus, addr))
  return added


def fingerprint_frames(frames: list[Frame], keys: list[tuple[int, int]]) -> list[Frame]:
  # the frames of these messages card needs to fingerprint them, the first, the last,
  # and in between one at most FRAME_FINGERPRINT frames after the previous kept wherever the route allows
  out: list[Frame] = [[] for _ in frames]
  for key in keys:
    at = [(i, (addr, dat, bus)) for i, frame in enumerate(frames) for addr, dat, bus in frame if (bus, addr) == key]
    last = None
    for j, (i, msg) in enumerate(at):
      last_one = j == len(at) - 1
      if last is None or last_one or at[j + 1][0] - last > FRAME_FINGERPRINT:
        out[i].append(msg)
        last = i
  return out


def fingerprint_problems(car: RouteCar, replay: list[Frame]) -> list[str]:
  # card derives its car params from the fingerprint the replay presents, not the route's
  shown = presented(replay)
  problem = _dut_problem(car, shown, replay)
  if problem is None:
    return []
  return [f"{problem} on the replay's fingerprint, car params differ: {', '.join(_differs(car, shown))}"]


@dataclass
class Replay:
  # what a station sends, one frame per DT, and the route's fingerprint messages added for the DUT's car params
  frames: list[Frame]
  added: list[tuple[int, int]]
  # the route's car params for the DUT's CarParamsCache, so card takes their firmware, None to skip the firmware query
  cache: bytes | None = None
  fw_count: int = 0
  # why the DUT runs without the route's firmware
  no_fw: str = ""

  def firmware(self) -> str:
    if self.cache is None:
      return f"firmware: none, SKIP_FW_QUERY, {self.no_fw}"
    return f"firmware: the route's {self.fw_count} carFw entries from CarParamsCache"

  def describe(self, budget: dict[int, float]) -> str:
    seconds = len(self.frames) * DT
    load = Counter(bus for f in self.frames for _, _, bus in f)
    messages = {(bus, addr) for f in self.frames for addr, _, bus in f}
    rates = ", ".join(f"bus {bus} {load[bus] / seconds:.0f}/{rate:.0f}" for bus, rate in sorted(budget.items()))
    added = ", ".join(f"{addr:#x} bus {bus}" for bus, addr in self.added) or "none"
    return f"replay: {len(messages)} messages, frames/s of budget {rates}, fingerprint additions: {added}"


def build_replay(s: Scenario, car: RouteCar, msgs: list, budget: dict[int, float]) -> Replay:
  CP = car.dut_CP
  window = window_frames(msgs, s.window)
  needed = needed_frames(CP, s.rx_checks, window)
  replay = thin(CP, s.rx_checks, needed, budget)
  added = fingerprint_additions(car, replay)
  if added:
    # the fingerprint messages count against the budget like the rest
    sparse = fingerprint_frames(window, added)
    merged = [a + b for a, b in zip(needed, sparse, strict=True)]
    replay = thin(CP, s.rx_checks, merged, budget, set(added))
  return Replay(replay, added, car.cache, len(car.dut_fw), car.no_fw)


def scenario_replay(s: Scenario) -> Replay:
  # a scenario's replay from its route, built in a worker while the previous car drives
  try:
    car, msgs = load_scenario(s)
    return build_replay(s, car, msgs, BUDGET)
  except Exception as e:
    # the worker pickles exceptions back, and not every exception pickles
    raise RuntimeError(repr(e)) from None


def replay_problems(s: Scenario, car: RouteCar, msgs: list, budget: dict[int, float]) -> list[str]:
  # what keeps a scenario's replay from playing its car on a station
  replay = build_replay(s, car, msgs, budget).frames
  problems = fingerprint_problems(car, replay)
  problems += [f"lane budget: {p}" for p in over_budget(replay, budget)]
  problems += radar_problems(car.dut_CP, window_frames(msgs, s.window), replay)
  return problems


def pick_replay(candidates: list[Scenario], budget: dict[int, float]) -> tuple[Scenario, list[str]]:
  # a pair's scenarios in routes order, the first whose replay plays on a station, or else the last
  for s in candidates:
    car, msgs = load_scenario(s)
    problems = replay_problems(s, car, msgs, budget)
    if not problems:
      break
  return s, problems


def pick_scenarios(routes, budget: dict[int, float]) -> tuple[list[Scenario], list[str]]:
  # what a rack run drives: each pair's scenario after the lane budget fallback, and the gaps
  candidates, gaps = build_candidates(routes)
  scenarios = []
  for pair_candidates in candidates.values():
    try:
      s, problems = pick_replay(pair_candidates, budget)
    except Exception as e:
      first = pair_candidates[0]
      gaps.append(f"{first.car_model} {first.route}: {e!r}")
      continue
    if problems:
      gaps.append(f"mode {s.mode} param {s.param}: {', '.join(problems)}")
    else:
      scenarios.append(s)
  return scenarios, gaps
