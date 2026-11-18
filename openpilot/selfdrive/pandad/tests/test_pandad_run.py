import logging
import multiprocessing
import os
import signal
import sys
import threading
import time
from unittest import mock

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import CarParams
from openpilot.common.parameterized import parameterized
from openpilot.common.realtime import Priority
from openpilot.common.swaglog import ForwardingHandler
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.pandad import pandad
from openpilot.selfdrive.pandad.pandad import can_send_thread, pandad_run
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.selfdrive.pandad.tests.helpers import FakePanda
from panda import Panda
from panda.python.spi import PandaSpiHandle, PandaSpiMissingAck, PandaSpiNackResponse

SafetyModel = CarParams.SafetyModel
real_sleep = time.sleep  # tests mock it in pandad
ELM327 = ("set_safety_mode", SafetyModel.elm327, 1)
NO_OUTPUT = ("set_safety_mode", SafetyModel.noOutput, 0)


def sendcan(frames, log_mono_time: int) -> bytes:
  msg = messaging.log_from_bytes(can_list_to_can_capnp(frames, 'sendcan')).as_builder()
  msg.logMonoTime = log_mono_time
  return msg.to_bytes()


def wait_for_exit(timeout: float) -> bool:
  deadline = time.monotonic() + timeout
  while not pandad.do_exit and time.monotonic() < deadline:
    time.sleep(0.01)
  return pandad.do_exit


def run_in_thread(target, *args):
  # can_send_thread names its thread, so not the test's. returns what it raised
  errors = []
  def run():
    try:
      target(*args)
    except Exception as e:
      errors.append(e)
  # a daemon thread, so a test that fails doesn't hang the run
  t = threading.Thread(target=run, daemon=True)
  t.start()
  return t, errors


class FakeSock:
  """sendcan, all queued up until a pause (seconds). stops the send thread once it's all been read"""
  def __init__(self, msgs: list, stop: threading.Event):
    self.msgs, self.stop = msgs, stop

  def receive(self, non_blocking=False):
    if non_blocking and (not self.msgs or isinstance(self.msgs[0], float)):
      return None
    if not self.msgs:
      self.stop.set()
      return None
    msg = self.msgs.pop(0)
    if isinstance(msg, float):
      real_sleep(msg)
      return self.receive()
    return msg


class TestCanSendThread(OpenpilotTestCase):
  def run_thread(self, msgs, fake_send=False, panda=None):
    panda, stop = panda or FakePanda(), threading.Event()
    with mock.patch.object(pandad.messaging, 'sub_sock', return_value=FakeSock(msgs, stop)) as sub_sock:
      t, errors = run_in_thread(can_send_thread, panda, stop, fake_send)
      t.join()
    assert errors == []
    # it sees the stop within 100 ms
    sub_sock.assert_called_once_with('sendcan', timeout=100)
    return [c for c in panda.calls if c[0] == "can_send_many"]

  @parameterized.expand([(False,), (True,)])
  def test_staleness(self, fake_send):
    now = time.monotonic_ns()
    with mock.patch.object(pandad, 'cloudlog') as log:
      sent = self.run_thread([
        sendcan([(1, b'1', 0), (5, b'5', 4)], now),  # bus 4 is another panda's
        sendcan([(2, b'2', 0)], now - 1_100_000_000),  # stale
        sendcan([(3, b'3', 0)], now + 30_000_000_000),  # from the future
        sendcan([(4, b'4', 1)], now - 500_000_000),
      ], fake_send)
    # what's queued goes out together. FAKESEND sends nothing
    assert sent == ([] if fake_send else [("can_send_many", [(1, b'1', 0), (4, b'4', 1)], pandad.CAN_SEND_TIMEOUT_MS)])
    # at most a line a second
    log.error.assert_called_once_with("sendcan: dropped 1 messages, too old to send")

  def test_invalid_message(self):
    # an invalid message is dropped on its own, not with the ones queued with it
    now = time.monotonic_ns()
    with mock.patch.object(pandad, 'cloudlog') as log:
      sent = self.run_thread([
        sendcan([(1, b'1', 0)], now),
        sendcan([(2, b'123456789', 0)], now),  # no DLC for 9 bytes
        b'garbage',
        messaging.new_message('can', 1).to_bytes(),  # not sendcan
        sendcan([(3, b'3', 1), (6, b'123456789', 4)], now),  # another panda's frames aren't checked
      ])
    assert [frames for _, frames, _ in sent] == [[(1, b'1', 0), (3, b'3', 1)]]
    assert log.error.call_count == 1 and log.error.call_args.args[0].startswith("sendcan: dropped 1 messages, invalid")
    assert not log.exception.called

  def test_batches(self):
    # a burst goes out in batches of about SEND_BATCH_FRAMES, a message is never split. messages with nothing to send
    # count too (up to SEND_BATCH_MSGS). sleeping after each lets the main loop have the SPI bus and the GIL
    now = time.monotonic_ns()
    msgs = [sendcan([(i, b'1', 0)] * 100, now) for i in range(5)] + [0.0]
    msgs += [sendcan([], now) for _ in range(pandad.SEND_BATCH_MSGS + 10)] + [sendcan([(1, b'1', 0)], now)]
    with mock.patch.object(pandad.time, 'sleep', wraps=time.sleep) as sleep:
      sent = self.run_thread(msgs)
    assert [len(frames) for _, frames, _ in sent] == [300, 200, 1]
    assert [c.args for c in sleep.call_args_list] == [(0.001,)] * 4

  def test_nack(self):
    # a full TX buffer drops the batch and the thread carries on. at most a line a second
    panda = FakePanda()
    now = time.monotonic_ns()
    # read after the pause, so not stale
    later = now + 900_000_000
    msgs = [sendcan([(1, b'1', 0)], now), 0.0, sendcan([(2, b'1', 0)], now), sendcan([(5, b'1', 0)], now - 1_100_000_000), 1.05,
            sendcan([(3, b'1', 0)], later), 0.0, sendcan([(4, b'1', 0)], later)]
    with mock.patch.object(pandad, 'cloudlog') as log, \
         mock.patch.object(panda, 'can_send_many', side_effect=[PandaSpiNackResponse] * 3 + [None]) as send:
      self.run_thread(msgs, panda=panda)
    assert send.call_count == 4
    # each reason counted on its own
    assert [c.args[0] for c in log.error.call_args_list] == ["sendcan: dropped 1 messages, panda isn't taking CAN frames",
      "sendcan: dropped 1 messages, too old to send; dropped 2 messages, panda isn't taking CAN frames"]

  @parameterized.expand([("panda_gone",), ("setup",)])
  def test_thread_ends(self, failure):
    # logged, and the thread ends. the main loop sees that and exits, so main() resets the panda
    panda = FakePanda()
    with mock.patch.object(pandad, 'cloudlog') as log, mock.patch.object(panda, 'can_send_many', side_effect=PandaSpiMissingAck) as send, \
         mock.patch.object(pandad.messaging, 'sub_sock', side_effect=RuntimeError if failure == "setup" else None) as sub_sock:
      sub_sock.return_value = FakeSock([sendcan([(i, b'1', 0)], time.monotonic_ns()) for i in (1, 2)], threading.Event())
      t, errors = run_in_thread(can_send_thread, panda, threading.Event(), False)
      t.join(timeout=5)
    assert not t.is_alive() and errors == []
    assert send.call_count == (failure == "panda_gone")
    log.exception.assert_called_once_with("pandad.can_send_thread_failed")
    log.bind.assert_called_once_with(daemon="pandad")


class TestPandadRun(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    mock.patch.object(pandad, 'do_exit', False).start()
    mock.patch.object(pandad.HARDWARE, 'set_ir_power').start()
    self.addCleanup(mock.patch.stopall)
    self.pm = messaging.PubMaster(['deviceState', 'selfdriveState', 'sendcan'])

  def publish(self, started: bool, selfdrive: tuple[bool, bool] | None, sendcan: bool = False):
    msg = messaging.new_message('deviceState')
    msg.deviceState.started = started
    msg.deviceState.fanSpeedPercentDesired = 50
    self.pm.send('deviceState', msg)
    if selfdrive is not None:
      msg = messaging.new_message('selfdriveState', valid=selfdrive[0])
      msg.selfdriveState.enabled = selfdrive[1]
      self.pm.send('selfdriveState', msg)
    if sendcan:
      self.pm.send('sendcan', can_list_to_can_capnp([(1, b'1', 0)], 'sendcan'))

  def run_until(self, panda, done, started=True, selfdrive=None, sendcan=False, timeout=5.):
    """runs pandad_run until done(), or until it raises. returns what it raised. selfdrive can be a function, called every tick"""
    t, errors = run_in_thread(pandad_run, panda)
    self.addCleanup(t.join)
    self.addCleanup(setattr, pandad, 'do_exit', True)
    deadline = time.monotonic() + timeout
    while t.is_alive() and not done():
      assert time.monotonic() < deadline, "timed out"
      self.publish(started, selfdrive() if callable(selfdrive) else selfdrive, sendcan)
      time.sleep(0.01)
    pandad.do_exit = True
    t.join(timeout=2)
    assert not t.is_alive()
    # the send thread is stopped either way
    assert not any(th.name == "pandad_can_send" for th in threading.enumerate())
    return errors

  @parameterized.expand([
    ("offroad", False, None, False),
    ("onroad_no_selfdrive", True, None, False),
    ("onroad_invalid", True, (False, True), False),
    ("onroad_disabled", True, (True, False), False),
    ("onroad_engaged", True, (True, True), True),
  ])
  def test_loop(self, name, started, selfdrive, engaged):
    panda = FakePanda(ignition_line=True, power_save_enabled=False)
    peripherals = mock.patch.object(pandad.PeripheralControl, 'update').start()
    def heartbeats():
      return [c[1] for c in panda.calls if c[0] == "send_heartbeat"]
    assert self.run_until(panda, lambda: len(heartbeats()) >= 6 and heartbeats()[-1] == engaged, started, selfdrive) == []

    # 100 Hz can, 20 Hz fan and IR, 10 Hz state + heartbeat and logs, 2 Hz peripheralState
    ticks = panda.count("can_recv")
    assert peripherals.call_count == (ticks + 4) // 5
    assert peripherals.call_args.args[1] == started
    assert panda.count("health") == len(heartbeats()) == (ticks + 9) // 10
    assert panda.count("debug_read") == (ticks + 4) // 10
    assert panda.count("get_fan_rpm") == (ticks + 49) // 50
    # engaged once selfdriveState comes in, which takes a few ticks
    hb = heartbeats()
    assert hb[hb.index(engaged):] == [engaged] * (len(hb) - hb.index(engaged))

    # ELM327 to fingerprint onroad. the relay closes on exit when onroad and not engaged
    safety_writes = [c for c in panda.writes() if c[0] == "set_safety_mode"]
    assert safety_writes == [ELM327] * started + [NO_OUTPUT] * (started and not engaged)

  def test_selfdrive_stops(self):
    # selfdrived hangs while engaged: the heartbeat goes back to not engaged once selfdriveState isn't alive
    panda = FakePanda(ignition_line=True, power_save_enabled=False)
    def heartbeats():
      return [c[1] for c in panda.calls if c[0] == "send_heartbeat"]
    def selfdrive():
      return None if True in heartbeats() else (True, True)
    assert self.run_until(panda, lambda: True in heartbeats() and heartbeats()[-1] is False, selfdrive=selfdrive) == []

  @parameterized.expand([(False,), (True,)])
  def test_env(self, fake):
    # FAKESEND: no CAN sent. NO_FAN_CONTROL: no fan writes. STARTED: ignition on, so power save goes off
    panda = FakePanda()
    with mock.patch.dict(os.environ, {"FAKESEND": "1", "NO_FAN_CONTROL": "1", "STARTED": "1"} if fake else {}):
      self.run_until(panda, lambda: panda.count("can_recv") >= 50, sendcan=True)
    assert (panda.count("can_send_many") > 0) != fake
    assert (panda.count("set_fan_power") > 0) != fake
    assert (("set_power_save", False) in panda.calls) == fake

  @parameterized.expand([
    ("recv", 'can_recv', PandaSpiMissingAck),
    ("health", 'health', RuntimeError),
    ("fan", 'set_fan_power', PandaSpiMissingAck),
    ("send", 'can_send_many', PandaSpiMissingAck),
  ])
  def test_errors(self, name, method, error):
    # it raises, e.g. the panda went silent or into its bootstub, so the daemon exits and main() resets the panda
    panda = FakePanda()
    mock.patch.object(panda, method, side_effect=error).start()
    mock.patch.object(pandad, 'cloudlog').start()
    errors = self.run_until(panda, lambda: False, sendcan=True)
    assert len(errors) == 1
    if method == 'health':
      # no heartbeat when the panda can't be read
      assert panda.count("send_heartbeat") == 0
    if method == 'can_send_many':
      assert str(errors[0]) == "can send thread died"


class TestRun(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    # run() sets the default for its process
    self.addCleanup(setattr, PandaSpiHandle, 'timeout', PandaSpiHandle.timeout)
    self.addCleanup(sys.setswitchinterval, sys.getswitchinterval())

  def test_connect(self):
    with mock.patch.object(pandad, 'Panda') as P, mock.patch.dict(os.environ):
      for k in ("BOARDD_LOOPBACK", "BOARDD_SKIP_FW_CHECK"):
        os.environ.pop(k, None)
      p = P.return_value

      assert pandad.connect('spi') is p
      P.assert_called_once_with('spi', cli=False, disable_checks=False)
      assert p.set_canfd_auto.call_args_list == [mock.call(i, True) for i in range(3)]
      p.set_can_loopback.assert_not_called()

      os.environ["BOARDD_LOOPBACK"] = "1"
      pandad.connect('spi')
      p.set_can_loopback.assert_called_once_with(True)

      # out of date firmware
      p.up_to_date.return_value = False
      with self.assertRaises(RuntimeError):
        pandad.connect('spi')
      os.environ["BOARDD_SKIP_FW_CHECK"] = "1"
      assert pandad.connect('spi') is p

  def test_run(self):
    # the internal (SPI) panda only. every request from connecting on gets the short timeout
    state = []
    def connect(serial):
      state.append(PandaSpiHandle.timeout)
      return FakePanda()
    with mock.patch.object(Panda, 'usb_list', return_value=['usb']), mock.patch.object(pandad, 'connect', side_effect=connect) as conn, \
         mock.patch.object(pandad, 'config_realtime_process') as rt, \
         mock.patch.object(pandad, 'pandad_run', side_effect=lambda p: state.append(sys.getswitchinterval())):
      with mock.patch.object(Panda, 'spi_list', return_value=[]):
        pandad.run()
      conn.assert_not_called()

      with mock.patch.object(Panda, 'spi_list', return_value=['spi']):
        pandad.run()
      conn.assert_called_once_with('spi')
    assert state == [pandad.PANDA_TIMEOUT_MS, 0.0005]
    # FIFO on core 3, above the send thread, so heavy sendcan can't starve the 100 Hz loop
    assert rt.call_args.args[0] == 3 and rt.call_args.args[1] > Priority.PANDAD_CAN_SEND


class TestMain(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    for sig in pandad.STOP_SIGNALS:
      self.addCleanup(signal.signal, sig, signal.getsignal(sig))
    # main() forwards the panda library's logs to cloudlog, a mock here
    panda_logger = logging.getLogger('panda')
    self.addCleanup(setattr, panda_logger, 'handlers', panda_logger.handlers)
    for p in (mock.patch.object(pandad, 'do_exit', False), mock.patch.object(pandad.Panda, 'list', return_value=[])):
      p.start()
      self.addCleanup(p.stop)

  def test_restart(self):
    # main() resets the panda and runs the daemon in a new process each time it's gone: crashed, or lost the panda. until a signal
    calls = []
    def run():
      # forked, so it has the calls up to here
      starts = calls.count("ready")
      if starts == 1:
        os.kill(os.getpid(), signal.SIGKILL)  # a hard crash, like a native abort
      elif starts == 2:
        raise PandaSpiMissingAck  # the panda's gone
      else:
        os.kill(os.getppid(), signal.SIGTERM)  # main() passes it on
        assert wait_for_exit(5)

    with mock.patch.object(pandad.HARDWARE, 'reset_internal_panda', lambda: calls.append("reset")), \
         mock.patch.object(pandad, 'panda_ready', lambda: calls.append("ready") or True), \
         mock.patch.object(pandad, 'run', run), mock.patch.object(pandad, 'cloudlog') as log:
      pandad.main()

    assert calls == ["reset", "ready"] * 3
    assert [c.args for c in log.error.call_args_list] == [(f"pandad exited with {-signal.SIGKILL}",), ("pandad exited with 1",)]
    assert [type(h) for h in logging.getLogger('panda').handlers] == [ForwardingHandler]

  def test_signal_at_fork(self):
    # a signal right after the fork still gets to the daemon, though the first SIGINT main() passes on is lost,
    # as one is when it lands before the new process's after-fork setup (CPython clears tripped signals there)
    kill, start = os.kill, multiprocessing.context.ForkProcess.start
    forwarded = []
    def lossy_kill(pid, sig):
      forwarded.append(sig)
      if len(forwarded) > 1:
        kill(pid, sig)

    def start_then_signal(proc):
      start(proc)
      kill(os.getpid(), signal.SIGTERM)

    def run():
      assert wait_for_exit(5)

    with mock.patch.object(multiprocessing.context.ForkProcess, 'start', start_then_signal), mock.patch.object(os, 'kill', lossy_kill), \
         mock.patch.object(pandad.HARDWARE, 'reset_internal_panda'), mock.patch.object(pandad, 'panda_ready', return_value=True), \
         mock.patch.object(pandad, 'run', run), mock.patch.object(pandad, 'cloudlog') as log:
      pandad.main()
    assert len(forwarded) > 1
    assert not log.error.called
    assert not log.exception.called
