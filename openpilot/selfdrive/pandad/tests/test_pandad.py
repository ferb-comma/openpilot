#!/usr/bin/env python3

import os
import signal
import subprocess
import time
import unittest

from openpilot.common.test import OpenpilotTestCase
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.gpio import gpio_set, gpio_init
from panda import Panda, PandaDFU
from openpilot.system.manager.process_config import managed_processes
from openpilot.common.hardware import HARDWARE
from openpilot.common.hardware.comma.pins import GPIO

HERE = os.path.dirname(os.path.realpath(__file__))


class TestPandad(OpenpilotTestCase):
  COMMA_HARDWARE_TEST = True

  def setUp(self):
    super().setUp()
    managed_processes['pandad'].stop()
    if not Panda.list():
      self._run_test()

  def teardown_method(self):
    managed_processes['pandad'].stop()

  def _run_test(self, timeout=30) -> float:
    st = time.monotonic()
    sm = messaging.SubMaster(['pandaStates'])

    managed_processes['pandad'].start()
    while (time.monotonic() - st) < timeout:
      sm.update(10)
      if len(sm['pandaStates']) and sm['pandaStates'][0].pandaType != log.PandaState.PandaType.unknown:
        break
    dt = time.monotonic() - st
    managed_processes['pandad'].stop()

    if len(sm['pandaStates']) == 0 or sm['pandaStates'][0].pandaType == log.PandaState.PandaType.unknown:
      raise Exception("pandad failed to start")

    return dt

  def _go_to_dfu(self):
    HARDWARE.recover_internal_panda()
    assert Panda.wait_for_dfu(None, 10)

  def _flash_bootstub(self, fn):
    self._go_to_dfu()
    pd = PandaDFU(None)
    if fn is None:
      fn = os.path.join(HERE, pd.get_mcu_type().config.bootstub_fn)
    with open(fn, "rb") as f:
      pd.program_bootstub(f.read())
    HARDWARE.reset_internal_panda()

  def test_in_dfu(self):
    HARDWARE.recover_internal_panda()
    self._run_test()

  def test_in_bootstub(self):
    with Panda() as p:
      p.reset(enter_bootstub=True)
      assert p.bootstub
    self._run_test()

  def test_in_reset(self):
    gpio_init(GPIO.STM_RST_N, True)
    gpio_set(GPIO.STM_RST_N, True)
    assert not Panda.list()
    self._run_test()

  def test_release_to_devel_bootstub(self):
    st = time.monotonic()
    self._flash_bootstub(None)
    print("flash done", time.monotonic() - st)
    self._run_test()

  def test_recover_from_bad_bootstub(self):
    self._go_to_dfu()
    with PandaDFU(None) as pd:
      pd._handle.program(pd.get_mcu_type().config.bootstub_address, b"\x00"*100)
    HARDWARE.reset_internal_panda()
    assert not Panda.list()
    assert not PandaDFU.list()

    self._run_test()

  def test_daemon_crash(self):
    # a hard crash of the daemon (a native abort, or here a SIGKILL) gets the panda reset and the daemon restarted
    sock = messaging.sub_sock('pandaStates', timeout=100)
    managed_processes['pandad'].start()
    daemons = []
    for _ in range(3):
      # from the new daemon, not queued from the old one
      messaging.drain_sock(sock)
      self._wait_for_state(sock)
      daemons.append(self._daemon_pid())
      os.kill(daemons[-1], signal.SIGKILL)
      time.sleep(0.1)
    assert len(set(daemons)) == len(daemons)
    # the supervisor is still running
    proc = managed_processes['pandad'].proc
    assert proc is not None and proc.exitcode is None

  def _supervisor_pid(self) -> int:
    proc = managed_processes['pandad'].proc
    assert proc is not None and proc.pid is not None
    return proc.pid

  def _daemon_pid(self) -> int:
    return int(subprocess.check_output(["pgrep", "-P", str(self._supervisor_pid())]))

  def _wait_for_state(self, sock, timeout=30):
    # the latest pandaState from a connected panda, not the oldest one queued
    st = time.monotonic()
    while time.monotonic() - st < timeout:
      for msg in reversed(messaging.drain_sock(sock, wait_for_one=True)):
        if msg.pandaStates[0].pandaType != log.PandaState.PandaType.unknown:
          return msg.pandaStates[0]
    raise AssertionError("no pandaStates")

  def test_bootstub_while_running(self):
    # the panda drops into its bootstub mid-run. pandad can't read its state anymore, so it resets it and comes back
    sock = messaging.sub_sock('pandaStates', timeout=100)
    managed_processes['pandad'].start()
    self._wait_for_state(sock)
    time.sleep(2)
    uptime = self._wait_for_state(sock).uptime
    daemon = self._daemon_pid()

    with Panda(Panda.spi_list()[0], cli=False, disable_checks=False) as p:
      p.reset(enter_bootstub=True, reconnect=False)
    st = time.monotonic()
    while self._wait_for_state(sock, 15).uptime >= uptime:
      assert time.monotonic() - st < 15, "pandad didn't reset the panda"
    assert self._daemon_pid() != daemon


if __name__ == "__main__":
  unittest.main()
