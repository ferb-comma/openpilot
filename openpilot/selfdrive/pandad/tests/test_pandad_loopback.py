#!/usr/bin/env python3

import os
import copy
import random
import subprocess
import threading
import time
import unittest
from collections import defaultdict
from pathlib import Path
from pprint import pprint

from openpilot.common.test import OpenpilotTestCase
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from opendbc.car.structs import car
from panda import Panda
from opendbc.car.can_definitions import CanData
from openpilot.common.utils import retry
from openpilot.common.params import Params
from openpilot.common.realtime import Priority
from openpilot.common.timeout import Timeout
from openpilot.selfdrive.pandad import can_list_to_can_capnp
from openpilot.selfdrive.test.helpers import with_processes
from openpilot.system.manager.process_config import managed_processes


def publish_device_state(pm, started):
  msg = messaging.new_message('deviceState')
  msg.deviceState.started = started
  pm.send('deviceState', msg)

@retry(attempts=3)
def setup_pandad():
  params = Params()
  params.clear_all()

  pm = messaging.PubMaster(['deviceState'])
  sm = messaging.SubMaster(['pandaStates'])
  publish_device_state(pm, False)
  with Timeout(90, "pandad didn't start"):
    while sm.recv_frame['pandaStates'] < 1 or len(sm['pandaStates']) == 0 or \
        any(ps.pandaType == log.PandaState.PandaType.unknown for ps in sm['pandaStates']):
      publish_device_state(pm, False)
      sm.update(100)

  # pandad safety setting relies on these params
  cp = car.CarParams.new_message()

  safety_config = car.CarParams.SafetyConfig.new_message()
  safety_config.safetyModel = car.CarParams.SafetyModel.allOutput
  cp.safetyConfigs = [safety_config]

  params.put_bool("FirmwareQueryDone", True, block=True)
  params.put_bool("ControlsReady", True, block=True)
  params.put("CarParams", cp.to_bytes(), block=True)

  publish_device_state(pm, True)
  with Timeout(90, "pandad didn't set safety mode"):
    while any(ps.safetyModel != car.CarParams.SafetyModel.allOutput for ps in sm['pandaStates']):
      publish_device_state(pm, True)
      sm.update(100)

def send_random_can_messages(sendcan, count):
  sent_msgs = defaultdict(set)
  for _ in range(count):
    to_send = []
    for __ in range(random.randrange(20)):
      bus = random.choice(range(3))
      addr = random.randrange(1, 1<<29)
      dat = bytes(random.getrandbits(8) for _ in range(random.randrange(1, 9)))
      if (addr, dat) in sent_msgs[bus]:
        continue
      sent_msgs[bus].add((addr, dat))
      to_send.append(CanData(addr, dat, bus))
    sendcan.send(can_list_to_can_capnp(to_send, msgtype='sendcan'))
  return sent_msgs


class TestBoarddLoopback(OpenpilotTestCase):
  COMMA_HARDWARE_TEST = True
  @classmethod
  def setup_class(cls):
    os.environ['STARTED'] = '1'
    os.environ['BOARDD_LOOPBACK'] = '1'

  # the test stops pandad itself
  @with_processes(['pandad'], ignore_stopped=['pandad'])
  def test_loopback(self):
    setup_pandad()

    # the daemon is a child of the manager's process. its main loop is realtime, on the core that handles the SPI interrupt,
    # and the send thread is one priority below it
    proc = managed_processes['pandad'].proc
    assert proc is not None
    pid = int(subprocess.check_output(["pgrep", "-P", str(proc.pid)]))
    send_tid = next(int(t.name) for t in Path(f"/proc/{pid}/task").iterdir() if (t / "comm").read_text().strip() == "pandad_can_send")
    for tid, prio in ((pid, Priority.PANDAD), (send_tid, Priority.PANDAD_CAN_SEND)):
      assert os.sched_getscheduler(tid) == os.SCHED_FIFO
      assert os.sched_getparam(tid).sched_priority == prio
      assert os.sched_getaffinity(tid) == {3}

    sendcan = messaging.pub_sock('sendcan')
    can = messaging.sub_sock('can', conflate=False, timeout=100)
    sm = messaging.SubMaster(['pandaStates'])
    time.sleep(1)

    n = 200
    for i in range(n):
      print(f"pandad loopback {i}/{n}")

      sent_msgs = send_random_can_messages(sendcan, random.randrange(20, 100))

      sent_loopback = copy.deepcopy(sent_msgs)
      sent_loopback.update({k+128: copy.deepcopy(v) for k, v in sent_msgs.items()})
      sent_total = {k: len(v) for k, v in sent_loopback.items()}
      for _ in range(100 * 5):
        sm.update(0)
        recvd = messaging.drain_sock(can, wait_for_one=True)
        for msg in recvd:
          for m in msg.can:
            key = (m.address, m.dat)
            assert key in sent_loopback[m.src], f"got unexpected msg: {m.src=} {m.address=} {m.dat=}"
            sent_loopback[m.src].discard(key)

        if all(len(v) == 0 for v in sent_loopback.values()):
          break

      # if a set isn't empty, messages got dropped
      pprint(sent_msgs)
      pprint(sent_loopback)
      print({k: len(x) for k, x in sent_loopback.items()})
      print(sum([len(x) for x in sent_loopback.values()]))
      pprint(sm['pandaStates'])  # may drop messages due to RX buffer overflow
      for bus in sent_loopback.keys():
        assert not len(sent_loopback[bus]), f"loop {i}: bus {bus} missing {len(sent_loopback[bus])} out of {sent_total[bus]} messages"

    # SIGINT: a clean exit (not a SIGKILL after the timeout) that closes the relay, since we're onroad and not engaged.
    # the heartbeat keeps the panda from going SILENT on its own (1-2 s without one), it never changes the safety mode
    with Panda(Panda.spi_list()[0], cli=False, disable_checks=False) as p:
      stopped = threading.Event()
      def heartbeat():
        while not stopped.wait(0.1):
          p.send_heartbeat(False)
      t = threading.Thread(target=heartbeat)
      t.start()
      try:
        assert managed_processes['pandad'].stop() == 0
        assert p.health()['safety_mode'] == car.CarParams.SafetyModel.noOutput
      finally:
        stopped.set()
        t.join()


if __name__ == "__main__":
  unittest.main()
