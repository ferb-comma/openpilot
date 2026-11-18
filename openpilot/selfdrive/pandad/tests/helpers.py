# fakes shared by the pandad unit tests
import openpilot.cereal.messaging as messaging
from opendbc.car.structs import CarParams
from openpilot.cereal import log
from panda import Panda

SafetyModel = CarParams.SafetyModel
TRES = int(log.PandaState.PandaType.tres)


class FakePubMaster(messaging.PubMaster):
  def __init__(self):
    self.sent: list[tuple[str, bytes]] = []

  def send(self, s, dat):
    self.sent.append((s, dat if isinstance(dat, bytes) else dat.to_bytes()))


# every value differs from the capnp default and from the others, so a dropped or swapped field shows
class FakePanda(Panda):
  def __init__(self, **health):
    self.calls: list[tuple] = []
    self.health_dict = {
      "uptime": 123, "voltage": 12000, "current": 500, "safety_tx_blocked": 12, "safety_rx_invalid": 13,
      "tx_buffer_overflow": 14, "rx_buffer_overflow": 15, "faults": 0, "ignition_line": False, "ignition_can": False,
      "controls_allowed": True, "car_harness_status": 1, "safety_mode": SafetyModel.noOutput, "safety_param": 17,
      "fault_status": 2, "power_save_enabled": True, "heartbeat_lost": True, "alternative_experience": 16,
      "interrupt_load": 0.25, "fan_power": 33, "safety_rx_checks_invalid": True, "spi_error_count": 19,
      "sbu1_voltage_mV": 1000, "sbu2_voltage_mV": 2000, "som_reset_triggered": True, "nmi_reset": True,
      "hardfault_reset": True, "heartbeat_disabled": False, "sound_output_level": 21, "temperature": 30.0,
    }
    self.health_dict.update(health)

  def health(self):
    self.calls.append(("health",))
    return dict(self.health_dict)

  def can_health(self, i):
    # counters differ per bus, and each flag is set on a different set of buses
    lec = Panda.LEC_ERROR_CODE
    return {
      "bus_off": int(i == 0), "bus_off_cnt": 20 * i + 1, "error_warning": int(i == 1), "error_passive": int(i == 2),
      "last_error": lec[(i + 1) % 8], "last_stored_error": lec[(i + 3) % 8], "last_data_error": lec[(i + 5) % 8],
      "last_data_stored_error": lec[(i + 7) % 8], "receive_error_cnt": 20 * i + 2, "transmit_error_cnt": 20 * i + 3,
      "total_error_cnt": 20 * i + 4, "total_tx_lost_cnt": 20 * i + 5, "total_rx_lost_cnt": 20 * i + 6,
      "total_tx_cnt": 20 * i + 7, "total_rx_cnt": 20 * i + 8, "total_fwd_cnt": 20 * i + 9,
      "total_tx_checksum_error_cnt": 20 * i + 10, "can_speed": 20 * i + 11, "can_data_speed": 20 * i + 12,
      "canfd_enabled": int(i != 0), "brs_enabled": int(i != 1), "canfd_non_iso": int(i != 2), "irq0_call_rate": 20 * i + 13,
      "irq1_call_rate": 20 * i + 14, "irq2_call_rate": 20 * i + 15, "can_core_reset_count": 20 * i + 16,
    }

  def get_fan_rpm(self):
    self.calls.append(("get_fan_rpm",))
    return 4321

  def get_type(self):
    return bytes([TRES])

  def can_recv(self):
    self.calls.append(("can_recv",))
    return []

  def can_send_many(self, arr, *, fd=False, timeout=0):
    self.calls.append(("can_send_many", [tuple(f) for f in arr], timeout))

  def debug_read(self, maxlen=1024):
    self.calls.append(("debug_read",))
    return b""

  # the writes just get recorded
  def set_safety_mode(self, mode=SafetyModel.silent, param=0):
    self.calls.append(("set_safety_mode", mode, param))

  def set_alternative_experience(self, alternative_experience):
    self.calls.append(("set_alternative_experience", alternative_experience))

  def set_power_save(self, power_save_enabled=0):
    self.calls.append(("set_power_save", power_save_enabled))

  def send_heartbeat(self, engaged=True):
    self.calls.append(("send_heartbeat", engaged))

  def set_fan_power(self, percentage):
    self.calls.append(("set_fan_power", percentage))

  def set_ir_power(self, percentage):
    self.calls.append(("set_ir_power", percentage))

  def writes(self):
    return [c for c in self.calls if c[0] not in ("health", "can_recv", "debug_read", "get_fan_rpm")]

  def count(self, name):
    return sum(c[0] == name for c in self.calls)
