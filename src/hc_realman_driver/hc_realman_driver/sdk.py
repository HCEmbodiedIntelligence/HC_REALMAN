"""Thin, mockable adapter around RealMan Python API2."""

from dataclasses import dataclass
import threading
from typing import Optional

from .config import ArmConfig, HandConfig
from .protocol import (COMMAND_REGISTER, STATUS_REGISTER, OmniPickerStatus,
                       decode_registers, encode_registers)


@dataclass
class ArmSnapshot:
    joint_deg: tuple[float, ...]
    pose: tuple[float, ...]
    error: object


class RealManSession:
    """One controller connection. All calls are serialized per controller."""

    _runtime_ready = threading.Event()
    _runtime_init_lock = threading.Lock()

    def __init__(self, arm_cfg: ArmConfig, hand_cfg: Optional[HandConfig],
                 initialize_runtime: bool, timeout_ms: int = 200):
        self.arm_cfg = arm_cfg
        self.hand_cfg = hand_cfg
        self._initialize_runtime = initialize_runtime
        self._timeout_ms = timeout_ms
        self._arm = None
        self._lock = threading.RLock()
        self._read_type = None
        self._write_type = None

    def connect(self) -> None:
        from Robotic_Arm.rm_robot_interface import (
            RoboticArm, rm_modbus_rtu_read_params_t,
            rm_modbus_rtu_write_params_t, rm_thread_mode_e,
        )

        if self._initialize_runtime:
            with self._runtime_init_lock:
                arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
                self._runtime_ready.set()
        else:
            if not self._runtime_ready.wait(timeout=5.0):
                raise RuntimeError("RealMan global runtime was not initialized by the first arm")
            arm = RoboticArm()
        arm.rm_set_timeout(self._timeout_ms)
        handle = arm.rm_create_robot_arm(self.arm_cfg.ip, self.arm_cfg.port, level=2)
        if handle.id == -1:
            raise ConnectionError(
                f"{self.arm_cfg.side} arm connect failed: {self.arm_cfg.ip}:{self.arm_cfg.port}"
            )
        if int(getattr(arm, "arm_dof", 0)) not in (0, 6):
            arm.rm_delete_robot_arm()
            raise RuntimeError(
                f"{self.arm_cfg.side} arm reports {arm.arm_dof} DOF, expected ECO65-B 6 DOF"
            )
        self._arm = arm
        self._read_type = rm_modbus_rtu_read_params_t
        self._write_type = rm_modbus_rtu_write_params_t

    @property
    def connected(self) -> bool:
        return self._arm is not None

    def disconnect(self) -> None:
        with self._lock:
            arm, self._arm = self._arm, None
            if arm is not None:
                arm.rm_delete_robot_arm()

    def read_arm(self) -> ArmSnapshot:
        with self._lock:
            code, state = self._require_arm().rm_get_current_arm_state()
        if code != 0:
            raise IOError(f"rm_get_current_arm_state returned {code}")
        joints = tuple(float(x) for x in state.get("joint", ()))
        if len(joints) != 6:
            raise IOError(f"controller returned {len(joints)} joints, expected 6")
        return ArmSnapshot(joints, tuple(state.get("pose", ())), state.get("err"))

    def command_arm(self, joint_deg: tuple[float, ...]) -> None:
        if len(joint_deg) != 6:
            raise ValueError("ECO65-B command must contain 6 joints")
        # API2 1.1.5 Python binding allocates float[7] internally. Supplying the
        # seventh zero avoids an ABI-dependent partial initializer.
        command = [*map(float, joint_deg), 0.0]
        with self._lock:
            code = self._require_arm().rm_movej_canfd(command, False)
        if code != 0:
            raise IOError(f"rm_movej_canfd returned {code}")

    def configure_hand(self, baudrate: int, tool_voltage: int) -> None:
        if self.hand_cfg is None:
            return
        voltage_code = {0: 0, 5: 1, 12: 2, 24: 3}.get(int(tool_voltage))
        if voltage_code is None:
            raise ValueError("tool_voltage must be one of 0, 5, 12, 24")
        with self._lock:
            arm = self._require_arm()
            code = arm.rm_set_tool_voltage(voltage_code)
            if code == 0:
                code = arm.rm_set_tool_rs485_mode(0, int(baudrate))
        if code != 0:
            raise IOError(f"configure tool RS485 returned {code}")

    def command_hand(self, position: int, velocity: int, force: int,
                     acceleration: int, deceleration: int) -> None:
        if self.hand_cfg is None:
            raise RuntimeError("hand is disabled")
        values = encode_registers(position, velocity, force, acceleration, deceleration)
        param = self._write_type(COMMAND_REGISTER, self.hand_cfg.device_id, 1,
                                 len(values), values)
        with self._lock:
            code = self._require_arm().rm_write_modbus_rtu_registers(param)
        if code != 0:
            raise IOError(f"write OmniPicker registers returned {code}")

    def read_hand(self) -> OmniPickerStatus:
        if self.hand_cfg is None:
            raise RuntimeError("hand is disabled")
        param = self._read_type(STATUS_REGISTER, self.hand_cfg.device_id, 1, 5)
        with self._lock:
            code, values = self._require_arm().rm_read_modbus_rtu_holding_registers(param)
        if code != 0:
            raise IOError(f"read OmniPicker registers returned {code}")
        return decode_registers(values)

    def clear_errors(self) -> None:
        with self._lock:
            arm = self._require_arm()
            code = arm.rm_clear_system_err()
            if code == 0:
                for joint in range(1, 7):
                    code = arm.rm_set_joint_clear_err(joint)
                    if code != 0:
                        break
        if code != 0:
            raise IOError(f"clear errors returned {code}")

    def move_home(self, speed: int = 10) -> None:
        with self._lock:
            code = self._require_arm().rm_movej(list(self.arm_cfg.home_deg), speed, 0, 0, 1)
        if code != 0:
            raise IOError(f"home rm_movej returned {code}")

    def _require_arm(self):
        if self._arm is None:
            raise ConnectionError("RealMan session is not connected")
        return self._arm
