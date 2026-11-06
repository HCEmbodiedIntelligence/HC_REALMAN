"""Thin, mockable adapter around RealMan Python API2."""

from dataclasses import dataclass
import threading
import time
from typing import Callable, Optional

from .config import ArmConfig, HandConfig
from .protocol import (COMMAND_REGISTER, STATUS_REGISTER, OmniPickerStatus,
                       decode_registers, encode_registers)


@dataclass
class ArmSnapshot:
    joint_deg: tuple[float, ...]
    pose: tuple[float, ...]
    error: object


class _RealtimeStateDispatcher:
    """Own the single process-global API2 UDP callback and route by arm IP."""

    _lock = threading.Lock()
    _handlers: dict[str, Callable[[ArmSnapshot, float], None]] = {}
    _callback_ref = None
    _registered = False

    @classmethod
    def register(cls, arm, ip: str,
                 handler: Callable[[ArmSnapshot, float], None]) -> None:
        with cls._lock:
            cls._handlers[ip] = handler
            if cls._registered:
                return
            from Robotic_Arm.rm_robot_interface import rm_realtime_arm_state_callback_ptr

            @rm_realtime_arm_state_callback_ptr
            def receive(data):
                try:
                    if int(data.errCode) != 0:
                        return
                    arm_ip = bytes(data.arm_ip).split(b"\0", 1)[0].decode("ascii")
                    joint = tuple(float(x) for x in data.joint_status.joint_position[:6])
                    waypoint = data.waypoint
                    pose = (
                        float(waypoint.position.x), float(waypoint.position.y),
                        float(waypoint.position.z), float(waypoint.euler.rx),
                        float(waypoint.euler.ry), float(waypoint.euler.rz),
                    )
                    snapshot = ArmSnapshot(joint, pose, data.err.to_dict())
                    with cls._lock:
                        target = cls._handlers.get(arm_ip)
                    if target is not None:
                        target(snapshot, time.monotonic())
                except Exception:
                    # Never let an exception escape through the ctypes callback.
                    return

            cls._callback_ref = receive  # ctypes callbacks require a strong reference.
            arm.rm_realtime_arm_state_call_back(receive)
            cls._registered = True

    @classmethod
    def unregister(cls, ip: str) -> None:
        with cls._lock:
            cls._handlers.pop(ip, None)


def arm_has_fault(error: object) -> bool:
    """Return whether an API2 arm-state error payload contains a real fault.

    API2 reports the healthy state as ``{"err_len": 1, "err": ["0"]}``, so
    ``err_len`` alone cannot be used as the fault flag.
    """
    if error is None:
        return False
    if not isinstance(error, dict):
        return bool(error)

    codes = error.get("err", ())
    if not isinstance(codes, (list, tuple)):
        codes = (codes,)
    try:
        count = int(error.get("err_len", len(codes)))
    except (TypeError, ValueError):
        return True
    if count <= 0:
        return False
    if not codes:
        return True

    for code in codes[:count]:
        try:
            if int(str(code), 0) != 0:
                return True
        except (TypeError, ValueError):
            if bool(code):
                return True
    return count > len(codes)


class RealManSession:
    """One controller connection. Calls are serialized per controller."""

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

        if not self._initialize_runtime and not self._runtime_ready.wait(timeout=10.0):
            raise RuntimeError("RealMan global runtime was not initialized by the first arm")
        # API2 owns process-global state while creating controller handles.  Keep
        # both create calls serialized; concurrent creation can leave a socket
        # connected while rm_create_robot_arm reports an invalid handle.
        with self._runtime_init_lock:
            if self._initialize_runtime:
                arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            else:
                arm = RoboticArm()
            arm.rm_set_timeout(self._timeout_ms)
            handle = arm.rm_create_robot_arm(self.arm_cfg.ip, self.arm_cfg.port, level=2)
            if handle.id == -1:
                # Do not retry on the same process-global API2 runtime. A failed
                # create can retain its TCP socket and a second create then
                # poisons the controller session. The worker reconnect loop
                # starts a clean attempt after disconnect/cleanup.
                try:
                    arm.rm_delete_robot_arm()
                finally:
                    raise ConnectionError(
                        f"{self.arm_cfg.side} arm connect failed: "
                        f"{self.arm_cfg.ip}:{self.arm_cfg.port}"
                    )
            if int(getattr(arm, "arm_dof", 0)) not in (0, 6):
                arm.rm_delete_robot_arm()
                raise RuntimeError(
                    f"{self.arm_cfg.side} arm reports {arm.arm_dof} DOF, expected ECO65-B 6 DOF"
                )
            self._arm = arm
            self._read_type = rm_modbus_rtu_read_params_t
            self._write_type = rm_modbus_rtu_write_params_t
            if self._initialize_runtime:
                self._runtime_ready.set()

    @property
    def connected(self) -> bool:
        return self._arm is not None

    def disconnect(self) -> None:
        _RealtimeStateDispatcher.unregister(self.arm_cfg.ip)
        with self._lock:
            arm, self._arm = self._arm, None
            if arm is not None:
                arm.rm_delete_robot_arm()

    def start_realtime_state(self, handler: Callable[[ArmSnapshot, float], None],
                             target_ip: str, port: int, cycle_ms: int) -> None:
        from Robotic_Arm.rm_robot_interface import rm_realtime_push_config_t

        if cycle_ms <= 0 or cycle_ms % 5:
            raise ValueError("UDP realtime cycle must be a positive multiple of 5 ms")
        with self._lock:
            arm = self._require_arm()
            _RealtimeStateDispatcher.register(arm, self.arm_cfg.ip, handler)
            # API2 encodes the configured value in 5 ms units despite naming
            # the field 'cycle'; cycle=2 was measured as a 10 ms stream.
            config = rm_realtime_push_config_t(
                cycle=cycle_ms // 5, enable=True, port=int(port),
                force_coordinate=-1, ip=str(target_ip),
            )
            code = arm.rm_set_realtime_push(config)
        if code != 0:
            _RealtimeStateDispatcher.unregister(self.arm_cfg.ip)
            raise IOError(f"configure realtime UDP state returned {code}")

    def read_arm(self) -> ArmSnapshot:
        with self._lock:
            code, state = self._require_arm().rm_get_current_arm_state()
            # API2 occasionally reports a single receive error (-2) on an
            # otherwise healthy persistent connection.  Retry once so only a
            # sustained communication failure withdraws hardware_ready.
            if code != 0:
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
            # High-follow is enabled per arm only after timing qualification.
            # The online single arm at 200 Hz measured a 6.6 ms worst-case gap,
            # below API2's strict 10 ms limit. Offline/unqualified arms retain
            # stable low-follow mode until separately benchmarked.
            code = self._require_arm().rm_movej_canfd(command, self.arm_cfg.high_follow)
        if code != 0:
            raise IOError(f"rm_movej_canfd returned {code}")

    def slow_stop(self) -> None:
        """Stop the active controller trajectory using the vendor deceleration path."""
        with self._lock:
            code = self._require_arm().rm_set_arm_slow_stop()
        if code != 0:
            raise IOError(f"rm_set_arm_slow_stop returned {code}")

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
