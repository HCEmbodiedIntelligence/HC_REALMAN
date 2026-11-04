"""Controller worker isolated from the ROS executor."""

from dataclasses import dataclass
import math
import threading
import time
from typing import Optional

from .config import ArmConfig, HandConfig
from .protocol import OmniPickerStatus
from .sdk import ArmSnapshot, RealManSession


@dataclass
class WorkerState:
    arm: Optional[ArmSnapshot] = None
    arm_stamp: float = 0.0
    hand: Optional[OmniPickerStatus] = None
    hand_stamp: float = 0.0
    connected: bool = False
    error: str = "not started"


class ControllerWorker:
    def __init__(self, arm_cfg: ArmConfig, hand_cfg: Optional[HandConfig],
                 initialize_runtime: bool, arms_control: bool, hands_control: bool,
                 control_rate_hz: float, state_rate_hz: float, hand_rate_hz: float,
                 command_timeout: float, baudrate: int, tool_voltage: int,
                 hand_params: tuple[int, int, int, int], max_velocity: float,
                 max_acceleration: float, sdk_timeout_ms: int):
        self.arm_cfg, self.hand_cfg = arm_cfg, hand_cfg
        self.arms_control = arms_control and arm_cfg.control_enabled
        self.hands_control = hands_control and bool(hand_cfg and hand_cfg.control_enabled)
        self.control_period = 1.0 / control_rate_hz
        self.state_period = 1.0 / state_rate_hz
        self.hand_period = 1.0 / hand_rate_hz
        self.command_timeout = command_timeout
        self.baudrate, self.tool_voltage = baudrate, tool_voltage
        self.hand_params = hand_params
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.session = RealManSession(arm_cfg, hand_cfg, initialize_runtime, sdk_timeout_ms)
        self._lock = threading.Lock()
        self._state = WorkerState()
        self._arm_target: Optional[tuple[float, ...]] = None
        self._arm_target_stamp = 0.0
        self._hand_target = 255
        self._hand_target_stamp = 0.0
        self._motion_gate = False
        self._command_position: Optional[list[float]] = None
        self._command_velocity = [0.0] * 6
        self._last_command_time = 0.0
        self._streaming = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"realman-{arm_cfg.side}", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._hold_once()
        self.session.disconnect()

    def snapshot(self) -> WorkerState:
        with self._lock:
            return WorkerState(**vars(self._state))

    def set_arm_target(self, target_deg: tuple[float, ...]) -> None:
        if len(target_deg) != 6 or any(not math.isfinite(x) for x in target_deg):
            raise ValueError("arm target must contain 6 finite values")
        if any(x < lo or x > hi for x, lo, hi in
               zip(target_deg, self.arm_cfg.lower_deg, self.arm_cfg.upper_deg)):
            raise ValueError(f"{self.arm_cfg.side} arm target outside configured limits")
        with self._lock:
            self._arm_target, self._arm_target_stamp = tuple(target_deg), time.monotonic()

    def set_hand_target(self, position: int) -> None:
        if not 0 <= position <= 255:
            raise ValueError("hand target must be in [0, 255]")
        with self._lock:
            self._hand_target, self._hand_target_stamp = int(position), time.monotonic()

    def set_motion_gate(self, enabled: bool) -> None:
        with self._lock:
            self._motion_gate = bool(enabled)

    def clear_targets(self) -> None:
        with self._lock:
            self._arm_target = None
            self._arm_target_stamp = 0.0
            self._hand_target_stamp = 0.0
            self._motion_gate = False
        self._hold_once()

    def clear_errors(self) -> None:
        self.session.clear_errors()

    def move_home(self) -> None:
        if not self.arms_control:
            raise PermissionError(f"{self.arm_cfg.side} arm is read-only")
        self.session.move_home()

    def _run(self) -> None:
        try:
            self.session.connect()
            if self.hand_cfg is not None:
                self.session.configure_hand(self.baudrate, self.tool_voltage)
            self._set_state(connected=True, error="")
        except Exception as exc:
            self._set_state(error=f"startup: {exc}")
            return
        next_arm_read = next_hand = 0.0
        while not self._stop.is_set():
            start = time.monotonic()
            try:
                with self._lock:
                    arm_target, arm_stamp = self._arm_target, self._arm_target_stamp
                    hand_target, hand_stamp = self._hand_target, self._hand_target_stamp
                    motion_gate = self._motion_gate
                arm_fresh = arm_target is not None and start - arm_stamp <= self.command_timeout
                if self.arms_control and motion_gate and arm_fresh:
                    self._send_smoothed_arm(arm_target, start)
                elif self._streaming:
                    self._hold_once()
                if start >= next_arm_read:
                    self._set_state(arm=self.session.read_arm(), arm_stamp=time.monotonic(), error="")
                    next_arm_read = start + self.state_period
                if self.hand_cfg is not None and start >= next_hand:
                    if self.hands_control and motion_gate and start - hand_stamp <= self.command_timeout:
                        self.session.command_hand(hand_target, *self.hand_params)
                    self._set_state(hand=self.session.read_hand(), hand_stamp=time.monotonic(), error="")
                    next_hand = start + self.hand_period
            except Exception as exc:
                self._set_state(error=str(exc))
            self._stop.wait(max(0.0, self.control_period - (time.monotonic() - start)))

    def _send_smoothed_arm(self, target: tuple[float, ...], now: float) -> None:
        if self._command_position is None:
            state = self.snapshot().arm
            if state is None:
                return
            self._command_position = list(state.joint_deg)
            self._command_velocity = [0.0] * 6
            self._last_command_time = now
        dt = min(max(now - self._last_command_time, 0.002), 0.05)
        output = []
        for index, desired in enumerate(target):
            position = self._command_position[index]
            velocity = self._command_velocity[index]
            error = desired - position
            desired_velocity = max(-self.max_velocity, min(self.max_velocity, error / 0.10))
            dv = max(-self.max_acceleration * dt,
                     min(self.max_acceleration * dt, desired_velocity - velocity))
            velocity = max(-self.max_velocity, min(self.max_velocity, velocity + dv))
            step = velocity * dt
            if abs(step) > abs(error):
                position, velocity = desired, 0.0
            else:
                position += step
            self._command_position[index] = position
            self._command_velocity[index] = velocity
            output.append(position)
        self.session.command_arm(tuple(output))
        self._last_command_time = now
        self._streaming = True

    def _hold_once(self) -> None:
        if not self._streaming or not self.session.connected:
            return
        try:
            state = self.session.read_arm()
            self.session.command_arm(state.joint_deg)
        except Exception as exc:
            self._set_state(error=f"hold failed: {exc}")
        finally:
            self._streaming = False
            self._command_position = None
            self._command_velocity = [0.0] * 6

    def _set_state(self, **values) -> None:
        with self._lock:
            for key, value in values.items():
                setattr(self._state, key, value)
