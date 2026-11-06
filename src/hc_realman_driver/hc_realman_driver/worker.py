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
    command_count: int = 0
    command_failures: int = 0
    max_command_gap_sec: float = 0.0
    last_command_stamp: float = 0.0


class ControllerWorker:
    def __init__(self, arm_cfg: ArmConfig, hand_cfg: Optional[HandConfig],
                 initialize_runtime: bool, arms_control: bool, hands_control: bool,
                 control_rate_hz: float, state_rate_hz: float, hand_rate_hz: float,
                 command_timeout: float, baudrate: int, tool_voltage: int,
                 hand_params: tuple[int, int, int, int], max_velocity: float,
                 max_acceleration: float, sdk_timeout_ms: int,
                 udp_target_ip: str = "192.168.1.100", udp_port: int = 8090,
                 udp_cycle_ms: int = 10):
        self.arm_cfg, self.hand_cfg = arm_cfg, hand_cfg
        self.arms_control = arms_control and arm_cfg.control_enabled
        self.hands_control = hands_control and bool(hand_cfg and hand_cfg.control_enabled)
        self.control_period = 1.0 / control_rate_hz
        del state_rate_hz  # State is delivered asynchronously by controller UDP.
        self.hand_period = 1.0 / hand_rate_hz
        self.command_timeout = command_timeout
        self.baudrate, self.tool_voltage = baudrate, tool_voltage
        self.hand_params = hand_params
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.udp_target_ip = str(udp_target_ip)
        self.udp_port = int(udp_port)
        self.udp_cycle_ms = int(udp_cycle_ms)
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
        self._hold_requested = False
        self._last_send_stamp = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"realman-{arm_cfg.side}", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self.clear_targets()
        deadline = time.monotonic() + 0.25
        while self._streaming and self._thread.is_alive() and time.monotonic() < deadline:
            time.sleep(min(self.control_period, 0.01))
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            self._set_state(error="command worker did not stop; controller left connected")
            return
        self.session.disconnect()

    def snapshot(self) -> WorkerState:
        with self._lock:
            return WorkerState(**vars(self._state))

    def validate_arm_target(self, target_deg: tuple[float, ...]) -> None:
        if len(target_deg) != 6 or any(not math.isfinite(x) for x in target_deg):
            raise ValueError("arm target must contain 6 finite values")
        if any(x < lo or x > hi for x, lo, hi in
               zip(target_deg, self.arm_cfg.lower_deg, self.arm_cfg.upper_deg)):
            raise ValueError(f"{self.arm_cfg.side} arm target outside configured limits")

    def set_arm_target(self, target_deg: tuple[float, ...], stamp: float | None = None) -> None:
        self.validate_arm_target(target_deg)
        with self._lock:
            self._arm_target = tuple(target_deg)
            self._arm_target_stamp = time.monotonic() if stamp is None else float(stamp)

    def set_hand_target(self, position: int) -> None:
        if not 0 <= position <= 255:
            raise ValueError("hand target must be in [0, 255]")
        with self._lock:
            self._hand_target, self._hand_target_stamp = int(position), time.monotonic()

    def set_motion_gate(self, enabled: bool) -> None:
        with self._lock:
            enabled = bool(enabled)
            if self._motion_gate and not enabled:
                self._hold_requested = True
            self._motion_gate = enabled

    def clear_targets(self) -> None:
        with self._lock:
            self._arm_target = None
            self._arm_target_stamp = 0.0
            self._hand_target_stamp = 0.0
            self._motion_gate = False
            self._hold_requested = True

    def clear_errors(self) -> None:
        self.session.clear_errors()

    def move_home(self) -> None:
        if not self.arms_control:
            raise PermissionError(f"{self.arm_cfg.side} arm is read-only")
        self.session.move_home()

    def _run(self) -> None:
        try:
            self.session.connect()
            self.session.start_realtime_state(
                self._on_arm_state, self.udp_target_ip,
                self.udp_port, self.udp_cycle_ms,
            )
            if self.hand_cfg is not None:
                self.session.configure_hand(self.baudrate, self.tool_voltage)
            self._set_state(connected=True, error="")
        except Exception as exc:
            self._set_state(error=f"startup: {exc}")
            return
        next_deadline = time.monotonic()
        next_hand = 0.0
        while not self._stop.is_set():
            start = time.monotonic()
            try:
                with self._lock:
                    arm_target, arm_stamp = self._arm_target, self._arm_target_stamp
                    hand_target, hand_stamp = self._hand_target, self._hand_target_stamp
                    motion_gate = self._motion_gate
                    hold_requested = self._hold_requested
                    self._hold_requested = False
                arm_fresh = arm_target is not None and start - arm_stamp <= self.command_timeout
                if hold_requested or (self._streaming and not (
                        self.arms_control and motion_gate and arm_fresh)):
                    self._hold_once()
                elif self.arms_control and motion_gate and arm_fresh:
                    self._send_smoothed_arm(arm_target, start)
                if self.hand_cfg is not None and start >= next_hand:
                    if self.hands_control and motion_gate and start - hand_stamp <= self.command_timeout:
                        self.session.command_hand(hand_target, *self.hand_params)
                    self._set_state(hand=self.session.read_hand(), hand_stamp=time.monotonic(), error="")
                    next_hand = start + self.hand_period
            except Exception as exc:
                self._streaming = False
                self._set_state(error=str(exc), command_failures=self.snapshot().command_failures + 1)
            next_deadline += self.control_period
            now = time.monotonic()
            if next_deadline < now - self.control_period:
                next_deadline = now + self.control_period
            self._stop.wait(max(0.0, next_deadline - now))

    def _on_arm_state(self, arm: ArmSnapshot, stamp: float) -> None:
        self._set_state(arm=arm, arm_stamp=stamp)

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
            # Braking-distance velocity gives a bounded triangular/trapezoidal
            # response without the long exponential tail of error / tau.
            braking_velocity = math.sqrt(2.0 * self.max_acceleration * abs(error))
            desired_velocity = math.copysign(
                min(self.max_velocity, braking_velocity), error
            ) if error else 0.0
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
        self._record_command(now)
        self._last_command_time = now
        self._streaming = True
        self._set_state(error="")

    def _hold_once(self) -> None:
        if not self._streaming or not self.session.connected:
            return
        try:
            self.session.slow_stop()
        except Exception as exc:
            # If the controller rejects slow-stop, make one best-effort hold
            # command from the latest UDP state without issuing a blocking read.
            try:
                state = self.snapshot().arm
                if state is not None:
                    now = time.monotonic()
                    self.session.command_arm(state.joint_deg)
                    self._record_command(now)
            except Exception as fallback_exc:
                self._set_state(error=f"stop failed: {exc}; hold failed: {fallback_exc}")
        finally:
            self._streaming = False
            # A later command starts a new stream.  Do not report the idle
            # time between streams as a missed real-time command deadline.
            self._last_send_stamp = 0.0
            self._command_position = None
            self._command_velocity = [0.0] * 6

    def _record_command(self, stamp: float) -> None:
        with self._lock:
            gap = stamp - self._last_send_stamp if self._last_send_stamp else 0.0
            self._last_send_stamp = stamp
            self._state.command_count += 1
            self._state.last_command_stamp = stamp
            if gap > self._state.max_command_gap_sec:
                self._state.max_command_gap_sec = gap

    def _set_state(self, **values) -> None:
        with self._lock:
            for key, value in values.items():
                setattr(self._state, key, value)
