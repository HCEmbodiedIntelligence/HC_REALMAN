"""ROS 2 entry point for dual ECO65-B + dual OmniPicker."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt16MultiArray
from std_srvs.srv import Trigger

from .config import load_config
from .protocol import normalized_to_register
from .worker import ControllerWorker


class HcRealmanNode(Node):
    def __init__(self):
        super().__init__("hc_realman_driver")
        path = self.declare_parameter("hardware_config_path", "").value
        if not path:
            raise ValueError("hardware_config_path is required")
        cfg = load_config(path)
        arms_control = bool(self.declare_parameter("arms_control", cfg.get("arms_control", False)).value)
        hands_control = bool(self.declare_parameter("hands_control", cfg.get("hands_control", False)).value)
        self.state_timeout = float(cfg.get("state_timeout_sec", 0.5))
        self.command_timeout = float(cfg.get("command_timeout_sec", 0.25))
        hands_raw = cfg.get("hands", {})
        # Powering an OmniPicker makes its firmware close once to find zero.
        # Keep the entire hand transport off unless hand control is explicitly enabled.
        hand_cfgs = ({h.side: h for h in cfg["parsed_hands"]} if hands_control else {})
        arms = cfg["parsed_arms"]
        hand_params = tuple(int(hands_raw.get(k, 80)) for k in
                            ("velocity", "force", "acceleration", "deceleration"))
        common = dict(
            arms_control=arms_control, hands_control=hands_control,
            control_rate_hz=float(cfg.get("control_rate_hz", 100.0)),
            state_rate_hz=float(cfg.get("state_rate_hz", 50.0)),
            hand_rate_hz=float(hands_raw.get("command_rate_hz", 20.0)),
            command_timeout=self.command_timeout,
            baudrate=int(hands_raw.get("baudrate", 115200)),
            tool_voltage=int(hands_raw.get("tool_voltage", 24)),
            hand_params=hand_params,
            max_velocity=float(cfg.get("max_joint_velocity_deg_s", 45.0)),
            max_acceleration=float(cfg.get("max_joint_acceleration_deg_s2", 120.0)),
            sdk_timeout_ms=round(float(hands_raw.get("response_timeout_sec", 0.2)) * 1000),
        )
        self.workers = {
            arms[0].side: ControllerWorker(arms[0], hand_cfgs.get(arms[0].side), True, **common),
            arms[1].side: ControllerWorker(arms[1], hand_cfgs.get(arms[1].side), False, **common),
        }
        self.joint_names = tuple(n for arm in arms for n in arm.joints)
        self.hand_names = tuple(hand_cfgs[s].joint_name for s in ("left", "right") if s in hand_cfgs)
        self._last_ready = None
        self._command_source = ""
        self._command_source_stamp = 0.0

        self.joint_pub = self.create_publisher(JointState, "/io_teleop/joint_states", 10)
        ready_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, "/io_teleop/hardware_ready", ready_qos)
        self.hand_pub = self.create_publisher(JointState, "/observation/hands/joint_states", 10)
        self.hand_raw_pub = self.create_publisher(UInt16MultiArray, "/status/hands/raw", 10)
        self.observation_pub = self.create_publisher(JointState, "/observation/joint_states", 10)
        self.status_ready_pub = self.create_publisher(Bool, "/status/hardware/ready", ready_qos)
        self.command_sub = self.create_subscription(
            JointState, "/io_teleop/joint_cmd",
            lambda msg: self._on_joint_command(msg, "io_teleop"), 10)
        self.action_sub = self.create_subscription(
            JointState, "/action/joint_states",
            lambda msg: self._on_joint_command(msg, "action"), 10)
        self.hand_subs = []
        for side in ("left", "right"):
            self.hand_subs.append(self.create_subscription(
                JointState, f"/action/{side}_hand/joint_states",
                lambda msg, side=side: self._on_hand_command(side, msg), 10))
        self.reset_srv = self.create_service(Trigger, "/io_teleop/reset_robot", self._on_reset)
        self.reset_compat_srv = self.create_service(Trigger, "/io_teleop_reset_robot", self._on_reset)
        self.timer = self.create_timer(0.02, self._publish)
        for worker in self.workers.values():
            worker.start()
        self.get_logger().info(
            f"HC_REALMAN started: arms_control={arms_control}, hands_control={hands_control}; "
            "OmniPicker transport=tool RS485 Modbus-RTU")

    def destroy_node(self):
        for worker in self.workers.values():
            worker.stop()
        return super().destroy_node()

    def _on_joint_command(self, msg: JointState, source: str) -> None:
        now = time.monotonic()
        if (self._command_source and self._command_source != source and
                now - self._command_source_stamp < 0.5):
            self.get_logger().error(
                f"reject {source} joint command: active source is {self._command_source}")
            return
        self._command_source, self._command_source_stamp = source, now
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self.get_logger().error("reject joint_cmd: names/positions mismatch or duplicate names")
            return
        values = dict(zip(msg.name, msg.position))
        for side, worker in self.workers.items():
            if not all(name in values for name in worker.arm_cfg.joints):
                continue
            target = tuple(math.degrees(float(values[name])) for name in worker.arm_cfg.joints)
            try:
                worker.set_arm_target(target)
            except ValueError as exc:
                self.get_logger().error(f"reject {side} arm command: {exc}")

    def _on_hand_command(self, side: str, msg: JointState) -> None:
        expected = self.workers[side].hand_cfg.joint_name if self.workers[side].hand_cfg else ""
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        if expected not in msg.name:
            self.get_logger().error(f"reject {side} hand command: missing joint {expected}")
            return
        value = msg.position[msg.name.index(expected)]
        if not math.isfinite(value):
            return
        try:
            self.workers[side].set_hand_target(normalized_to_register(value))
        except ValueError as exc:
            self.get_logger().error(f"reject {side} hand command: {exc}")

    def _publish(self) -> None:
        now_mono = time.monotonic()
        snapshots = {side: worker.snapshot() for side, worker in self.workers.items()}
        arm_fresh = all(s.arm is not None and now_mono - s.arm_stamp <= self.state_timeout
                        for s in snapshots.values())
        if arm_fresh:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.joint_names)
            msg.position = [math.radians(v) for side in ("left", "right")
                            for v in snapshots[side].arm.joint_deg]
            self.joint_pub.publish(msg)
            self.observation_pub.publish(msg)
        hand_states = [(side, snapshots[side].hand) for side in ("left", "right")
                       if snapshots[side].hand is not None]
        if hand_states:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [self.workers[s].hand_cfg.joint_name for s, _ in hand_states]
            msg.position = [h.normalized_position for _, h in hand_states]
            self.hand_pub.publish(msg)
            raw = UInt16MultiArray()
            raw.data = [x for _, h in hand_states
                        for x in (h.fault, h.state, h.position, h.velocity, h.force)]
            self.hand_raw_pub.publish(raw)
        ready = all(s.connected and s.arm is not None and
                    now_mono - s.arm_stamp <= self.state_timeout and not s.error and
                    not self._arm_has_fault(s.arm.error)
                    for s in snapshots.values())
        if self.hand_names:
            ready = ready and all(s.hand is not None and
                                  now_mono - s.hand_stamp <= self.state_timeout and
                                  s.hand.fault == 0
                                  for s in snapshots.values())
        self.ready_pub.publish(Bool(data=ready))
        self.status_ready_pub.publish(Bool(data=ready))
        for worker in self.workers.values():
            worker.set_motion_gate(ready)
        if ready != self._last_ready:
            log = self.get_logger().info if ready else self.get_logger().warning
            details = "; ".join(f"{side}={s.error or 'ok'}" for side, s in snapshots.items())
            log(f"hardware_ready={ready}: {details}")
            self._last_ready = ready

    def _on_reset(self, request, response):
        del request
        errors = []
        for worker in self.workers.values():
            worker.clear_targets()
        for side, worker in self.workers.items():
            try:
                worker.clear_errors()
            except Exception as exc:
                errors.append(f"{side}: {exc}")
        response.success = not errors
        response.message = "errors cleared" if not errors else "; ".join(errors)
        return response

    @staticmethod
    def _arm_has_fault(error) -> bool:
        if error is None:
            return False
        if isinstance(error, dict):
            return int(error.get("err_len", len(error.get("err", [])))) > 0
        return bool(error)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HcRealmanNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
