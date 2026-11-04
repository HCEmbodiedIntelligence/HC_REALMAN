"""Configuration parsing and validation without ROS dependencies."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import yaml


@dataclass(frozen=True)
class ArmConfig:
    side: str
    ip: str
    port: int
    joints: tuple[str, ...]
    lower_deg: tuple[float, ...]
    upper_deg: tuple[float, ...]
    home_deg: tuple[float, ...]
    control_enabled: bool


@dataclass(frozen=True)
class HandConfig:
    side: str
    device_id: int
    joint_name: str
    control_enabled: bool


def _arm(side: str, raw: dict[str, Any]) -> ArmConfig:
    joints = tuple(str(x) for x in raw["joints"])
    lower = tuple(float(x) for x in raw["lower_limits_deg"])
    upper = tuple(float(x) for x in raw["upper_limits_deg"])
    home = tuple(float(x) for x in raw["home_deg"])
    if len(joints) != 6 or len(set(joints)) != 6:
        raise ValueError(f"{side}_arm.joints must contain 6 unique names")
    if not (len(lower) == len(upper) == len(home) == 6):
        raise ValueError(f"{side}_arm limits/home must each contain 6 values")
    if any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError(f"{side}_arm contains invalid joint limits")
    if any(not lo <= h <= hi for lo, h, hi in zip(lower, home, upper)):
        raise ValueError(f"{side}_arm home is outside joint limits")
    return ArmConfig(side, str(raw["ip"]), int(raw.get("port", 8080)), joints,
                     lower, upper, home, bool(raw.get("control_enabled", True)))


def _hand(side: str, raw: dict[str, Any]) -> HandConfig:
    device_id = int(raw["device_id"])
    if not 1 <= device_id <= 247:
        raise ValueError(f"hands.{side}.device_id must be in [1, 247]")
    return HandConfig(side, device_id, str(raw["joint_name"]),
                      bool(raw.get("control_enabled", True)))


def load_config(path: str) -> dict[str, Any]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("hardware config root must be a mapping")
    left_arm, right_arm = _arm("left", raw["left_arm"]), _arm("right", raw["right_arm"])
    if set(left_arm.joints) & set(right_arm.joints):
        raise ValueError("left/right arm joint names overlap")
    if left_arm.ip == right_arm.ip and left_arm.port == right_arm.port:
        raise ValueError("left/right arms cannot use the same IP and port")
    for key in ("control_rate_hz", "state_rate_hz", "command_timeout_sec", "state_timeout_sec"):
        if float(raw.get(key, 0.0)) <= 0.0:
            raise ValueError(f"{key} must be positive")
    for key in ("max_joint_velocity_deg_s", "max_joint_acceleration_deg_s2"):
        if float(raw.get(key, 0.0)) <= 0.0:
            raise ValueError(f"{key} must be positive")
    hands_raw = raw.get("hands", {})
    if hands_raw.get("enabled", True):
        if int(hands_raw.get("baudrate", 0)) not in (9600, 115200, 460800):
            raise ValueError("hands.baudrate must be 9600, 115200, or 460800")
        if int(hands_raw.get("tool_voltage", -1)) not in (0, 5, 12, 24):
            raise ValueError("hands.tool_voltage must be 0, 5, 12, or 24")
        if float(hands_raw.get("command_rate_hz", 0.0)) <= 0.0:
            raise ValueError("hands.command_rate_hz must be positive")
    raw["parsed_arms"] = (left_arm, right_arm)
    raw["parsed_hands"] = (
        _hand("left", hands_raw["left"]), _hand("right", hands_raw["right"])
    ) if hands_raw.get("enabled", True) else ()
    return raw
