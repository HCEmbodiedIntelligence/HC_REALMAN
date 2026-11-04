"""Pure OmniPicker Modbus register helpers (no ROS/SDK dependency)."""

from dataclasses import dataclass

COMMAND_REGISTER = 10
STATUS_REGISTER = 20


def _u8(value: int, name: str) -> int:
    value = int(value)
    if not 0 <= value <= 255:
        raise ValueError(f"{name} must be in [0, 255], got {value}")
    return value


def normalized_to_register(position: float) -> int:
    value = float(position)
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"normalized gripper position must be in [0, 1], got {value}")
    return round(value * 255.0)


def encode_registers(position: int, velocity: int, force: int,
                     acceleration: int, deceleration: int) -> list[int]:
    return [_u8(position, "position"), _u8(velocity, "velocity"),
            _u8(force, "force"), _u8(acceleration, "acceleration"),
            _u8(deceleration, "deceleration"), 1]


@dataclass(frozen=True)
class OmniPickerStatus:
    fault: int
    state: int
    position: int
    velocity: int
    force: int

    @property
    def normalized_position(self) -> float:
        return self.position / 255.0


def decode_registers(data: list[int]) -> OmniPickerStatus:
    if len(data) != 5:
        raise ValueError(f"OmniPicker status must contain 5 registers, got {len(data)}")
    return OmniPickerStatus(*(int(x) for x in data))
