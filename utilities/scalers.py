import math


def map_exponential(value: float, base: float) -> float:
    """Takes a value in [-1,1] and maps it to an exponential curve."""
    assert base > 1
    return math.copysign((base ** abs(value) - 1) / (base - 1), value)


def apply_deadzone(value: float, deadzone: float) -> float:
    range = 1.0 - deadzone
    scale = 1.0 / range
    if value >= deadzone:
        return (value - deadzone) * scale
    elif value <= -deadzone:
        return (value + deadzone) * scale
    else:
        return 0.0
