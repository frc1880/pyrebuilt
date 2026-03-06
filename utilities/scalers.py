import math


def map_exponential(value: float, base: float) -> float:
    """Takes a value in [-1,1] and maps it to an exponential curve."""
    assert base > 1
    return math.copysign((base ** abs(value) - 1) / (base - 1), value)
