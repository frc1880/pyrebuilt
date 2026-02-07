import enum

from phoenix6 import CANBus


class CanbusId:
    """CAN bus ID for multiple buses on the robot"""

    DRIVETRAIN = CANBus("rio")
    INTAKE = CANBus("rio")
    SHOOTER = CANBus("rio")
    INDEXER = CANBus("rio")


@enum.unique
class TalonId(enum.IntEnum):
    """CAN ID for CTRE Talon motor controllers (e.g. Talon FX, Talon SRX)."""

    INTAKE_MOTOR = 22
    SHOOTER_MOTOR = 6
    INDEXER_ROTATION_MOTOR = 12
    INDEXER_EXIT_MOTOR = 13


@enum.unique
class CancoderId(enum.IntEnum):
    """CAN ID for CTRE CANcoder."""

    pass


@enum.unique
class SparkId(enum.IntEnum):
    """CAN ID for REV SPARK motor controllers (Spark Max, Spark Flex)."""

    pass


@enum.unique
class DioChannel(enum.IntEnum):
    """roboRIO Digital I/O channel number."""

    pass


@enum.unique
class PwmChannel(enum.IntEnum):
    """roboRIO PWM output channel number."""

    pass


@enum.unique
class RioSerialNumber(enum.StrEnum):
    """roboRIO serial number"""

    TEST_BOT = "12345678"
    COMP_BOT = "87654321"


@enum.unique
class AnalogChannel(enum.IntEnum):
    """roboRIO Analog input channel number"""

    pass
