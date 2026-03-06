from phoenix6.controls.solid_color import SolidColor
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor
from wpilib import DriverStation

from components.ballistics import Ballistics
from ids import CanbusId, CandleId
from utilities.game import is_hub_active


class Leds:
    """
    Light patterns for driver feedback.
    """

    # Injected by MagicBot
    ballistics: Ballistics

    # Colours
    RED = RGBWColor(255, 0, 0, 0)
    GREEN = RGBWColor(0, 255, 0, 0)
    YELLOW = RGBWColor(255, 255, 0, 0)
    BLUE = RGBWColor(0, 0, 255, 0)
    WHITE = RGBWColor(0, 0, 0, 255)
    ORANGE = RGBWColor(255, 165, 0, 0)

    led_start: int = 0
    led_end: int = 7
    brightness: float = 1.0

    def __init__(self) -> None:
        self._candle = CANdle(device_id=CandleId.LED, canbus=CanbusId.LEDS)

        # Default pattern
        self._pattern = self._solid(self.WHITE)

    def _solid(self, color: RGBWColor) -> SolidColor:
        return SolidColor(self.led_start, self.led_end, color)

    def intake(self) -> None:
        self._pattern = self._solid(self.BLUE)

        # self._pattern = RainbowAnimation(

    #     self.led_start, self.led_end, brightness=self.brightness
    #  )

    def climb(self) -> None:
        self._pattern = self._solid(self.YELLOW)

    def in_range(self) -> None:
        self._pattern = self._solid(self.GREEN)

    def not_in_range(self) -> None:
        self._pattern = self._solid(self.RED)

    def inactive(self) -> None:
        self._pattern = self._solid(self.WHITE)

    def teleop_lights(self) -> None:
        if not is_hub_active():
            self.inactive()
            return

        if self.ballistics.in_range():
            self.in_range()
        else:
            self.not_in_range()

    def execute(self) -> None:
        if not DriverStation.isTestEnabled():
            pass

        self._candle.set_control(self._pattern)
