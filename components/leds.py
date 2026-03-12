from phoenix6.controls.solid_color import SolidColor
from phoenix6.controls.strobe_animation import StrobeAnimation
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor
from wpilib import DriverStation

from components.ballistics import Ballistics
from ids import CanbusId, CandleId
from utilities.game import is_hub_active, time_to_hub_active


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
        self._pattern: SolidColor | StrobeAnimation = self._solid(self.WHITE)

    def _solid(self, color: RGBWColor) -> SolidColor:
        return SolidColor(self.led_start, self.led_end, color)

    def _flashing(self, color: RGBWColor) -> StrobeAnimation:
        return StrobeAnimation(self.led_start, self.led_end, 0, color)

    def intake(self) -> None:
        self._pattern = self._solid(self.BLUE)

    def in_range(self, should_flash: bool = False) -> None:
        self._pattern = (
            self._flashing(self.GREEN) if should_flash else self._solid(self.GREEN)
        )

    def not_in_range(self, should_flash: bool = False) -> None:
        self._pattern = (
            self._flashing(self.RED) if should_flash else self._solid(self.RED)
        )

    def execute(self) -> None:
        if not DriverStation.isTestEnabled():
            should_flash = time_to_hub_active() < 5.0 and not is_hub_active()

            if not is_hub_active() or should_flash:
                if self.ballistics.is_within_range():
                    self.in_range(should_flash)
                else:
                    self.not_in_range(should_flash)

        self._candle.set_control(self._pattern)
