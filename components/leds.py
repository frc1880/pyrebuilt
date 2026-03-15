from phoenix6.controls.empty_animation import EmptyAnimation
from phoenix6.controls.rainbow_animation import RainbowAnimation
from phoenix6.controls.solid_color import SolidColor
from phoenix6.controls.strobe_animation import StrobeAnimation
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor
from wpilib import DriverStation

from components.ballistics import Ballistics
from ids import CanbusId, CandleId
from utilities.game import is_blue, is_hub_active, is_red, time_to_hub_active


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
    PURPLE = RGBWColor(160, 0, 255, 0)
    ORANGE = RGBWColor(255, 165, 0, 0)
    OFF = RGBWColor(0, 0, 0, 0)

    led_start_intake: int = 0
    led_end_intake: int = 8
    led_start: int = 11
    led_end: int = 31
    brightness: float = 1.0

    def __init__(self) -> None:
        self._candle = CANdle(device_id=CandleId.LED, canbus=CanbusId.LEDS)

        self._intake_pattern: SolidColor | StrobeAnimation = self._solid(
            self.led_start_intake, self.led_end_intake, self.OFF
        )

        # Default pattern
        self._range_pattern: SolidColor | StrobeAnimation = self._solid(
            self.led_start, self.led_end, self.WHITE
        )

    def _solid(self, start: int, end: int, color: RGBWColor) -> SolidColor:
        return SolidColor(start, end, color)

    def _flashing(
        self, start: int, end: int, slot: int, color: RGBWColor
    ) -> StrobeAnimation:
        return StrobeAnimation(start, end, 1, color)

    def _rainbow(self, start: int, end: int, brightness: float) -> RainbowAnimation:
        return RainbowAnimation(start, end, 0, brightness)

    def intake(self, should_flash: bool = False) -> None:
        self._intake_pattern = (
            self._flashing(self.led_start_intake, self.led_end_intake, 0, self.BLUE)
            if should_flash
            else self._solid(self.led_start_intake, self.led_end_intake, self.BLUE)
        )

    def stop_intake(self) -> None:
        self._intake_pattern = self._solid(
            self.led_start_intake, self.led_end_intake, self.OFF
        )

    def in_range(self, should_flash: bool = False) -> None:
        self._range_pattern = (
            self._flashing(self.led_start, self.led_end, 0, self.GREEN)
            if should_flash
            else self._solid(self.led_start, self.led_end, self.GREEN)
        )

    def not_in_range(self, should_flash: bool = False) -> None:
        self._range_pattern = (
            self._flashing(self.led_start, self.led_end, 0, self.RED)
            if should_flash
            else self._solid(self.led_start, self.led_end, self.RED)
        )

    def won_auto(self) -> None:
        self._intake_pattern = self._flashing(
            self.led_start_intake, self.led_end_intake, 1, self.PURPLE
        )
        self._range_pattern = self._flashing(
            self.led_start, self.led_end, 1, self.PURPLE
        )

    def lost_auto(self) -> None:
        self._intake_pattern = self._flashing(
            self.led_start_intake, self.led_end_intake, 1, self.YELLOW
        )
        self._range_pattern = self._flashing(
            self.led_start, self.led_end, 1, self.YELLOW
        )

    def execute(self) -> None:
        if not DriverStation.isTestEnabled():
            should_flash = time_to_hub_active() < 5.0 and not is_hub_active()
            match_time = DriverStation.getMatchTime()

            if not is_hub_active() or should_flash:
                if self.ballistics.is_within_range():
                    self.in_range(should_flash)
                else:
                    self.not_in_range(should_flash)

            game_data = DriverStation.getGameSpecificMessage()

            if match_time > 128 and game_data:
                if (game_data == "R" and is_red()) or (game_data == "B" and is_blue()):
                    self.won_auto()
                else:
                    self.lost_auto()

        if isinstance(self._intake_pattern, SolidColor):
            self._candle.set_control(EmptyAnimation(0))
        self._candle.set_control(self._range_pattern)
        self._candle.set_control(self._intake_pattern)
