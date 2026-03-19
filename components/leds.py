from enum import Enum, auto

from magicbot import feedback
from phoenix6.controls.color_flow_animation import ColorFlowAnimation
from phoenix6.controls.single_fade_animation import SingleFadeAnimation
from phoenix6.controls.solid_color import SolidColor
from phoenix6.controls.strobe_animation import StrobeAnimation
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor
from wpilib import DriverStation
from wpimath.geometry import Transform2d

from components.ballistics import Ballistics
from ids import CanbusId, CandleId
from utilities.game import is_hub_active, time_to_hub_active


class Pattern(Enum):
    NO_VISION = auto()
    NO_AUTO = auto()
    ROTATE_CW = auto()
    ROTATE_CCW = auto()
    MOVE_FORWARD = auto()
    MOVE_BACKWARD = auto()
    MOVE_LEFT = auto()
    MOVE_RIGHT = auto()
    DISABLED = auto()
    IN_RANGE = auto()
    IN_RANGE_FLASH = auto()
    NOT_IN_RANGE = auto()
    NOT_IN_RANGE_FLASH = auto()
    OFF = auto()


class Leds:
    """
    Light patterns for driver feedback.
    """

    # Injected by MagicBot
    ballistics: Ballistics

    # Colours
    RED = RGBWColor(255, 0, 0)
    GREEN = RGBWColor(0, 255, 0)
    YELLOW = RGBWColor(255, 255, 0)
    BLUE = RGBWColor(0, 0, 255)
    WHITE = RGBWColor(255, 255, 255)
    ORANGE = RGBWColor(255, 165, 0)

    led_start: int = 0
    led_end: int = 37
    segments = [0, 7, 14, 21, 29, 35]
    brightness: float = 1.0

    def __init__(self) -> None:
        self._candle = CANdle(device_id=CandleId.LED, canbus=CanbusId.LEDS)

        # Default pattern
        self._pattern = Pattern.DISABLED
        self._previous_pattern = Pattern.OFF

    def in_range(self, should_flash: bool = False) -> None:
        self._pattern = Pattern.IN_RANGE_FLASH if should_flash else Pattern.IN_RANGE

    def not_in_range(self, should_flash: bool = False) -> None:
        self._pattern = (
            Pattern.NOT_IN_RANGE_FLASH if should_flash else Pattern.NOT_IN_RANGE
        )

    def missing_vision(self) -> None:
        self._pattern = Pattern.NO_VISION

    def missing_auto(self) -> None:
        self._pattern = Pattern.NO_AUTO

    def wrong_start(self, error: Transform2d):
        # Light up various parts of the robot to show the direction to move it
        if error.rotation().radians() > 0:
            # Negative z rotation required
            self._pattern = Pattern.ROTATE_CW
        elif error.rotation().radians() < 0:
            # Positive z rotation required
            self._pattern = Pattern.ROTATE_CCW
        else:
            # Rotation is okay, so give feedback on position
            if error.translation().x > 0:
                # Move back required
                self._pattern = Pattern.MOVE_BACKWARD
            elif error.translation().x < 0:
                # Move forward required
                self._pattern = Pattern.MOVE_FORWARD
            if error.translation().y > 0:
                # Move right required
                self._pattern = Pattern.MOVE_RIGHT
            elif error.translation().y < 0:
                # Move left required
                self._pattern = Pattern.MOVE_LEFT

    def disabled(self) -> None:
        self._pattern = Pattern.DISABLED

    @feedback
    def pattern(self) -> str:
        return self._pattern.name

    def execute(self) -> None:
        if not (DriverStation.isTestEnabled() or DriverStation.isDisabled()):
            should_flash = time_to_hub_active() < 5.0 and not is_hub_active()

            if is_hub_active() or should_flash:
                if self.ballistics.is_within_range():
                    self._pattern = (
                        Pattern.IN_RANGE_FLASH if should_flash else Pattern.IN_RANGE
                    )
                else:
                    self._pattern = (
                        Pattern.NOT_IN_RANGE_FLASH
                        if should_flash
                        else Pattern.NOT_IN_RANGE
                    )
            else:
                self._pattern = Pattern.OFF

        if self._pattern != self._previous_pattern:
            # Don't keep overwriting pattern, so only update if changed
            self._candle.clear_all_animations()
            match self._pattern:
                case Pattern.NO_VISION:
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[0], self.segments[-1], color=self.BLUE
                        )
                    )
                case Pattern.NO_AUTO:
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[0], self.segments[-1], color=self.ORANGE
                        )
                    )
                case Pattern.ROTATE_CW:
                    self._candle.set_control(
                        ColorFlowAnimation(
                            self.segments[1], self.segments[-1], color=self.YELLOW
                        )
                    )
                case Pattern.ROTATE_CCW:
                    self._candle.set_control(
                        ColorFlowAnimation(
                            self.segments[-1], self.segments[1], color=self.YELLOW
                        )
                    )
                case Pattern.MOVE_FORWARD:
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[1], self.segments[3], color=self.YELLOW
                        )
                    )
                case Pattern.MOVE_BACKWARD:
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[3], self.segments[5], color=self.YELLOW
                        )
                    )
                case Pattern.MOVE_LEFT:
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[2], self.segments[4], color=self.YELLOW
                        )
                    )

                case Pattern.MOVE_RIGHT:
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[1], self.segments[2], color=self.YELLOW
                        )
                    )
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[3], self.segments[4], 1, color=self.YELLOW
                        )
                    )
                case Pattern.DISABLED:
                    for idx in range(4):
                        start = self.segments[idx + 1]
                        end = self.segments[idx + 2]
                        mid = int((start + end) / 2)
                        self._candle.set_control(
                            SingleFadeAnimation(start, mid, idx * 2, color=self.BLUE)
                        )
                        self._candle.set_control(
                            SingleFadeAnimation(mid, end, idx * 2 + 1, color=self.WHITE)
                        )
                case Pattern.IN_RANGE:
                    self._candle.set_control(
                        SolidColor(
                            self.segments[0], self.segments[-1], color=self.GREEN
                        )
                    )
                case Pattern.IN_RANGE_FLASH:
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[0], self.segments[-1], color=self.GREEN
                        )
                    )
                case Pattern.NOT_IN_RANGE:
                    self._candle.set_control(
                        SolidColor(self.segments[0], self.segments[-1], color=self.RED)
                    )
                case Pattern.NOT_IN_RANGE_FLASH:
                    self._candle.set_control(
                        StrobeAnimation(
                            self.segments[0], self.segments[-1], color=self.RED
                        )
                    )
                case Pattern.OFF:
                    pass

        self._previous_pattern = self._pattern
