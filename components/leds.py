from ballistics import Ballistics
from phoenix6.controls.fire_animation import FireAnimation
from phoenix6.controls.rainbow_animation import RainbowAnimation
from phoenix6.controls.solid_color import SolidColor
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor

from ids import CanbusId, CandleId
from utilities.game import is_hub_active


class LEDConfigs:
    led_start = 0
    led_end = 7  # change according to your strip length

    brightness = 1.0
    speed = 0.6


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
    WHITE = RGBWColor(0, 0, 0, 255)

    def __init__(self, config: LEDConfigs | None = None):
        self._candle = CANdle(device_id=CandleId.LED, canbus=CanbusId.LEDS)

        self.config = config or LEDConfigs()

        # Default pattern
        self._pattern = SolidColor(self.WHITE, self.config)

    def intake(self) -> None:
        self._pattern = RainbowAnimation(self.config)

    def shoot(self) -> None:
        self._pattern = FireAnimation(self.config)

    def climb(self) -> None:
        self._pattern = SolidColor(self.YELLOW, self.config)

    def in_range(self) -> None:
        self._pattern = SolidColor(self.GREEN, self.config)

    def not_in_range(self) -> None:
        self._pattern = SolidColor(self.RED, self.config)

    def default(self) -> None:
        self._pattern = SolidColor(self.WHITE, self.config)

    def teleop_lights(self) -> None:
        if not is_hub_active():
            self.default()
            return

        if self.ballistics.in_range():
            self.in_range()
        else:
            self.not_in_range()

    def execute(self) -> None:
        # Apply the currently selected pattern to the CANdle
        self._candle.set_control(self._pattern)
