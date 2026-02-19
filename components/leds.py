from phoenix6.controls.fire_animation import FireAnimation
from phoenix6.controls.rainbow_animation import RainbowAnimation
from phoenix6.controls.solid_color import SolidColor
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor

from ids import CandleId
from utilities.game import is_hub_active


class LEDMode:
    INTAKE = "intake_rainbow"
    IN_RANGE = "green"
    NOT_IN_RANGE = "red"
    SHOOTING = "fire"
    CLIMBING = "yellow"
    DEFAULT = "hub_not_active"


class LEDConfigs:
    led_start = 0
    led_end = 7  # change acc to add strip

    brightness = 1.0
    speed = 0.6


class Leds:
    """
    Light patterns for driver feedback.
    """

    # Colours
    RED = RGBWColor(255, 0, 0, 0)
    GREEN = RGBWColor(0, 255, 0, 0)
    YELLOW = RGBWColor(255, 255, 0, 0)
    WHITE = RGBWColor(0, 0, 0, 255)

    def __init__(self, config: LEDConfigs):
        self.candle = CANdle(device_id=CandleId.LED)

        self.config = LEDConfigs()

        self.range_status = False

        self.desired_mode: LEDMode = LEDMode.DEFAULT
        self.prev_mode: LEDMode | None = None

        self.curr_mode = self.change_mode(self.desired_mode)

    def set_intake(self) -> None:
        self.desired_mode(LEDMode.INTAKE)

    def set_shoot(self) -> None:
        self.desired_mode(LEDMode.SHOOTING)

    def set_climb(self) -> None:
        self.desired_mode(LEDMode.CLIMBING)

    # def range_check(self, in_range: bool) -> None:
    #    self.range_status = in_range

    def set_in_range(self) -> None:
        self.desired_mode(LEDMode.IN_RANGE)

    def set_not_in_range(self) -> None:
        self.desired_mode(LEDMode.NOT_IN_RANGE)

    def default(self) -> None:
        self.desired_mode(LEDMode.DEFAULT)

    def change_mode(self, mode: LEDMode):
        s = self.config.led_start
        e = self.config.led_end
        b = self.config.brightness
        sp = self.config.speed

        if mode == LEDMode.INTAKE:
            return RainbowAnimation(
                led_start_index=s,
                led_end_index=e,
                brightness=b,
                speed=sp,
            )

        if mode == LEDMode.SHOOTING:
            return FireAnimation(
                led_start_index=s,
                led_end_index=e,
                brightness=b,
                speed=sp,
            )

        if mode == LEDMode.CLIMBING:
            return SolidColor(
                led_start_index=s,
                led_end_index=e,
                brightness=b,
                speed=sp,
                color=self.YELLOW,
            )

        if not is_hub_active():
            return SolidColor(
                led_start_index=s,
                led_end_index=e,
                brightness=b,
                speed=sp,
                color=self.WHITE,
            )

        return SolidColor(
            led_start_index=s,
            led_end_index=e,
            brightness=b,
            speed=sp,
            color=Leds.GREEN if self.range_status else Leds.RED,
        )

    def execute(self) -> None:
        hub_active = is_hub_active()
        set_mode = (self.desired_mode, hub_active, self.range_status)

        if set_mode != self.prev_mode:
            self.curr_mode = self.change_mode(self.desired_mode)
            self.candle.set_control(self.curr_mode)
            self.prev_mode = set_mode
