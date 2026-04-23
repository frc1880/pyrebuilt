import phoenix6
from magicbot import feedback, tunable, will_reset_to
from phoenix6 import configs, signals
from wpilib import Timer

import ids


class Indexer:
    indexer_speed_rotation = tunable(0.5)
    backdrive_speed = tunable(-0.5)
    _should_feed = will_reset_to(False)
    _should_backdrive = will_reset_to(False)

    def __init__(self) -> None:
        self._indexer_rotation_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INDEXER_ROTATION_MOTOR, ids.CanbusId.INDEXER
        )
        invert_cfg = configs.MotorOutputConfigs().with_inverted(
            signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        current_cfg = configs.CurrentLimitsConfigs()
        current_cfg.supply_current_limit = 20.0
        current_cfg.supply_current_limit_enable = True
        current_cfg.supply_current_lower_limit = 5.0
        current_cfg.supply_current_lower_time = 1.0

        self._indexer_rotation_motor.configurator.apply(
            configs.TalonFXConfiguration()
            .with_motor_output(invert_cfg)
            .with_current_limits(current_cfg)
        )
        self._injector_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INDEXER_INJECTOR_MOTOR, ids.CanbusId.INDEXER
        )

        self._timer = Timer()
        self._timer.start()

    def feed(self) -> None:
        self._should_feed = True

    def backdrive(self) -> None:
        self._should_backdrive = True

    @feedback
    def is_hopper_empty(self) -> bool:
        return self._timer.hasElapsed(0.5)

    def execute(self) -> None:
        if (
            self._injector_motor.get_supply_current().value > 20.0
            or not self._should_feed
        ):
            self._timer.reset()

        if self._should_feed:
            # Spin the index motor
            self._indexer_rotation_motor.set(self.indexer_speed_rotation)
            self._injector_motor.set(1.0)
        elif self._should_backdrive:
            self._injector_motor.set(-1.0)
            self._indexer_rotation_motor.set(-abs(self.backdrive_speed))
        else:
            self._indexer_rotation_motor.stopMotor()
            self._injector_motor.stopMotor()
