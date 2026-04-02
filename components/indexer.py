import phoenix6
from magicbot import feedback, tunable, will_reset_to
from phoenix6 import CANBus, configs, hardware, signals
from wpilib import Timer

import ids


class Indexer:
    indexer_speed_rotation = tunable(1.0)
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
        self._indexer_rotation_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(invert_cfg)
        )
        self._injector_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INDEXER_INJECTOR_MOTOR, ids.CanbusId.INDEXER
        )
        self.canrange = hardware.CANrange(28, CANBus("rio"))

        cfg = configs.CANrangeConfiguration()
        cfg.proximity_params.min_signal_strength_for_valid_measurement = 2500
        cfg.proximity_params.proximity_threshold = 0.55
        cfg.to_f_params.update_mode = signals.UpdateModeValue.SHORT_RANGE100_HZ

        self.canrange.configurator.apply(cfg)

        self._timer = Timer()
        self._timer.start()
        self._indexer_empty = True

    def feed(self) -> None:
        self._should_feed = True

    def backdrive(self) -> None:
        self._should_backdrive = True

    @feedback
    def is_indexer_empty(self) -> bool:
        return self._indexer_empty

    def execute(self) -> None:

        detected = self.canrange.get_is_detected().value
        dist = self.canrange.get_distance().value

        if detected and dist >= 0.05:
            self._timer.reset()
            self._indexer_empty = False
        elif self._timer.hasElapsed(0.5):
            self._indexer_empty = True

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
