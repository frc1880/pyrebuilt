import phoenix6
from magicbot import tunable
from phoenix6 import configs, signals

import ids


class Indexer:
    indexer_speed_rotation = tunable(0.1)

    def __init__(self) -> None:
        self._should_feed = False
        self._indexer_rotation_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INDEXER_ROTATION_MOTOR, ids.CanbusId.INDEXER
        )
        invert_cfg = configs.MotorOutputConfigs().with_inverted(
            signals.InvertedValue.CLOCKWISE_POSITIVE
        )
        self._indexer_rotation_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(invert_cfg)
        )

    def feed(self) -> None:
        self._should_feed = True

    def execute(self) -> None:
        if self._should_feed:
            # Spin the index motor
            self._indexer_rotation_motor.set(self.indexer_speed_rotation)
        else:
            self._indexer_rotation_motor.stopMotor()
        self._should_feed = False
