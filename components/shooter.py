import phoenix6
from magicbot import tunable

import ids


class Shooter:
    shooter_speed = tunable(0.25)

    def __init__(self) -> None:
        self._should_shoot = False
        self._shooter_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_MOTOR, ids.CanbusId.SHOOTER
        )

    def shoot(self) -> None:
        self._should_shoot = True

    def execute(self) -> None:
        if self._should_shoot:
            # spin shooter motor
            self._shooter_motor.set(self.shooter_speed)
        else:
            self._shooter_motor.stopMotor()
        self._should_shoot = False
