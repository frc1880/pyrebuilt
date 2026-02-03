import phoenix6
from magicbot import tunable

import ids


class Shooter:
    shooter_speed = tunable(0.25)

    def __init__(self) -> None:
        self._should_shooter = False
        self._shooter_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_MOTOR, ids.CanbusId.SHOOTER
        )

    def shoot(self) -> None:
        self._should_shooter = True

    def execute(self) -> None:
        if self._should_shooter:
            # spin shooter motor
            self._shooter_motor.set(self.shooter_speed)
        else:
            self._shooter_motor.stopMotor()
        self._should_shooter = False
