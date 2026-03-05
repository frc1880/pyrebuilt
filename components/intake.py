import phoenix6
from magicbot import tunable

import ids


class Intake:
    intake_speed = tunable(0.5)

    def __init__(self) -> None:
        self._should_intake = False
        self._intake_roller_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_ROLLER_MOTOR, ids.CanbusId.INTAKE
        )
        self._intake_deploy_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_DEPLOY_MOTOR, ids.CanbusId.INTAKE
        )

    def intake(self) -> None:
        self._should_intake = True

    def execute(self) -> None:
        if self._should_intake:
            # Spin the intake motor
            self._intake_roller_motor.set(self.intake_speed)
        else:
            self._intake_roller_motor.stopMotor()
        self._should_intake = False
