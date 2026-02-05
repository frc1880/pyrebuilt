import phoenix6
from magicbot import tunable
from phoenix6 import configs, controls

import ids


class Shooter:
    shooter_speed = tunable(10)

    def __init__(self) -> None:
        self._should_shoot = False
        self._shooter_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_MOTOR, ids.CanbusId.SHOOTER
        )

        flywheel_gains_cfg = (
            configs.Slot0Configs()
            .with_k_p(0.20324)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.10208)
            .with_k_v(0.11809)
            .with_k_a(0.030786)
        )

        self._shooter_motor.configurator.apply(
            configs.TalonFXConfiguration().with_slot0(flywheel_gains_cfg)
        )

    def shoot(self) -> None:
        self._should_shoot = True

    def execute(self) -> None:
        if self._should_shoot:
            # spin shooter motor
            self._shooter_motor.set_control(
                controls.VelocityVoltage(self.shooter_speed)
            )
        else:
            self._shooter_motor.stopMotor()
        self._should_shoot = False
