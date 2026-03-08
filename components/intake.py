import phoenix6
from magicbot import tunable
from phoenix6 import configs, controls, signals

import ids


class Intake:
    intake_speed = tunable(0.5)

    deployed_position = tunable(0.0)

    def __init__(self) -> None:
        self._should_intake = False
        self._roller_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_ROLLER_MOTOR, ids.CanbusId.INTAKE
        )
        self._deploy_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_DEPLOY_MOTOR, ids.CanbusId.INTAKE
        )

        deploy_pid_cfg = configs.Slot0Configs()
        # TODO tune these values
        deploy_pid_cfg.k_p = 1.0  # 1 rev error will output 1V
        deploy_pid_cfg.k_i = 0.0
        deploy_pid_cfg.k_d = 0.0
        self._deploy_motor.configurator.apply(
            configs.TalonFXConfiguration().with_slot0(deploy_pid_cfg)
        )

        # Variables used for zeroing against hard stop
        self._desired_intake_position = 0.0
        self._prev_intake_angle = 0.0
        self._initialized = False

        reverse_cfg = configs.MotorOutputConfigs()
        reverse_cfg.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        reverse_cfg.neutral_mode = signals.NeutralModeValue.COAST
        self._roller_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(reverse_cfg)
        )

    def intake(self) -> None:
        self._should_intake = True

    def deploy(self) -> None:
        self._desired_intake_position = self.deployed_position

    def retract(self) -> None:
        self._desired_intake_position = 0.0

    def execute(self) -> None:
        self._initialized = True  # TODO Remove this when we have a retract motor
        if not self._initialized:
            # Drive the motor very slowly towards the hard stop
            # Check to see if we are still moving/current spike
            # If we are stopped, reset the encoder value and put the motor in closed loop mode
            # TODO Is this output too small?
            self._deploy_motor.set_control(controls.DutyCycleOut(0.05))

            current = self._deploy_motor.get_stator_current().value
            angle = self._deploy_motor.get_position().value

            # TODO Check 1 degree threshold is okay
            # TODO Maybe use current as well - we expect a spike when stalled, but it will also spike on start
            # TODO Check 2A current threshold is okay
            if abs(angle - self._prev_intake_angle) < 0.5 and current > 4.0:
                self._deploy_motor.set_position(0.0)
                self._desired_intake_position = 0.0
                self._initialized = True
            else:
                self._prev_intake_angle = angle
                return  # we can't intake until we are ready

        self._deploy_motor.set_control(
            controls.PositionVoltage(self._desired_intake_position)
        )

        if self._should_intake:
            # Spin the intake motor
            self._roller_motor.set(self.intake_speed)
        else:
            self._roller_motor.stopMotor()
        self._should_intake = False
