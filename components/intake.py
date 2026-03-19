import math

import phoenix6
from magicbot import feedback, tunable, will_reset_to
from phoenix6 import configs, controls, signals

import ids


class Intake:
    intake_speed = tunable(1.0)

    deployed_position = tunable(-21.0)
    carry_position = tunable(-7.0)
    _should_spin = will_reset_to(False)
    _should_deploy = will_reset_to(False)

    def __init__(self) -> None:
        self._roller_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_ROLLER_MOTOR, ids.CanbusId.INTAKE
        )
        self._deploy_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_DEPLOY_MOTOR, ids.CanbusId.INTAKE
        )

        talon_fx_configs = configs.TalonFXConfiguration()

        slot0_configs = talon_fx_configs.slot0
        # TODO tune these values
        slot0_configs.k_s = (
            0.25 * 0.0
        )  # Add this voltage output to overcome static friction
        slot0_configs.k_v = (
            6.72 * 0.15  # A velocity target of 1 rps results in this voltage output
        )
        slot0_configs.k_a = (
            0.04  # An acceleration of 1 rps/s requires this voltage output
        )
        slot0_configs.k_p = 15.0  # 1 rev error will output this voltage
        slot0_configs.k_i = 0.0  # Integrated error
        slot0_configs.k_d = (
            0.12 * 0.5  # A velocity error of 1 rps results in this voltage output
        )

        motion_magic_configs = talon_fx_configs.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = 1000.0
        motion_magic_configs.motion_magic_expo_k_a = 0.1 * 1
        motion_magic_configs.motion_magic_expo_k_v = 0.12 * 0.5

        self._deploy_motor.configurator.apply(talon_fx_configs)

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

    @feedback
    def initialized(self) -> bool:
        return self._initialized

    @feedback
    def position(self) -> float:
        return self._deploy_motor.get_position().value

    @feedback
    def setpoint(self) -> float:
        return self._desired_intake_position

    def intake(self) -> None:
        self._should_deploy = True
        self._should_spin = True

    def execute(self) -> None:
        if not self._initialized:
            # Drive the motor very slowly towards the hard stop
            # Check to see if we are still moving/current spike
            # If we are stopped, reset the encoder value and put the motor in closed loop mode
            # TODO Is this output too small?
            self._deploy_motor.set_control(controls.DutyCycleOut(0.15))

            current = self._deploy_motor.get_stator_current().value
            angle = self._deploy_motor.get_position().value

            # TODO Check 1 degree threshold is okay
            # TODO Maybe use current as well - we expect a spike when stalled, but it will also spike on start
            # TODO Check 4A current threshold is okay
            if abs(angle - self._prev_intake_angle) < 0.05 and current > 10.0:
                self._deploy_motor.stopMotor()
                self._deploy_motor.set_position(0.0)
                self._desired_intake_position = 0.0
                self._initialized = True
                return
            else:
                self._prev_intake_angle = angle
                return  # we can't intake until we are ready

        # while intake go to set position
        if self._should_deploy:
            self._desired_intake_position = self.deployed_position
        else:
            self._desired_intake_position = self.carry_position

        # Only use the motion profile if we are away from the setpoint
        if (
            True
            and abs(
                self._desired_intake_position - self._deploy_motor.get_position().value
            )
            > 0.25
        ):
            # Compensate for gravity
            ff_volts = -0.5 * math.sin(
                self.position() / self.deployed_position * math.pi / 2.0
            )
            self._deploy_motor.set_control(
                controls.MotionMagicExpoVoltage(
                    self._desired_intake_position, feed_forward=ff_volts
                )
            )
        else:
            self._deploy_motor.stopMotor()

        if self._should_spin:
            # Spin the intake motor
            self._roller_motor.set(self.intake_speed)
        else:
            self._roller_motor.stopMotor()
