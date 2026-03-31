import phoenix6
from magicbot import feedback, tunable, will_reset_to
from phoenix6 import configs, controls, signals

import ids


class Intake:
    intake_speed = tunable(1.0)

    # All positions are in mechanism rotations
    deployed_position = 0.0
    carry_position = tunable(0.25)
    start_position = tunable(0.3)
    _should_spin = will_reset_to(False)
    _should_feed = will_reset_to(False)
    _should_deploy = will_reset_to(False)

    def __init__(self) -> None:
        self._roller_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_ROLLER_MOTOR, ids.CanbusId.INTAKE
        )
        self._deploy_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_DEPLOY_MOTOR, ids.CanbusId.INTAKE
        )
        self._deploy_follower_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_DEPLOY_FOLLOWER_MOTOR, ids.CanbusId.INTAKE
        )

        talon_fx_configs = configs.TalonFXConfiguration()

        slot0_configs = talon_fx_configs.slot0
        # TODO tune these values
        slot0_configs.k_g = 0.0
        slot0_configs.k_v = (
            0.0  # A velocity target of 1 rps results in this voltage output
        )
        slot0_configs.k_a = (
            0.0  # An acceleration of 1 rps/s requires this voltage output
        )
        slot0_configs.k_p = 0.0  # 1 rev error will output this voltage
        slot0_configs.k_i = 0.0  # Integrated error
        slot0_configs.k_d = (
            0.0  # A velocity error of 1 rps results in this voltage output
        )
        slot0_configs.gravity_type = signals.GravityTypeValue.ARM_COSINE
        slot0_configs.gravity_arm_position_offset = 0.0

        motion_magic_configs = talon_fx_configs.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = 0.0
        motion_magic_configs.motion_magic_expo_k_a = 0.0
        motion_magic_configs.motion_magic_expo_k_v = 0.0

        talon_fx_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        # Chain sprockets are 24:12 after a 15:1 maxplanetary gearbox reduction
        talon_fx_configs.feedback.sensor_to_mechanism_ratio = 24.0 / 12.0 * 15.0 / 1.0

        self._deploy_motor.configurator.apply(talon_fx_configs)

        # Variables used for zeroing against hard stop
        self._desired_intake_position = 0.0
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

    def spin(self) -> None:
        self._should_spin = True

    def feed(self) -> None:
        self._should_feed = True

    def execute(self) -> None:
        if not self._initialized:
            self._deploy_motor.set_position(self.start_position)
            self._desired_intake_position = self.start_position
            self._initialized = True
            return

        # while intake go to set position
        if self._should_deploy:
            self._desired_intake_position = self.deployed_position
        else:
            self._desired_intake_position = self.carry_position

        self._deploy_motor.set_control(
            controls.MotionMagicExpoTorqueCurrentFOC(self._desired_intake_position)
        )
        self._deploy_follower_motor.set_control(
            controls.Follower(
                ids.TalonId.INTAKE_DEPLOY_MOTOR, signals.MotorAlignmentValue.OPPOSED
            )
        )

        if self._should_spin:
            # Spin the intake motor
            self._roller_motor.set(self.intake_speed)
        elif self._should_feed:
            self._roller_motor.set(0.3)
        else:
            self._roller_motor.stopMotor()
