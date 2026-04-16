import phoenix6
from magicbot import feedback, tunable, will_reset_to
from phoenix6 import configs, controls, signals
from wpilib import Timer

import ids


class Intake:
    intake_speed = tunable(0.8)

    # All positions are in mechanism rotations
    deployed_position = 0.0
    carry_position = tunable(0.20)
    retracted_position = 0.3
    timeSinceDeployed = 0.0
    deployed = False
    _should_spin = will_reset_to(False)
    _should_backdrive = will_reset_to(False)
    _should_feed = will_reset_to(False)

    def __init__(self) -> None:
        self._roller_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_ROLLER_MOTOR, ids.CanbusId.INTAKE
        )
        self._roller_follower_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.INTAKE_ROLLER_FOLLOWER_MOTOR, ids.CanbusId.INTAKE
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
        slot0_configs.k_g = 0.2
        slot0_configs.k_v = (
            5.0  # A velocity target of 1 rps results in this voltage output
        )
        slot0_configs.k_a = (
            0.0  # An acceleration of 1 rps/s requires this voltage output
        )
        slot0_configs.k_p = 30.0  # 1 rev error will output this voltage
        slot0_configs.k_i = 0.0  # Integrated error
        slot0_configs.k_d = (
            0.0  # A velocity error of 1 rps results in this voltage output
        )
        slot0_configs.gravity_type = signals.GravityTypeValue.ARM_COSINE
        slot0_configs.gravity_arm_position_offset = 0.0

        motion_magic_configs = talon_fx_configs.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = 0.0
        motion_magic_configs.motion_magic_expo_k_a = 5.0
        motion_magic_configs.motion_magic_expo_k_v = 1.0

        talon_fx_configs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        # Chain sprockets are 24:12 after a (3*9):1 maxplanetary gearbox reduction
        talon_fx_configs.feedback.rotor_to_sensor_ratio = 27.0 / 2.0
        talon_fx_configs.feedback.sensor_to_mechanism_ratio = 1.0
        # talon_fx_configs.feedback.sensor_to_mechanism_ratio = 2.0 * 27.0
        talon_fx_configs.feedback.feedback_sensor_source = (
            signals.FeedbackSensorSourceValue.REMOTE_CANCODER
        )
        talon_fx_configs.feedback.feedback_remote_sensor_id = ids.CancoderId.INTAKE
        talon_fx_configs.software_limit_switch.forward_soft_limit_threshold = 0.34
        talon_fx_configs.software_limit_switch.forward_soft_limit_enable = True
        talon_fx_configs.software_limit_switch.reverse_soft_limit_threshold = 0.0
        talon_fx_configs.software_limit_switch.reverse_soft_limit_enable = True
        self._deploy_motor.configurator.apply(talon_fx_configs)

        cc_cfg = configs.CANcoderConfiguration()
        cc_cfg.magnet_sensor.sensor_direction = (
            signals.SensorDirectionValue.CLOCKWISE_POSITIVE
        )
        cc_cfg.magnet_sensor.magnet_offset = -0.441
        self._cancoder = phoenix6.hardware.CANcoder(
            ids.CancoderId.INTAKE, ids.CanbusId.INTAKE
        )
        self._cancoder.configurator.apply(cc_cfg)
        # initial_position = self._cancoder.get_position().value
        # if initial_position < -0.1:
        #    self._cancoder.set_position(initial_position + 1.0)

        reverse_cfg = configs.MotorOutputConfigs()
        reverse_cfg.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        reverse_cfg.neutral_mode = signals.NeutralModeValue.COAST
        self._roller_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(reverse_cfg)
        )
        self._timer = Timer()
        self._timer.start()

    def setup(self) -> None:
        self._desired_intake_position = self.carry_position
        # self._deploy_motor.set_position(0.33)

    @feedback
    def position(self) -> float:
        return self._deploy_motor.get_position().value

    @feedback
    def cancoder_position(self) -> float:
        return self._cancoder.get_absolute_position().value

    @feedback
    def setpoint(self) -> float:
        return self._desired_intake_position

    def intake(self) -> None:
        self.deployed = True
        self._desired_intake_position = self.deployed_position
        self._should_spin = True

    def carry(self) -> None:
        self._desired_intake_position = self.carry_position
        self.deployed = False

    def retract(self) -> None:
        self._desired_intake_position = self.retracted_position
        self.deployed = False

    def backdrive(self) -> None:
        self._should_backdrive = True

    def spin(self) -> None:
        self._should_spin = True

    def feed(self) -> None:
        self._desired_intake_position = self.carry_position
        self._should_feed = True

    def execute(self) -> None:
        self._deploy_motor.set_control(
            controls.MotionMagicExpoVoltage(self._desired_intake_position)
        )
        self._deploy_follower_motor.set_control(
            controls.Follower(
                ids.TalonId.INTAKE_DEPLOY_MOTOR, signals.MotorAlignmentValue.OPPOSED
            )
        )
        self.timeSinceDeployed = self._timer.get()
        if not self.deployed:
            self._timer.reset()

        if self._should_spin:
            # Spin the intake motor
            self._roller_motor.set(self.intake_speed)
        elif self._should_feed:
            self._roller_motor.set(0.5)
        elif self._should_backdrive:
            self._roller_motor.set(-1.0)
        else:
            self._roller_motor.stopMotor()

        self._roller_follower_motor.set_control(
            controls.Follower(
                ids.TalonId.INTAKE_ROLLER_MOTOR, signals.MotorAlignmentValue.OPPOSED
            )
        )
