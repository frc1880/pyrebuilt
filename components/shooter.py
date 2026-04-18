import numpy
import phoenix6
from magicbot import tunable, will_reset_to
from phoenix6 import configs, controls, signals
from wpilib import DriverStation

import ids
from components.ballistics import Ballistics
from components.drivetrain import Drivetrain
from utilities.positions import is_in_alliance_zone


class Shooter:
    ballistics: Ballistics
    drivetrain: Drivetrain

    speed = tunable(25.0)
    desired_hood_angle = tunable(60.0)
    _should_shoot = will_reset_to(False)

    # TODO check these values
    HOOD_MIN_ANGLE = 48.0  # degrees from horizontal
    HOOD_MAX_ANGLE = 70.0

    def __init__(self) -> None:
        self._shooter_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_FLYWHEEL_MOTOR, ids.CanbusId.SHOOTER
        )

        self._shooter_follower_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_FOLLOWER_FLYWHEEL_MOTOR, ids.CanbusId.SHOOTER
        )

        self._hood_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_HOOD_MOTOR, ids.CanbusId.SHOOTER
        )

        # Hood
        talon_fx_configs = configs.TalonFXConfiguration()
        hood_pid_cfg = talon_fx_configs.slot0
        # TODO tune these values
        hood_pid_cfg.k_p = 0.5  # Voltage per 1 deg error
        hood_pid_cfg.k_i = 0.005
        hood_pid_cfg.k_d = 0.01

        current_cfg = talon_fx_configs.current_limits
        current_cfg.stator_current_limit = 40.0
        current_cfg.stator_current_limit_enable = True
        current_cfg.supply_current_limit = 20.0
        current_cfg.supply_current_limit_enable = True
        current_cfg.supply_current_lower_limit = 5.0
        current_cfg.supply_current_lower_time = 1.0

        feedback_cfg = talon_fx_configs.feedback
        feedback_cfg.feedback_remote_sensor_id = ids.CancoderId.HOOD
        feedback_cfg.feedback_sensor_source = (
            signals.FeedbackSensorSourceValue.FUSED_CANCODER
        )
        # The hood is driven by a 24T:12T belt and pulleys, which then drives a 205T sector gear with a 24T spur gear
        feedback_cfg.sensor_to_mechanism_ratio = (
            205.0 / 24.0
        ) / 360.0  # scale to degrees
        feedback_cfg.rotor_to_sensor_ratio = 24.0 / 12.0

        self._hood_motor.configurator.apply(
            talon_fx_configs.with_current_limits(current_cfg)
        )

        self._cancoder = phoenix6.hardware.CANcoder(
            ids.CancoderId.HOOD, ids.CanbusId.SHOOTER
        )
        cc_cfg = configs.CANcoderConfiguration()
        cc_cfg.magnet_sensor.sensor_direction = (
            signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        cc_cfg.magnet_sensor.magnet_offset = -0.313721
        self._cancoder.configurator.apply(cc_cfg)

        # Example of closed loop mode once we have run sysid
        flywheel_gains_cfg = (
            configs.Slot0Configs()
            .with_k_p(0.25)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.29794)
            .with_k_v(0.12638)
            .with_k_a(0.032839)
        )
        current_cfg = (
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(80.0)
            .with_stator_current_limit_enable(True)
        )
        # TODO Invert shooter motor if required
        reverse_cfg = configs.MotorOutputConfigs()
        reverse_cfg.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        reverse_cfg.neutral_mode = signals.NeutralModeValue.COAST
        self._shooter_motor.configurator.apply(
            configs.TalonFXConfiguration()
            .with_slot0(flywheel_gains_cfg)
            .with_current_limits(current_cfg)
            .with_motor_output(reverse_cfg)
        )
        self._shooter_follower_motor.configurator.apply(
            configs.TalonFXConfiguration().with_current_limits(current_cfg)
        )
        self._initialized = False

    def shoot(self) -> None:
        self._should_shoot = True

    # @feedback
    def hood_angle(self) -> float:
        return self._hood_motor.get_position().value + 70.0

    def hood_cancoder_position(self) -> float:
        return self._cancoder.get_position().value

    # @feedback
    def hood_cancoder_absolute_position(self) -> float:
        return self._cancoder.get_absolute_position().value

    # @feedback
    def setpoint(self) -> float:
        return self.desired_hood_angle

    # @feedback
    def current_speed(self) -> float:
        return self._shooter_motor.get_velocity().value

    def shooter_motor_current(self) -> float:
        return self._shooter_motor.get_supply_current().value

    def at_speed(self) -> bool:
        return abs(self._shooter_motor.get_velocity().value - self.speed) < 5

    def execute(self) -> None:
        # Always run except in test mode
        if DriverStation.isTestEnabled():
            # Get values from dashboard
            desired_hood_angle = self.desired_hood_angle
            desired_speed = self.speed
            should_spin = self._should_shoot
        else:
            # Teleop or auto, so always set to what ballistics says
            in_alliance = is_in_alliance_zone(self.drivetrain.pose())
            solution = self.ballistics.solution()
            # Use the same flat shot for ferrying
            desired_hood_angle = (
                solution.hood_angle
                if in_alliance
                else 40.0
                if self._should_shoot
                else self.HOOD_MAX_ANGLE - 1.0
            )
            desired_speed = solution.flywheel_speed if in_alliance else 95.0
            self.speed = desired_speed
            self.desired_hood_angle = desired_hood_angle
            should_spin = self._should_shoot or in_alliance

        # Update hood setpoint even if not shooting
        mechanism_hood_angle = (
            numpy.clip(desired_hood_angle, self.HOOD_MIN_ANGLE, self.HOOD_MAX_ANGLE)
            - 70.0
        )
        self._hood_motor.set_control(controls.PositionVoltage(mechanism_hood_angle))

        if should_spin:
            # spin shooter motor
            # See https://www.chiefdelphi.com/t/kraken-x60-limp-mode-behavior/515080/44
            # for teams that have problems with Krakens in follower mode with FOC on
            # For now, we run without FOC enabled
            self._shooter_motor.set_control(
                controls.VelocityVoltage(desired_speed, enable_foc=False)
            )
        else:
            self._shooter_motor.stopMotor()

        # The motors are facing one another so set "opposed" mode
        # This will automatically invert the motor if required, even if the main shooter motor is reverse above
        self._shooter_follower_motor.set_control(
            controls.Follower(
                ids.TalonId.SHOOTER_FLYWHEEL_MOTOR, signals.MotorAlignmentValue.OPPOSED
            )
        )
