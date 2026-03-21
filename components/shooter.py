import numpy
import phoenix6
from magicbot import feedback, tunable, will_reset_to
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
    desired_hood_angle = tunable(70.0)
    _should_shoot = will_reset_to(False)

    # TODO check these values
    HOOD_MIN_ANGLE = 36.0  # degrees from horizontal
    HOOD_MAX_ANGLE = 70.0

    # The hood is driven by a 42T:12T belt and pulleys, which then drives a 200T sector gear with a 20T spur gear
    GEAR_RATIO = 42.0 / 12.0 * 200.0 / 20.0

    def __init__(self) -> None:
        self._shooter_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_FLYWHEEL_MOTOR, ids.CanbusId.SHOOTER
        )

        # TODO Invert shooter motor if required
        reverse_cfg = configs.MotorOutputConfigs()
        reverse_cfg.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        reverse_cfg.neutral_mode = signals.NeutralModeValue.COAST
        self._shooter_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(reverse_cfg)
        )

        self._shooter_follower_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_FOLLOWER_FLYWHEEL_MOTOR, ids.CanbusId.SHOOTER
        )

        self._hood_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_HOOD_MOTOR, ids.CanbusId.SHOOTER
        )
        hood_pid_cfg = configs.Slot0Configs()
        # TODO tune these values
        hood_pid_cfg.k_p = 4.0  # 1 rev error will output 1V
        hood_pid_cfg.k_i = 0.0
        hood_pid_cfg.k_d = 0.0
        self._hood_motor.configurator.apply(
            configs.TalonFXConfiguration().with_slot0(hood_pid_cfg)
        )

        # Variables used for zeroing against hard stop
        self._initialized = False
        self._prev_hood_angle = 999.0

        # Example of closed loop mode once we have run sysid
        flywheel_gains_cfg = (
            configs.Slot0Configs()
            .with_k_p(0.6)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.22446)
            .with_k_v(0.1213)
            .with_k_a(0.024026)
        )
        current_cfg = (
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(30.0)
            .with_stator_current_limit_enable(True)
        )
        self._shooter_motor.configurator.apply(
            configs.TalonFXConfiguration()
            .with_slot0(flywheel_gains_cfg)
            .with_current_limits(current_cfg)
        )
        self._shooter_follower_motor.configurator.apply(
            configs.TalonFXConfiguration().with_current_limits(current_cfg)
        )

    def shoot(self) -> None:
        self._should_shoot = True

    @feedback
    def is_initialized(self) -> bool:
        return self._initialized

    @feedback
    def hood_angle(self) -> float:
        return self._hood_motor.get_position().value * 360.0 / self.GEAR_RATIO

    @feedback
    def hood_current(self) -> float:
        return self._hood_motor.get_stator_current().value

    @feedback
    def setpoint(self) -> float:
        return self.desired_hood_angle / 360.0 * self.GEAR_RATIO

    @feedback
    def current_speed(self) -> float:
        return self._shooter_motor.get_velocity().value

    @feedback
    def at_speed(self) -> bool:
        return abs(abs(self._shooter_motor.get_velocity().value) - abs(self.speed)) < 5

    def execute(self) -> None:
        if not self._initialized:
            # # Drive the motor very slowly towards the hard stop
            # # Check to see if we are still moving/current spike
            # # If we are stopped, reset the encoder value and put the motor in closed loop mode
            # # TODO Is this output too small?
            # self._hood_motor.set_control(controls.DutyCycleOut(0.1))

            # angle = self.hood_angle()
            # current = self.hood_current()

            # # TODO Check 1 degree threshold is okay
            # # TODO Maybe use current as well - we expect a spike when stalled, but it will also spike on start
            # # TODO Check 2A current threshold is okay
            # if abs(angle - self._prev_hood_angle) < 0.25 and current > 6.0:
            #     self._hood_motor.set_position(
            #         self.HOOD_MAX_ANGLE / 360.0 * self.GEAR_RATIO
            #     )  # max angle is the high, lob shot
            #     self._initialized = True
            # else:
            #     self._prev_hood_angle = angle
            #     return  # we can't shoot until we are ready
            self._hood_motor.set_position(self.HOOD_MAX_ANGLE / 360.0 * self.GEAR_RATIO)
            self._initialized = True
            return

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
            desired_speed = solution.flywheel_speed if in_alliance else 75.0
            self.speed = desired_speed
            self.desired_hood_angle = desired_hood_angle
            should_spin = in_alliance or self._should_shoot

        # Update hood setpoint even if not shooting
        desired_hood_rotation = (
            numpy.clip(desired_hood_angle, self.HOOD_MIN_ANGLE, self.HOOD_MAX_ANGLE)
            / 360.0
            * self.GEAR_RATIO
        )
        self._hood_motor.set_control(controls.PositionVoltage(desired_hood_rotation))

        if should_spin:
            # spin shooter motor
            # See https://www.chiefdelphi.com/t/kraken-x60-limp-mode-behavior/515080/44
            # for teams that have problems with Krakens in follower mode with FOC on
            # For now, we run without FOC enabled
            self._shooter_motor.set_control(
                controls.VelocityVoltage(-desired_speed, enable_foc=False)
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
