import numpy
import phoenix6
from magicbot import feedback, tunable, will_reset_to
from phoenix6 import configs, controls, signals

import ids


class Shooter:
    speed = tunable(0.25)
    desired_hood_angle = tunable(70)
    _should_shoot = will_reset_to(False)

    # TODO check these values
    HOOD_MIN_ANGLE = 45.0  # degrees from horizontal
    HOOD_MAX_ANGLE = 70.0

    # The hood is driven by a 40T:12T belt and pulleys, which then drives a 200T sector gear with a 20T spur gear
    GEAR_RATIO = 40.0 / 12.0 * 200.0 / 20.0

    def __init__(self) -> None:
        self._should_shoot = False

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
        # The motors are facing one another so set "opposed" mode
        # This will automatically invert the motor if required, even if the main shooter motor is reverse above
        self._shooter_follower_motor.set_control(
            controls.Follower(
                ids.TalonId.SHOOTER_FLYWHEEL_MOTOR, signals.MotorAlignmentValue.OPPOSED
            )
        )

        self._hood_motor = phoenix6.hardware.TalonFX(
            ids.TalonId.SHOOTER_HOOD_MOTOR, ids.CanbusId.SHOOTER
        )
        hood_pid_cfg = configs.Slot0Configs()
        # TODO tune these values
        hood_pid_cfg.k_p = 1.0  # 1 rev error will output 1V
        hood_pid_cfg.k_i = 0.0
        hood_pid_cfg.k_d = 0.0
        self._hood_motor.configurator.apply(
            configs.TalonFXConfiguration().with_slot0(hood_pid_cfg)
        )

        # Variables used for zeroing against hard stop
        self._initialized = False
        self._prev_hood_angle = 999.0

        # Example of closed loop mode once we have run sysid
        # flywheel_gains_cfg = (
        #     configs.Slot0Configs()
        #     .with_k_p(0.20324)
        #     .with_k_i(0)
        #     .with_k_d(0)
        #     .with_k_s(0.10208)
        #     .with_k_v(0.11809)
        #     .with_k_a(0.030786)
        # )

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

    def execute(self) -> None:
        if not self._initialized:
            # Drive the motor very slowly towards the hard stop
            # Check to see if we are still moving/current spike
            # If we are stopped, reset the encoder value and put the motor in closed loop mode
            # TODO Is this output too small?
            self._hood_motor.set_control(controls.DutyCycleOut(0.05))

            angle = self.hood_angle()
            current = self.hood_current()

            # TODO Check 1 degree threshold is okay
            # TODO Maybe use current as well - we expect a spike when stalled, but it will also spike on start
            # TODO Check 2A current threshold is okay
            if abs(angle - self._prev_hood_angle) < 0.5 and current > 4.0:
                self._hood_motor.set_position(
                    self.HOOD_MAX_ANGLE / 360.0 * self.GEAR_RATIO
                )  # max angle is the high, lob shot
                self._initialized = True
            else:
                self._prev_hood_angle = angle
                return  # we can't shoot until we are ready

        # Update hood setpoint even if not shooting
        desired_hood_rotation = (
            numpy.clip(
                self.desired_hood_angle, self.HOOD_MIN_ANGLE, self.HOOD_MAX_ANGLE
            )
            / 360.0
            * self.GEAR_RATIO
        )
        self._hood_motor.set_control(controls.PositionVoltage(desired_hood_rotation))

        if self._should_shoot:
            # spin shooter motor
            self._shooter_motor.set_control(
                controls.DutyCycleOut(self.speed)
                # controls.VelocityVoltage(self.shooter_speed)
            )
        else:
            self._shooter_motor.stopMotor()
