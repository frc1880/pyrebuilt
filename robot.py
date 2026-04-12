import math

import magicbot
import wpilib
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform2d,
    Transform3d,
    Translation2d,
    Translation3d,
)
from wpimath.units import inchesToMeters

from autonomous.routines import AutoBase
from components.ballistics import Ballistics
from components.drivetrain import Drivetrain
from components.indexer import Indexer
from components.intake import Intake
from components.leds import Leds
from components.shooter import Shooter
from components.vision import Vision
from components.vision_coordinator import VisionCoordinator
from controllers.shooter import ShooterController
from utilities import game, positions
from utilities.scalers import apply_deadzone, map_exponential


####################
# Enabling LiveWindow breaks test mode with a CANRange
# MagicBot doesn't use LiveWindow in teleop or auto
# Monkeypatch it so it won't start in test mode either
def noop(enabled: bool) -> None:
    pass


wpilib.LiveWindow.setEnabled = noop  # type: ignore[method-assign]
# END OF MONKEYPATCH
####################


class MyRobot(magicbot.MagicRobot):
    # CONTROLLERS MUST COME FIRST SO THAT will_reset_to works properly!!
    # Controllers
    shooter_controller: ShooterController

    # Components
    ballistics: Ballistics
    drivetrain: Drivetrain
    intake: Intake
    shooter: Shooter
    indexer: Indexer
    leds: Leds

    shooter_vision: Vision
    shooter_vision_camera_name = "shooter"
    # Offsets are measured from the robot corner in CAD, hence the calcs below
    shooter_vision_transform = Transform3d(
        Translation3d(
            inchesToMeters(-26.0 / 2 + 0.25),
            inchesToMeters(28.0 / 2 - 6.5),
            inchesToMeters(12.875),
        ),
        Rotation3d(0, math.radians(-30), math.radians(180)),
    )
    red_vision: Vision
    red_vision_camera_name = "red"
    red_vision_transform = Transform3d(
        Translation3d(
            inchesToMeters(-26.0 / 2 + 5.123),
            inchesToMeters(28.0 / 2 - 1.039),
            inchesToMeters(7.125),
        ),
        Rotation3d(0, math.radians(-30), math.radians(90)),
    )
    white_vision: Vision
    white_vision_camera_name = "white"
    white_vision_transform = Transform3d(
        Translation3d(
            inchesToMeters(-26.0 / 2 + 5.123),
            -inchesToMeters(28.0 / 2 - 1.039),
            inchesToMeters(7.125),
        ),
        Rotation3d(0, math.radians(-30), math.radians(-90)),
    )

    vision_coordinator: VisionCoordinator

    def createObjects(self) -> None:
        self.gamepad = wpilib.XboxController(0)

        # Visualisation on smartdashboard
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        # Variables used in test mode
        self._test_shooter_on = False

    def _get_start_pose_error(self, selected_auto: AutoBase) -> Transform2d:
        # Check that we are in the right spot to start
        if selected_auto.starting_pose:
            pose = self.drivetrain.pose()
            translation_error = (
                pose.translation() - selected_auto.starting_pose.translation()
            )
            rotation_error = pose.rotation() - selected_auto.starting_pose.rotation()

            # make the light codes relative to the robot
            translation_error = translation_error.rotateBy(-pose.rotation())

            error = Transform2d(translation_error, rotation_error)
            if abs(error.rotation().degrees()) < 20.0:
                # Blank out the rotation if below the threshold
                error = Transform2d(error.translation(), Rotation2d())
            if abs(error.x) < 0.2:
                # Blank out x error if below threshold
                error = Transform2d(Translation2d(0.0, error.y), error.rotation())
            if abs(error.y) < 0.2:
                # Same for y
                error = Transform2d(Translation2d(error.x, 0.0), error.rotation())
            return error
        else:
            # Can start from anywhere
            return Transform2d()

    def _update_vision(self) -> None:
        self.shooter_vision.execute()
        self.red_vision.execute()
        self.white_vision.execute()

    def is_vision_alive(self) -> bool:
        self._update_vision()  # Run in case we forget to do it in a mode that needs it
        return (
            self.shooter_vision.alive()
            or self.red_vision.alive()
            or self.white_vision.alive()
        )

    def is_vision_initialized(self) -> bool:
        return (
            self.shooter_vision.is_initialized()
            or self.red_vision.is_initialized()
            or self.white_vision.is_initialized()
        )

    def disabledInit(self) -> None:
        self.gamepad.setRumble(self.gamepad.RumbleType.kLeftRumble, 0.0)
        self.gamepad.setRumble(self.gamepad.RumbleType.kRightRumble, 0.0)

    def disabledPeriodic(self) -> None:
        self._update_vision()
        self.ballistics.execute()
        self.leds.execute()

        # First check that one of our cameras has seen multitag in the last 2 seconds
        if self.is_vision_initialized() or wpilib.DriverStation.isFMSAttached():
            if not self.is_vision_alive():
                self.leds.missing_vision()
            else:
                # Indicate that we don't have an auto mode selected
                selected_auto = self._automodes.chooser.getSelected()
                if not isinstance(selected_auto, AutoBase):
                    # No auto so set the lights
                    self.leds.missing_auto()
                else:
                    error = self._get_start_pose_error(selected_auto)
                    if error != Transform2d():
                        self.leds.wrong_start(error)
                    else:
                        self.leds.disabled()
        else:
            self.leds.off()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        vx = (
            -map_exponential(apply_deadzone(self.gamepad.getLeftY(), 0.1), 1.5)
            * self.drivetrain.max_speed
        )
        vy = (
            -map_exponential(apply_deadzone(self.gamepad.getLeftX(), 0.1), 1.5)
            * self.drivetrain.max_speed
        )
        vz = -(
            map_exponential(apply_deadzone(self.gamepad.getRightX(), 0.1), 2.0)
            * self.drivetrain.max_angular_rate
        )
        self.drivetrain.drive_field(vx, vy, vz)
        if self.intake.timeSinceDeployed > 2 and self.intake.deployed:
            self.gamepad.setRumble(self.gamepad.RumbleType.kLeftRumble, 0.5)
            self.gamepad.setRumble(self.gamepad.RumbleType.kRightRumble, 0.5)

        else:
            self.gamepad.setRumble(self.gamepad.RumbleType.kLeftRumble, 0.0)
            self.gamepad.setRumble(self.gamepad.RumbleType.kRightRumble, 0.0)

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.intake.intake()
        if self.gamepad.getLeftBumper():
            self.intake.carry()
        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.shooter_controller.engage()
        elif self.gamepad.getAButton():
            self.indexer.backdrive()
            self.intake.backdrive()
        if self.gamepad.getXButton():
            home_pose = Pose2d(
                12.972, 3.915, Rotation2d()
            )  # measured from robotigers' practice field
            home_pose = (
                positions.field_flip_pose2d(home_pose) if game.is_blue() else home_pose
            )
            self.drivetrain.set_pose(home_pose)

    def testInit(self) -> None:
        self._test_shooter_on = False

    def testPeriodic(self) -> None:
        if self.gamepad.getXButtonPressed():
            self._test_shooter_on = not self._test_shooter_on
        if self.gamepad.getRightTriggerAxis() > 0.5 and self._test_shooter_on:
            self.indexer.feed()
        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.intake.intake()
        if self.gamepad.getLeftBumper():
            self.intake.carry()
        if self.gamepad.getPOV() == 180:
            self.intake.retract()
        if self.intake.timeSinceDeployed > 2 and self.intake.deployed:
            self.gamepad.setRumble(self.gamepad.RumbleType.kLeftRumble, 0.5)
            self.gamepad.setRumble(self.gamepad.RumbleType.kRightRumble, 0.5)
        else:
            self.gamepad.setRumble(self.gamepad.RumbleType.kLeftRumble, 0.0)
            self.gamepad.setRumble(self.gamepad.RumbleType.kRightRumble, 0.0)
        if self.gamepad.getLeftBumper():
            self.leds.disabled()
            if self.gamepad.getPOV() == 0:
                self.leds.wrong_start(Transform2d(-1, 0, Rotation2d()))
            if self.gamepad.getPOV() == 180:
                self.leds.wrong_start(Transform2d(1, 0, Rotation2d()))
            if self.gamepad.getPOV() == 90:
                self.leds.wrong_start(Transform2d(0, 1, Rotation2d()))
            if self.gamepad.getPOV() == 270:
                self.leds.wrong_start(Transform2d(0, -1, Rotation2d()))
            if self.gamepad.getLeftStickButton():
                self.leds.wrong_start(Transform2d(0, -1, Rotation2d.fromDegrees(-30)))
            if self.gamepad.getRightStickButton():
                self.leds.wrong_start(Transform2d(0, -1, Rotation2d.fromDegrees(30)))

            if self.gamepad.getAButton():
                self.leds.in_range()
            if self.gamepad.getBButton():
                self.leds.in_range(True)
            if self.gamepad.getXButton():
                self.leds.not_in_range()
            if self.gamepad.getYButton():
                self.leds.not_in_range(True)
        if self.gamepad.getAButton():
            self.indexer.backdrive()
            self.intake.backdrive()
        if self._test_shooter_on:
            self.shooter.shoot()

        if self.gamepad.getRightBumper():
            speed_scaling = 0.25
            vx = (
                -map_exponential(self.gamepad.getLeftY(), 1.5)
                * self.drivetrain.max_speed
                * speed_scaling
            )
            vy = (
                -map_exponential(self.gamepad.getLeftX(), 1.5)
                * self.drivetrain.max_speed
                * speed_scaling
            )
            vz = -(
                map_exponential(self.gamepad.getRightX(), 2.0)
                * self.drivetrain.max_angular_rate
                * speed_scaling
            )
            self.drivetrain.drive_robot(vx, vy, vz)

        self.drivetrain.execute()
        self.shooter.execute()
        self.intake.execute()
        self.indexer.execute()
        self.leds.execute()
        self._update_vision()
