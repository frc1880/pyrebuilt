import math

import magicbot
import wpilib
from wpimath.geometry import (
    Rotation2d,
    Rotation3d,
    Transform2d,
    Transform3d,
    Translation2d,
    Translation3d,
)

from autonomous.routines import AutoBase
from components.ballistics import Ballistics
from components.drivetrain import Drivetrain
from components.indexer import Indexer
from components.intake import Intake
from components.leds import Leds
from components.shooter import Shooter
from components.vision import Vision
from controllers.shooter import ShooterController
from utilities import game
from utilities.conversion import inch_to_metre
from utilities.scalers import map_exponential


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
            inch_to_metre(-26.0 / 2 + 4.751181),
            inch_to_metre(28.0 / 2 - 0.795),
            inch_to_metre(13.887533),
        ),
        Rotation3d(0, math.radians(-20), math.radians(90)),
    )
    white_vision: Vision
    white_vision_camera_name = "white"
    white_vision_transform = Transform3d(
        Translation3d(
            inch_to_metre(-26.0 / 2 + 0.633),
            inch_to_metre(-28.0 / 2 + 7.748819),
            inch_to_metre(7.471),
        ),
        Rotation3d(0, math.radians(-30), math.radians(180)),
    )
    blue_vision: Vision
    blue_vision_camera_name = "blue"
    blue_vision_transform = Transform3d(
        Translation3d(
            inch_to_metre(-26.0 / 2 + 8.251),
            inch_to_metre(-28.0 / 2 + 0.633),
            inch_to_metre(7.471),
        ),
        Rotation3d(0, math.radians(-30), math.radians(-90)),
    )

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
            starting_pose = (
                selected_auto.starting_pose
                if game.is_blue()
                else game.field_flip_pose2d(selected_auto.starting_pose)
            )
            error = self.drivetrain.pose() - starting_pose
            # Clean up to add tolerances
            if abs(error.rotation().radians()) < math.radians(10.0):
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

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        self.shooter_vision.execute()
        self.white_vision.execute()
        self.blue_vision.execute()
        self.ballistics.execute()
        self.leds.execute()

        # First check that one of our cameras has seen multitag
        if not (
            self.shooter_vision.is_initialized()
            or self.blue_vision.is_initialized()
            or self.white_vision.is_initialized()
        ):
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

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        vx = -map_exponential(self.gamepad.getLeftY(), 1.5) * self.drivetrain.max_speed
        vy = -map_exponential(self.gamepad.getLeftX(), 1.5) * self.drivetrain.max_speed
        vz = -(
            map_exponential(self.gamepad.getRightX(), 2.0)
            * self.drivetrain.max_angular_rate
        )
        self.drivetrain.drive_field(vx, vy, vz)

        if self.gamepad.getLeftBumper():
            self.intake.retract()
        elif self.gamepad.getLeftTriggerAxis() > 0.5:
            self.intake.intake()
        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.shooter_controller.engage()

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
            self.intake.retract()

        if self.gamepad.getPOV() == 0:
            self.leds.not_in_range()
        if self.gamepad.getPOV() == 180:
            self.leds.in_range()
        if self.gamepad.getPOV() == 90:
            self.leds.intake()

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
