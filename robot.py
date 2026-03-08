import math

import magicbot
import wpilib
from wpimath.geometry import Rotation3d, Transform3d, Translation3d

from components.ballistics import Ballistics
from components.drivetrain import Drivetrain
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
from components.vision import Vision
from utilities.scalers import map_exponential


class MyRobot(magicbot.MagicRobot):
    # Components
    ballistics: Ballistics
    drivetrain: Drivetrain
    intake: Intake
    shooter: Shooter
    indexer: Indexer

    front_vision: Vision
    front_vision_camera_name = "Front Camera"
    front_vision_transform = Transform3d(
        Translation3d(), Rotation3d(0, math.radians(-30), 0)
    )

    def createObjects(self) -> None:
        self.gamepad = wpilib.XboxController(0)

        # Visualisation on smartdashboard
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        # Variables used in test mode
        self._test_shooter_on = False

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        self.front_vision.execute()
        self.ballistics.execute()

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

        if self.gamepad.getAButton():
            self.intake.intake()
        if self.gamepad.getYButton():
            self.drivetrain.track_hub()

    def testInit(self) -> None:
        self._test_shooter_on = False

    def testPeriodic(self) -> None:
        if self.gamepad.getAButton():
            self.intake.intake()
        if self.gamepad.getXButtonPressed():
            self._test_shooter_on = not self._test_shooter_on
        if self.gamepad.getYButton():
            self.indexer.feed()

        if self._test_shooter_on:
            self.shooter.shoot()

        if self.gamepad.getLeftBumper():
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
