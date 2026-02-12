import math

import magicbot
import wpilib
from wpimath.geometry import Rotation3d, Transform3d, Translation3d

from components.drivetrain import Drivetrain
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
from components.vision import Vision


class MyRobot(magicbot.MagicRobot):
    # Components
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

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        self.front_vision.execute()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        vx = -self.gamepad.getLeftY() * self.drivetrain.max_speed
        vy = -self.gamepad.getLeftX() * self.drivetrain.max_speed
        vz = -self.gamepad.getRightX() * self.drivetrain.max_angular_rate
        self.drivetrain.drive_field(vx, vy, vz)

        if self.gamepad.getAButton():
            self.intake.intake()

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        if self.gamepad.getAButton():
            self.intake.intake()
        if self.gamepad.getXButton():
            self.shooter.shoot()
        if self.gamepad.getYButton():
            self.indexer.feed()

        self.shooter.execute()
        self.intake.execute()
        self.indexer.execute()
