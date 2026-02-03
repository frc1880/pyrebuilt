import magicbot
import wpilib

from components.drivetrain import Drivetrain
from components.intake import Intake
from components.shooter import Shooter


class MyRobot(magicbot.MagicRobot):
    # Components
    drivetrain: Drivetrain
    intake: Intake
    shooter: Shooter

    def createObjects(self) -> None:
        self.gamepad = wpilib.XboxController(0)

        # Visualisation on smartdashboard
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        vx = -self.gamepad.getLeftY() * self.drivetrain.max_speed
        vy = -self.gamepad.getLeftX() * self.drivetrain.max_speed
        vz = -self.gamepad.getRightX() * self.drivetrain.max_angular_rate
        self.drivetrain.drive_field(vx, vy, vz)

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        if self.gamepad.getAButton():
            self.intake.intake()
        if self.gamepad.getXButton():
            self.shooter.shoot()

        self.shooter.execute()
