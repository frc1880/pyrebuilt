import magicbot
import wpilib

from components.drivetrain import Drivetrain


class Jackie(magicbot.MagicRobot):
    # Components
    drivetrain: Drivetrain

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
        pass

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass
