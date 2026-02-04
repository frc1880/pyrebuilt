from phoenix6.swerve.sim_swerve_drivetrain import SimSwerveDrivetrain
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import RoboRioSim
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics

from generated.tuner_constants import TunerConstants
from robot import MyRobot
from sysid_robot import SysIdRobot


class PhysicsEngine:
    def __init__(
        self,
        physics_controller: PhysicsInterface,
        robot: MyRobot | SysIdRobot,
    ) -> None:
        self.robot = robot
        if isinstance(self.robot, SysIdRobot):
            return

        self.physics_controller = physics_controller

        self.roborio = RoboRioSim()

        swerve_constants = TunerConstants()
        module_constants = [
            swerve_constants.front_left,
            swerve_constants.front_right,
            swerve_constants.back_left,
            swerve_constants.back_right,
        ]
        positions = [
            Translation2d(
                mc.location_x,
                mc.location_y,
            )
            for mc in module_constants
        ]
        self.swerve = SimSwerveDrivetrain(
            positions,
            self.robot.drivetrain.pigeon2.sim_state,
            module_constants,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        if isinstance(self.robot, SysIdRobot):
            return

        self.swerve.update(
            tm_diff, self.roborio.getVInVoltage(), self.robot.drivetrain.modules
        )
        states = tuple([m.get_current_state() for m in self.robot.drivetrain.modules])
        # Confirm that we have 4 states in a tuple so type checking by mypy works
        assert len(states) == 4
        assert isinstance(self.robot.drivetrain.kinematics, SwerveDrive4Kinematics)
        speeds = self.robot.drivetrain.kinematics.toChassisSpeeds(states)
        self.physics_controller.drive(speeds, tm_diff)
