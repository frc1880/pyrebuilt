from phoenix6.swerve.sim_swerve_drivetrain import SimSwerveDrivetrain
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import RoboRioSim
from wpimath.geometry import Translation2d

from generated.tuner_constants import TunerConstants
from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot) -> None:
        self.robot = robot
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
        self.swerve.update(
            tm_diff, self.roborio.getVInVoltage(), self.robot.drivetrain.modules
        )
        states = tuple([m.get_current_state() for m in self.robot.drivetrain.modules])
        # Confirm that we have 4 states in a tuple so type checking by mypy works
        assert len(states) == 4
        speeds = self.robot.drivetrain.kinematics.toChassisSpeeds(states)
        self.physics_controller.drive(speeds, tm_diff)
