import math

import wpilib
from magicbot import AutonomousStateMachine, state, timed_state
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.path import GoalEndState, PathConstraints, PathPlannerPath
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from components.drivetrain import Drivetrain
from components.intake import Intake
from controllers.shooter import ShooterController
from utilities.game import is_blue, is_red
from utilities.positions import field_flip_pose2d, shooter_to_hub


class AutoBase(AutonomousStateMachine):
    """
    Base class to be used to create more sophisticated routines.
    Allows for easy path following without repeating code.
    """

    field: wpilib.Field2d
    drivetrain: Drivetrain
    intake: Intake
    shooter_controller: ShooterController

    starting_pose: Pose2d | None = None

    def setup(self) -> None:
        # All the things that are the same in each routine...
        self._constraints = PathConstraints(
            maxVelocityMps=3.5,
            maxAccelerationMpsSq=6.0,
            maxAngularVelocityRps=2.0 * math.pi,
            maxAngularAccelerationRpsSq=4.0 * math.pi,
        )
        self._controller = PPHolonomicDriveController(
            translation_constants=PIDConstants(kP=3.0),
            rotation_constants=PIDConstants(kP=5.0),
        )

    def get_starting_pose(self) -> Pose2d | None:
        if self.starting_pose is None:
            return None
        return (
            self.starting_pose if is_blue() else field_flip_pose2d(self.starting_pose)
        )

    def on_enable(self) -> None:
        # configure defaults for pose in sim

        # Setup starting position in the simulator
        starting_pose = self.get_starting_pose()
        if wpilib.RobotBase.isSimulation() and starting_pose is not None:
            self.drivetrain.set_pose(starting_pose)

        super().on_enable()

    def set_trajectory(
        self,
        waypoints: list[Pose2d],
        goal_rotation: Rotation2d,
        field_flip: bool = False,
        mirror: bool = False,
    ) -> bool:
        pp_waypoints = PathPlannerPath.waypointsFromPoses(waypoints)
        pp_path = PathPlannerPath(
            waypoints=pp_waypoints,
            constraints=self._constraints,
            ideal_starting_state=None,
            goal_end_state=GoalEndState(
                velocity=0.0,
                rotation=goal_rotation,
            ),
        )
        if field_flip:
            pp_path = pp_path.flipPath()
        if mirror:
            pp_path = pp_path.mirrorPath()
        self._trajectory = pp_path.generateTrajectory(
            starting_speeds=ChassisSpeeds(),
            starting_rotation=self.drivetrain.pose().rotation(),
            config=self.drivetrain.pp_robot_config,
        )
        auto_path = self.field.getObject("auto")
        auto_path.setPoses([s.pose for s in self._trajectory.getStates()])

        return self._trajectory is not None

    def follow_trajectory(self, state_tm: float) -> None:
        target_state = self._trajectory.sample(state_tm)
        target_speeds = self._controller.calculateRobotRelativeSpeeds(
            self.drivetrain.pose(), target_state
        )
        self.drivetrain.drive_robot(
            target_speeds.vx, target_speeds.vy, target_speeds.omega
        )

    def is_trajectory_expired(self, state_tm: float) -> bool:
        return state_tm > self._trajectory.getTotalTimeSeconds()
    def turn_to_rotation(self, target: Rotation2d) -> bool:
        current = self.drivetrain.pose().rotation()

        error = (target - current).radians()
        error = math.atan2(math.sin(error), math.cos(error))

        kP = 8
        omega = kP * error

        self.drivetrain.drive_robot(0.0, 0.0, omega)

        return abs(error) < math.radians(3)


class Shoot(AutoBase):
    """
    Basic functionality to drive back by 1 metre, then shoot.
    """

    MODE_NAME = "Shoot"

    @state(first=True)
    def driving_to_shoot(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            # Create a trajectory to the shooting position
            robot_pose = self.drivetrain.pose()
            delta_x = -0.5 if is_blue() else 0.5
            path_heading = (
                Rotation2d.fromDegrees(180.0)
                if is_blue()
                else Rotation2d.fromDegrees(0.0)
            )
            shooting_position = Translation2d(robot_pose.x + delta_x, robot_pose.y)
            shooting_pose = Pose2d(shooting_position, path_heading)

            self.set_trajectory(
                [robot_pose, shooting_pose],
                shooter_to_hub(shooting_pose),
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory(state_tm)
        if self.is_trajectory_expired(state_tm):
            self.drivetrain.stop()
            self.next_state("shooting")

    @timed_state(duration=3.0)
    def shooting(self) -> None:
        # Shoot for a fixed period of time
        self.shooter_controller.engage()


class ShootGobblerRight(AutoBase):
    MODE_NAME = "Shoot + Gobbler Right"

    starting_pose = Pose2d(3.6, 0.75, Rotation2d.fromDegrees(0.0))

    @timed_state(first=True, duration=2.5, next_state="turning_collect")
    def shooting(self) -> None:
        # Shoot for a fixed period of time
        self.shooter_controller.engage()
    @state
    def turning_collect(self, initial_call: bool) -> None:
        target_heading = Rotation2d.fromDegrees(-180.0)

        if self.turn_to_rotation(target_heading):
            self.drivetrain.stop()
            self.next_state("collect")
    @state
    def turning_collect2(self, initial_call: bool) -> None:
        target_heading = Rotation2d.fromDegrees(-180.0)

        if self.turn_to_rotation(target_heading):
            self.drivetrain.stop()
            self.next_state("collect2")
    @state
    def turning_return(self, initial_call: bool) -> None:
        target_heading = Rotation2d.fromDegrees(18.0)

        if self.turn_to_rotation(target_heading):
            self.drivetrain.stop()
            self.next_state("returning2")
           
    @state
    def collect(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            # Create a trajectory to the shooting position
            assert self.starting_pose
            p1 = Pose2d(
                self.starting_pose.x + 2.5,
                self.starting_pose.y,
                Rotation2d.fromDegrees(0.0),
            )
            p2 = Pose2d(
                self.starting_pose.x + 4.1,
                self.starting_pose.y + 1.0,
                Rotation2d.fromDegrees(90.0),
            )
            p3 = Pose2d(
                self.starting_pose.x + 4.1,
                self.starting_pose.y + 1.75,
                Rotation2d.fromDegrees(90.0),
            )

            waypoints = [self.starting_pose, p1, p2, p3]

            self.set_trajectory(
                waypoints, Rotation2d.fromDegrees(90.0), field_flip=is_red()
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory(state_tm)
        sp = self.get_starting_pose()
        assert sp
        if self.drivetrain.pose().translation().distance(sp.translation()) > 1.0:
            self.intake.intake()
        if self.is_trajectory_expired(state_tm):
            self.drivetrain.stop()
            self.next_state("returning")

    @state
    def returning(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            # Create a trajectory to the shooting position
            assert self.starting_pose
            p1 = Pose2d(
                self.starting_pose.x + 2.5,
                self.starting_pose.y,
                Rotation2d.fromDegrees(180.0),
            )
            p2 = Pose2d(
                self.starting_pose.x + 4.1,
                self.starting_pose.y + 1.0,
                Rotation2d.fromDegrees(-90.0),
            )
            p3 = Pose2d(
                self.starting_pose.x + 4.1,
                self.starting_pose.y + 1.75,
                Rotation2d.fromDegrees(-90.0),
            )
            sp = Pose2d(
                self.starting_pose.x + 0.0,
                self.starting_pose.y,
                Rotation2d.fromDegrees(180.0),
            )
            waypoints = [p3, p2, p1, sp]

            self.set_trajectory(
                waypoints, Rotation2d.fromDegrees(0.0), field_flip=is_red()
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory(state_tm)
        if self.is_trajectory_expired(state_tm):
            self.drivetrain.stop()
            self.next_state("spraying")

    @timed_state(duration=4, next_state="turning_collect2")
    def spraying(self) -> None:
        # Shoot for a fixed period of time
        self.shooter_controller.engage()

    @state
    def collect2(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            # Create a trajectory to the shooting position
            assert self.starting_pose
            p1 = Pose2d(
                self.starting_pose.x + 2.5,
                self.starting_pose.y,
                Rotation2d.fromDegrees(0.0),
            )
            p2 = Pose2d(
                self.starting_pose.x + 4.1,
                self.starting_pose.y + 1.5,
                Rotation2d.fromDegrees(90.0),
            )
            p3 = Pose2d(
                self.starting_pose.x + 3,
                self.starting_pose.y + 2.5,
                Rotation2d.fromDegrees(90.0),
            )
            sp = Pose2d(
                self.starting_pose.x + 0.0,
                self.starting_pose.y,
                Rotation2d.fromDegrees(0.0),
            )

            waypoints = [sp, p1, p2, p3]

            self.set_trajectory(
                waypoints, Rotation2d.fromDegrees(180.0), field_flip=is_red()
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory(state_tm)
        starting_pose = self.get_starting_pose()
        assert starting_pose
        if (
            self.drivetrain.pose().translation().distance(starting_pose.translation())
            > 1.0
        ):
            self.intake.intake()
        if self.is_trajectory_expired(state_tm):
            self.drivetrain.stop()
            self.next_state("turning_return")

    @state
    def returning2(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            # Create a trajectory to the shooting position
            assert self.starting_pose
            p1 = Pose2d(
                self.starting_pose.x + 2.5,
                self.starting_pose.y,
                Rotation2d.fromDegrees(180.0),
            )
            p2 = Pose2d(
                self.starting_pose.x + 3,
                self.starting_pose.y + 1.5,
                Rotation2d.fromDegrees(-90.0),
            )
            p3 = Pose2d(
                self.starting_pose.x + 3,
                self.starting_pose.y + 2.5,
                Rotation2d.fromDegrees(0.0),
            )
            sp = Pose2d(
                self.starting_pose.x + 0.0,
                self.starting_pose.y,
                Rotation2d.fromDegrees(180.0),
            )
            waypoints = [p3, p2, p1, sp]

            self.set_trajectory(
                waypoints, Rotation2d.fromDegrees(0.0), field_flip=is_red()
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory(state_tm)
        if self.is_trajectory_expired(state_tm):
            self.drivetrain.stop()
            self.next_state("spraying2")
    @timed_state(duration=5.0)
    def spraying2(self) -> None:
        # Shoot for a fixed period of time
        self.shooter_controller.engage()


class GobblerRight(ShootGobblerRight):
    MODE_NAME = "Gobbler only - Right"

    @state(first=True)
    def shooting(self) -> None:
        self.next_state_now("collect")
