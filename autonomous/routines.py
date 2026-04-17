import math

import wpilib
from magicbot import AutonomousStateMachine, state, timed_state
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.path import (
    GoalEndState,
    PathConstraints,
    PathPlannerPath,
    PathPlannerTrajectory,
    RotationTarget,
)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from components.drivetrain import Drivetrain
from components.indexer import Indexer
from components.intake import Intake
from controllers.shooter import ShooterController
from utilities.game import (
    field_flip_pose2d,
    field_flip_rotation2d,
    field_mirror_pose2d,
    field_mirror_rotation2d,
    is_blue,
    is_red,
)
from utilities.positions import shooter_to_hub


class AutoBase(AutonomousStateMachine):
    """
    Base class to be used to create more sophisticated routines.
    Allows for easy path following without repeating code.
    """

    field: wpilib.Field2d
    drivetrain: Drivetrain
    intake: Intake
    indexer: Indexer
    shooter_controller: ShooterController

    blue_starting_pose: Pose2d | None = None
    mirror: bool = False

    def setup(self) -> None:
        # All the things that are the same in each routine...
        self._constraints = PathConstraints(
            maxVelocityMps=3.5,
            maxAccelerationMpsSq=6.0,
            maxAngularVelocityRps=4.0 * math.pi,
            maxAngularAccelerationRpsSq=80.0 * math.pi,
        )
        self._controller = PPHolonomicDriveController(
            translation_constants=PIDConstants(kP=3.0),
            rotation_constants=PIDConstants(kP=8.0, kD=0.25),
        )
        self._trajectories: dict[tuple[str, bool, bool], PathPlannerTrajectory] = {}
        self.precalc_trajectories()

    @property
    def starting_pose(self) -> Pose2d | None:
        if self.blue_starting_pose is None:
            return None
        alliance_pose = (
            self.blue_starting_pose
            if is_blue()
            else field_flip_pose2d(self.blue_starting_pose)
        )
        return alliance_pose if not self.mirror else field_mirror_pose2d(alliance_pose)

    def on_enable(self) -> None:
        # configure defaults for pose in sim

        # Setup starting position in the simulator
        if wpilib.RobotBase.isSimulation() and self.starting_pose is not None:
            self.drivetrain.set_pose(self.starting_pose)

        super().on_enable()

    def generate_trajectory(
        self,
        waypoints: list[Pose2d],
        starting_rotation: Rotation2d,
        goal_rotation: Rotation2d,
        holonomic_rotations: list[RotationTarget] | None = None,
        field_flip: bool = False,
        mirror: bool = False,
    ) -> PathPlannerTrajectory:
        pp_waypoints = PathPlannerPath.waypointsFromPoses(waypoints)
        if holonomic_rotations is None:
            # done this way because Ruff doesn't want mutables as default arguments
            holonomic_rotations = []
        pp_path = PathPlannerPath(
            waypoints=pp_waypoints,
            constraints=self._constraints,
            ideal_starting_state=None,
            goal_end_state=GoalEndState(
                velocity=0.0,
                rotation=goal_rotation,
            ),
            holonomic_rotations=holonomic_rotations,
        )
        if field_flip:
            pp_path = pp_path.flipPath()
            starting_rotation = field_flip_rotation2d(starting_rotation)
        if mirror:
            pp_path = pp_path.mirrorPath()
            starting_rotation = field_mirror_rotation2d(starting_rotation)
        return pp_path.generateTrajectory(
            starting_speeds=ChassisSpeeds(),
            starting_rotation=starting_rotation,
            config=self.drivetrain.pp_robot_config,
        )

    def set_state_trajectory(self) -> None:
        self._trajectory = self._trajectories[
            (self.current_state, is_red(), self.mirror)
        ]
        auto_path = self.field.getObject("auto")
        auto_path.setPoses([s.pose for s in self._trajectory.getStates()])

    def precalc_trajectories(self) -> None:
        pass

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

            self._trajectory = self.generate_trajectory(
                [robot_pose, shooting_pose],
                self.drivetrain.pose().rotation(),
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
    MODE_NAME = "Shoot + Gobbler - Right"

    blue_starting_pose = Pose2d(3.6, 0.75, Rotation2d.fromDegrees(90.0))

    def precalc_trajectories(self) -> None:
        assert self.blue_starting_pose
        # Collect
        # All trajectories assume blue alliance, so flip current pose if required
        sp = Pose2d(
            self.blue_starting_pose.x,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(0.0),
        )
        p1 = Pose2d(
            self.blue_starting_pose.x + 2.5,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(0.0),
        )
        p2 = Pose2d(
            self.blue_starting_pose.x + 4.1 - 1.0,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(0.0),
        )
        p3 = Pose2d(
            self.blue_starting_pose.x + 4.1,
            self.blue_starting_pose.y + 1.0,
            Rotation2d.fromDegrees(90.0),
        )

        p4 = Pose2d(
            self.blue_starting_pose.x + 4.1,
            self.blue_starting_pose.y + 1.8,
            Rotation2d.fromDegrees(90.0),
        )

        waypoints = [sp, p1, p2, p3, p4]

        for flip in [True, False]:
            for mirror in [True, False]:
                self._trajectories[("collect", flip, mirror)] = (
                    self.generate_trajectory(
                        waypoints,
                        self.blue_starting_pose.rotation(),
                        Rotation2d.fromDegrees(90),
                        field_flip=flip,
                        mirror=mirror,
                    )
                )

        # Returning
        # Create a trajectory to the shooting position
        assert self.blue_starting_pose
        sp = Pose2d(
            self.blue_starting_pose.x + 4.1,
            self.blue_starting_pose.y + 1.8,
            Rotation2d.fromDegrees(-90.0),
        )
        p5 = Pose2d(
            self.blue_starting_pose.x + 4.1,
            self.blue_starting_pose.y + 1.0,
            Rotation2d.fromDegrees(-90.0),
        )
        p6 = Pose2d(
            self.blue_starting_pose.x + 4.1 - 1.0,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(180.0),
        )
        p7 = Pose2d(
            self.blue_starting_pose.x,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(180.0),
        )
        waypoints = [sp, p5, p6, p7]

        for flip in [True, False]:
            for mirror in [True, False]:
                self._trajectories[("returning", flip, mirror)] = (
                    self.generate_trajectory(
                        waypoints,
                        Rotation2d.fromDegrees(90.0),
                        Rotation2d.fromDegrees(0.0),
                        field_flip=flip,
                        mirror=mirror,
                    )
                )

        # Circuit
        sp = Pose2d(
            self.blue_starting_pose.x,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(0.0),
        )
        p1 = Pose2d(
            self.blue_starting_pose.x + 2.5,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(0.0),
        )
        p2 = Pose2d(
            self.blue_starting_pose.x + 4.1 - 1.0,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(0.0),
        )
        p3 = Pose2d(
            self.blue_starting_pose.x + 4.1,
            self.blue_starting_pose.y + 1.0,
            Rotation2d.fromDegrees(90.0),
        )
        p4 = Pose2d(
            self.blue_starting_pose.x + 4.1,
            self.blue_starting_pose.y + 1.8,
            Rotation2d.fromDegrees(90.0),
        )
        p5 = Pose2d(
            self.blue_starting_pose.x + 3.6,
            self.blue_starting_pose.y + 3.0,
            Rotation2d.fromDegrees(180.0),
        )
        p6 = Pose2d(
            self.blue_starting_pose.x + 3.1,
            self.blue_starting_pose.y + 1.8,
            Rotation2d.fromDegrees(-90.0),
        )
        p7 = Pose2d(
            self.blue_starting_pose.x + 2.5,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(180.0),
        )
        p8 = Pose2d(
            self.blue_starting_pose.x,
            self.blue_starting_pose.y,
            Rotation2d.fromDegrees(180.0),
        )

        waypoints = [sp, p1, p2, p3, p4, p5, p6, p7, p8]
        rotations = [
            RotationTarget(2.0, Rotation2d.fromDegrees(90.0)),
            RotationTarget(5.0, Rotation2d.fromDegrees(180.0)),
            RotationTarget(6.0, Rotation2d.fromDegrees(-90.0)),
            RotationTarget(7.0, Rotation2d.fromDegrees(180.0)),
        ]

        for flip in [True, False]:
            for mirror in [True, False]:
                self._trajectories[("circuit", flip, mirror)] = (
                    self.generate_trajectory(
                        waypoints,
                        self.blue_starting_pose.rotation(),
                        Rotation2d.fromDegrees(180),
                        rotations,
                        field_flip=flip,
                        mirror=mirror,
                    )
                )

    def on_enable(self) -> None:
        self._cycle_count = 0
        super().on_enable()

    @timed_state(first=True, duration=2.5, next_state="aligning")
    def shooting(self, state_tm: float) -> None:
        # Shoot for a fixed period of time
        self.shooter_controller.engage()
        if self.indexer.is_hopper_empty() and state_tm > 1.45:
            self.next_state("aligning")

    @state
    def aligning(self, initial_call: bool) -> None:
        self.drivetrain.track_heading(
            math.radians(0.0) if is_blue() else math.radians(180.0)
        )
        if self.drivetrain.is_aligned() and not initial_call:
            if self._cycle_count != 1:
                self.next_state("collect")
            else:
                self.next_state("circuit")

    @state
    def collect(self, initial_call: bool, state_tm: float) -> None:
        assert self.blue_starting_pose
        if initial_call:
            self.set_state_trajectory()

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory(state_tm)

        assert self.starting_pose
        in_zone = (self.drivetrain.pose().x < 11) and (self.drivetrain.pose().x > 5.5)
        if in_zone:
            self.intake.intake()
        else:
            self.intake.carry()
        if self.is_trajectory_expired(state_tm):
            self.drivetrain.stop()
            self.next_state("returning")

    @state
    def circuit(self, initial_call: bool, state_tm: float) -> None:
        assert self.blue_starting_pose
        if initial_call:
            self.set_state_trajectory()

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory(state_tm)

        assert self.starting_pose
        in_zone = (self.drivetrain.pose().x < 11) and (self.drivetrain.pose().x > 5.5)
        if in_zone:
            self.intake.intake()
        else:
            self.intake.carry()
        if self.is_trajectory_expired(state_tm):
            self.drivetrain.stop()
            self.next_state("spraying")

    @state
    def returning(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            self.set_state_trajectory()
        self.intake.carry()
        # Follow the trajectory until we are in shooting position
        self.follow_trajectory(state_tm)
        if self.is_trajectory_expired(state_tm):
            self.drivetrain.stop()
            self.next_state("spraying")

    @timed_state(duration=6, next_state="aligning")
    def spraying(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            self._cycle_count += 1
        # Shoot for a fixed period of time
        self.shooter_controller.engage()
        if self.indexer.is_hopper_empty() and state_tm > 2.5 and self._cycle_count == 1:
            self.next_state("aligning")


class GobblerRight(ShootGobblerRight):
    MODE_NAME = "Gobbler only - Right"

    blue_starting_pose = Pose2d(3.6, 0.75, Rotation2d.fromDegrees(0.0))

    @state(first=True)
    def shooting(self) -> None:
        self.next_state_now("aligning")


class ShootGobblerLeft(ShootGobblerRight):
    MODE_NAME = "Shoot + Gobbler - Left"

    mirror = True


class GobblerLeft(GobblerRight):
    MODE_NAME = "Gobbler only - Left"

    mirror = True
