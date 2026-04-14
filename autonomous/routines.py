import math

import wpilib
from magicbot import AutonomousStateMachine, state, timed_state
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from bline import bline
from components.drivetrain import Drivetrain
from components.indexer import Indexer
from components.intake import Intake
from controllers.shooter import ShooterController
from utilities.game import (
    field_flip_pose2d,
    field_mirror_pose2d,
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
        constraints = bline.MotionParameters(
            translation_tolerance=0.1,
            rotation_tolerance=math.radians(5),
            max_linear_speed=3.5,
            max_linear_acceleration=6.0 * 0.0,
            max_angular_speed=2.0 * math.pi,
            max_angular_acceleration=4.0 * math.pi * 0.0,
        )
        self._controller = bline.BLine(
            translation_controller=PIDController(Kp=3.0, Ki=0.0, Kd=0.0),
            rotation_controller=PIDController(Kp=2.0, Ki=0.0, Kd=0.0),
            # rotation_controller=PIDController(Kp=8.0, Ki=0.0, Kd=0.25),
            cross_track_controller=PIDController(Kp=3.0, Ki=0.0, Kd=0.0),
            motion_parameters=constraints,
        )

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

    def set_trajectory(
        self,
        waypoints: list[bline.Waypoint],
        field_flip: bool = False,
        mirror: bool = False,
    ) -> None:
        self._controller.set_path(
            self.drivetrain.pose(),
            waypoints=waypoints,
            should_flip=field_flip,
            should_mirror=mirror,
        )

        auto_path = self.field.getObject("auto")
        auto_path.setPoses(self._controller.waypoints())

    def follow_trajectory(self) -> None:
        target_speeds = self._controller.calculate(self.drivetrain.pose())
        self.drivetrain.drive_robot(
            target_speeds.vx, target_speeds.vy, target_speeds.omega
        )

    def is_trajectory_expired(self) -> bool:
        return self._controller.is_at_goal()


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
            shooting_rotation = shooter_to_hub(shooting_pose)

            self.set_trajectory(
                [
                    bline.Waypoint(shooting_position, shooting_rotation),
                ]
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()
        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self.next_state("shooting")

    @timed_state(duration=3.0)
    def shooting(self) -> None:
        # Shoot for a fixed period of time
        self.shooter_controller.engage()


class ShootGobblerRight(AutoBase):
    MODE_NAME = "Shoot + Gobbler - Right"

    blue_starting_pose = Pose2d(3.6, 0.75, Rotation2d.fromDegrees(-90.0))

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
            self.next_state("collect")

    @state
    def collect(self, initial_call: bool, state_tm: float) -> None:
        assert self.blue_starting_pose
        if initial_call:
            # Create a trajectory to the shooting position
            p1 = bline.Waypoint(
                Translation2d(
                    self.blue_starting_pose.x + 2.5, self.blue_starting_pose.y
                ),
                Rotation2d.fromDegrees(0.0),
            )
            p2 = bline.Waypoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1 - 1, self.blue_starting_pose.y
                ),
                None,
            )
            p3 = bline.Waypoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1, self.blue_starting_pose.y + 1
                ),
                Rotation2d.fromDegrees(90.0),
            )

            p4 = bline.Waypoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1,
                    self.blue_starting_pose.y + 2 + 0.5 * self._cycle_count,
                ),
                Rotation2d.fromDegrees(90.0),
            )

            waypoints = [p1, p2, p3, p4]

            self.set_trajectory(
                waypoints,
                field_flip=is_red(),
                mirror=self.mirror,
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()

        assert self.starting_pose
        in_zone = (self.drivetrain.pose().x < 11) and (self.drivetrain.pose().x > 5.5)
        if in_zone:
            self.intake.intake()
        else:
            self.intake.carry()
        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self._cycle_count += 1
            self.next_state("returning")

    @state
    def returning(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            # Create a trajectory to the shooting position
            assert self.blue_starting_pose
            p5 = bline.Waypoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1, self.blue_starting_pose.y + 1
                ),
                Rotation2d.fromDegrees(90.0),
            )
            p6 = bline.Waypoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1 - 1, self.blue_starting_pose.y
                ),
                None,
            )
            p7 = bline.Waypoint(
                Translation2d(
                    self.blue_starting_pose.x + 2.5, self.blue_starting_pose.y
                ),
                Rotation2d.fromDegrees(0.0),
            )
            p8 = bline.Waypoint(
                Translation2d(self.blue_starting_pose.x, self.blue_starting_pose.y),
                Rotation2d.fromDegrees(0.0),
            )
            waypoints = [p5, p6, p7, p8]

            self.set_trajectory(
                waypoints,
                field_flip=is_red(),
                mirror=self.mirror,
            )
        self.intake.carry()
        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()
        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self.next_state("spraying")

    @timed_state(duration=6, next_state="aligning")
    def spraying(self, state_tm: float) -> None:
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
