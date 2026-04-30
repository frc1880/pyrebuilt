import math

import wpilib
from magicbot import AutonomousStateMachine, state, timed_state
from wpilib import DataLogManager
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpiutil.log import BooleanLogEntry, DoubleLogEntry

from components.drivetrain import Drivetrain
from components.indexer import Indexer
from components.intake import Intake
from components.vision_coordinator import VisionCoordinator
from controllers.shooter import ShooterController
from motion import vector_pursuit
from utilities.game import (
    field_flip_pose2d,
    field_mirror_pose2d,
    is_blue,
    is_red,
)
from utilities.positions import is_in_alliance_zone, shooter_to_hub


class AutoBase(AutonomousStateMachine):
    """
    Base class to be used to create more sophisticated routines.
    Allows for easy path following without repeating code.
    """

    field: wpilib.Field2d
    drivetrain: Drivetrain
    intake: Intake
    indexer: Indexer
    vision_coordinator: VisionCoordinator
    shooter_controller: ShooterController

    blue_starting_pose: Pose2d | None = None
    mirror: bool = False

    def setup(self) -> None:
        # All the things that are the same in each routine...
        constraints = vector_pursuit.MotionParameters(
            translation_tolerance=0.1,
            rotation_tolerance=math.radians(15),
            max_linear_speed=2.5,
            max_linear_acceleration=5.0,
            max_angular_speed=3.0 * math.pi,
        )
        self._controller = vector_pursuit.VectorPursuitController(
            rotation_controller=PIDController(Kp=8.5, Ki=0.0, Kd=0.25),
            cross_track_controller=PIDController(Kp=3.0, Ki=0.0, Kd=0.0),
            motion_parameters=constraints,
        )

        DataLogManager.start()
        DataLogManager.logNetworkTables(True)
        log = DataLogManager.getLog()
        self._angular_error = DoubleLogEntry(log, "/auto/angular_error")
        self._translational_error = DoubleLogEntry(log, "/auto/translational_error")
        self._cross_track_error = DoubleLogEntry(log, "/auto/translational_error")
        self._angular_goal = BooleanLogEntry(log, "/auto/angular_goal")
        self._translational_goal = BooleanLogEntry(log, "/auto/translational_goal")
        self._cross_track_goal = BooleanLogEntry(log, "/auto/cross_track_goal")

    def log_errors(self):
        self._angular_error.append(
            self._controller._rotation_controller.getPositionError()
        )
        self._translational_error.append(
            self._controller._translation_controller.getPositionError()
        )
        self._cross_track_error.append(
            self._controller._cross_track_controller.getPositionError()
        )
        self._angular_goal.append(self._controller._rotation_controller.atSetpoint())
        self._translational_goal.append(
            self._controller._translation_controller.atSetpoint()
        )
        self._cross_track_goal.append(
            self._controller._cross_track_controller.atSetpoint()
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
        if (
            wpilib.RobotBase.isSimulation() or not self.vision_coordinator.on_field()
        ) and self.starting_pose is not None:
            self.drivetrain.set_pose(self.starting_pose)

        super().on_enable()

    def set_trajectory(
        self,
        path_points: list[vector_pursuit.PathPoint],
        field_flip: bool,
        mirror: bool,
    ) -> None:
        self._controller.set_path(
            self.drivetrain.pose(),
            path_points=path_points,
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
    blue_starting_pose = field_flip_pose2d(
        Pose2d(12.972, 3.915, Rotation2d())
    )  # measured from robotigers' practice field

    @state(first=True)
    def driving_to_shoot(self, initial_call: bool, state_tm: float) -> None:

        if initial_call:
            # Create a trajectory to the shooting position
            assert self.starting_pose
            robot_pose = self.starting_pose
            delta_x = -1 if is_blue() else 1
            shooting_position = Translation2d(robot_pose.x + delta_x, robot_pose.y)

            self.set_trajectory(
                [
                    vector_pursuit.PathPoint(
                        shooting_position,
                        shooter_to_hub(Pose2d(shooting_position, Rotation2d())),
                    )
                ],
                field_flip=False,
                mirror=False,
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


class ShootDepot(AutoBase):
    """
    Basic functionality to drive back by 1 metre, then shoot.
    """

    MODE_NAME = "Shoot + Depot"
    blue_starting_pose = field_flip_pose2d(
        Pose2d(12.972, 3.915, Rotation2d())
    )  # measured from robotigers' practice field

    def on_enable(self) -> None:
        self._cycle_count = 0
        super().on_enable()

    @state(first=True)
    def driving_to_shoot(self, initial_call: bool, state_tm: float) -> None:

        if initial_call:
            # Create a trajectory to the shooting position
            assert self.blue_starting_pose
            robot_pose = self.blue_starting_pose
            delta_x = 1 if is_blue() else -1
            shooting_position = Translation2d(robot_pose.x + delta_x, robot_pose.y)
            p1 = vector_pursuit.PathPoint(
                shooting_position,
                shooter_to_hub(Pose2d(shooting_position, Rotation2d())),
            )
            p2 = vector_pursuit.PathPoint(
                Translation2d(
                    shooting_position.x,
                    self.blue_starting_pose.y - 2,
                ),
            )
            p3 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x - 3,
                    self.blue_starting_pose.y - 2,
                ),
            )
            p4 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x - 2,
                    self.blue_starting_pose.y - 2,
                ),
            )
            match self._cycle_count:
                case 1:
                    waypoints = [p2, p3]
                case 2:
                    waypoints = [p4]
                case _:
                    waypoints = [p1]
            self.set_trajectory(
                waypoints,
                field_flip=True,
                mirror=True,
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()
        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self._cycle_count += 1
            if self._cycle_count == 2:
                self.next_state("driving_to_shoot")
            else:
                self.next_state("shooting")

    @timed_state(duration=3.0)
    def shooting(self) -> None:
        # Shoot for a fixed period of time
        self.shooter_controller.engage()
        if self.indexer.is_hopper_empty() and self._cycle_count == 1:
            self.next_state("driving_to_shoot")


class ShootHuman(AutoBase):
    """
    Basic functionality to drive back by 1 metre, then shoot.
    """

    MODE_NAME = "Shoot + Human Player"
    blue_starting_pose = field_flip_pose2d(
        Pose2d(12.972, 3.915, Rotation2d())
    )  # measured from robotigers' practice field

    def on_enable(self) -> None:
        self._cycle_count = 0
        super().on_enable()

    @state(first=True)
    def drive_handler(self, initial_call: bool, state_tm: float) -> None:

        if initial_call:
            # Create a trajectory to the shooting position
            assert self.starting_pose
            robot_pose = self.starting_pose
            delta_x = -1 if is_blue() else 1
            shooting_position = Translation2d(robot_pose.x + delta_x, robot_pose.y)
            p1 = vector_pursuit.PathPoint(
                shooting_position,
                shooter_to_hub(Pose2d(shooting_position, Rotation2d())),
            )
            p2 = vector_pursuit.PathPoint(
                Translation2d(
                    shooting_position.x,
                    self.starting_pose.y + 3,
                ),
            )
            p3 = vector_pursuit.PathPoint(
                Translation2d(
                    self.starting_pose.x - 3,
                    self.starting_pose.y + 3,
                ),
            )
            p4 = vector_pursuit.PathPoint(
                Translation2d(
                    self.starting_pose.x - 2.5,
                    self.starting_pose.y + 2.5,
                ),
            )
            match self._cycle_count:
                case 1:
                    waypoints = [p2, p3]
                case 2:
                    waypoints = [p4]
                case _:
                    waypoints = [p1]

            self.set_trajectory(
                waypoints,
                field_flip=False,
                mirror=True,
            )

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()
        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self._cycle_count += 1
            if self._cycle_count == 2:
                self.next_state("wait_human_player")
            else:
                self.next_state("shooting")

    @timed_state(duration=3.0, next_state="drive_handler")
    def wait_human_player(self):
        self.intake.intake()

    @timed_state(duration=3.0)
    def shooting(self) -> None:
        # Shoot for a fixed period of time
        self.shooter_controller.engage()
        if self.indexer.is_hopper_empty() and self._cycle_count == 1:
            self.next_state("drive_handler")


class ShootGobblerRight(AutoBase):
    MODE_NAME = "Shoot + Gobbler - Right"

    blue_starting_pose = Pose2d(3.6, 0.75, Rotation2d.fromDegrees(90.0))

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
                self.next_state("hub_collect")

    @state
    def collect(self, initial_call: bool) -> None:
        if initial_call:
            assert self.blue_starting_pose
            # Collect
            # All trajectories assume blue alliance, so flip current pose if required
            p1 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 2.5,
                    self.blue_starting_pose.y,
                ),
                Rotation2d.fromDegrees(0.0),
            )
            p2 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1 - 1.0,
                    self.blue_starting_pose.y,
                ),
                Rotation2d.fromDegrees(90.0),
            )
            p3 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1,
                    self.blue_starting_pose.y + 1.0,
                )
            )

            p4 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1,
                    self.blue_starting_pose.y + 2.5,
                )
            )

            path_points = [p1, p2, p3, p4]
            self.set_trajectory(path_points, field_flip=is_red(), mirror=self.mirror)

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()

        # Handle Intake
        if is_in_alliance_zone(self.drivetrain.pose()):
            self.intake.carry()
        else:
            self.intake.intake()

        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self.next_state("returning")

    @state
    def hub_collect(self, initial_call: bool) -> None:
        if initial_call:
            assert self.blue_starting_pose
            sp = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x - 0.5,
                    self.blue_starting_pose.y,
                ),
                Rotation2d.fromDegrees(0.0),
            )
            p1 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 2.2,
                    self.blue_starting_pose.y,
                ),
                Rotation2d.fromDegrees(0.0),
            )
            p2 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 3.2 - 0.8,
                    self.blue_starting_pose.y,
                ),
            )
            p3 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 3.2,
                    self.blue_starting_pose.y + 0.8,
                ),
                Rotation2d.fromDegrees(90.0),
            )

            p4 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 3.2,
                    self.blue_starting_pose.y + 3.0,
                ),
            )

            waypoints = [sp, p1, p2, p3, p4]
            self.set_trajectory(waypoints, field_flip=is_red(), mirror=self.mirror)

        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()

        # Handle Intake
        if is_in_alliance_zone(self.drivetrain.pose()):
            self.intake.carry()
        else:
            self.intake.intake()

        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self.next_state("hub_returning")

    @state
    def returning(self, initial_call: bool) -> None:
        if initial_call:
            assert self.blue_starting_pose
            p5 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1,
                    self.blue_starting_pose.y + 1.0,
                ),
            )
            p6 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 4.1 - 1.0,
                    self.blue_starting_pose.y,
                ),
                Rotation2d.fromDegrees(0.0),
            )
            p7 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x,
                    self.blue_starting_pose.y,
                ),
                Rotation2d.fromDegrees(0.0),
            )
            p8 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x - 1.0,
                    self.blue_starting_pose.y + 1.0,
                ),
                Rotation2d.fromDegrees(-135.0),
            )
            waypoints = [p5, p6, p7, p8]
            self.set_trajectory(waypoints, field_flip=is_red(), mirror=self.mirror)

        self.intake.carry()
        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()

        # log Errors
        self.log_errors()
        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self.next_state("spraying")

    @state
    def hub_returning(self, initial_call: bool) -> None:
        if initial_call:
            assert self.blue_starting_pose
            p5 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 3.2,
                    self.blue_starting_pose.y + 0.8,
                ),
            )
            p6 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x + 3.2 - 0.8,
                    self.blue_starting_pose.y,
                ),
            )
            p7 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x,
                    self.blue_starting_pose.y,
                ),
                Rotation2d.fromDegrees(0.0),
            )
            p8 = vector_pursuit.PathPoint(
                Translation2d(
                    self.blue_starting_pose.x - 1.0,
                    self.blue_starting_pose.y + 1.0,
                ),
                Rotation2d.fromDegrees(-135.0),
            )
            waypoints = [p5, p6, p7, p8]
            self.set_trajectory(waypoints, field_flip=is_red(), mirror=self.mirror)

        self.intake.carry()
        # Follow the trajectory until we are in shooting position
        self.follow_trajectory()
        # log Errors
        self.log_errors()
        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self.next_state("spraying")

    @state
    def spraying(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            self._cycle_count += 1
        # Shoot for a fixed period of time
        self.shooter_controller.engage()
        if self.indexer.is_hopper_empty() and self._cycle_count == 1:
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


class TestDriving(AutoBase):
    MODE_NAME = "Driving test"

    @timed_state(first=True, duration=2.0, next_state="drive")
    def pause(self):
        pass

    @state
    def drive(self, initial_call: bool):
        if initial_call:
            self.drivetrain.set_pose(Pose2d())
            pp = vector_pursuit.PathPoint(
                Translation2d(2.0, 0.0), Rotation2d(), speed=1.0
            )
            self.set_trajectory(
                [
                    pp,
                ],
                False,
                False,
            )
        self.follow_trajectory()
        if self.is_trajectory_expired():
            self.drivetrain.stop()
            self.done()
