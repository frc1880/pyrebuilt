import math

import wpilib
from magicbot import AutonomousStateMachine, state, timed_state
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.path import GoalEndState, PathConstraints, PathPlannerPath
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from components.drivetrain import Drivetrain
from controllers.shooter import ShooterController
from utilities.game import is_blue
from utilities.positions import bearing_to_hub


class AutoBase(AutonomousStateMachine):
    """
    Base class to be used to create more sophisticated routines.
    Allows for easy path following without repeating code.
    """

    field: wpilib.Field2d
    drivetrain: Drivetrain
    shooter_controller: ShooterController

    def setup(self) -> None:
        # All the things that are the same in each routine...
        self._constraints = PathConstraints(
            maxVelocityMps=3.0,
            maxAccelerationMpsSq=3.0,
            maxAngularVelocityRps=2.0 * math.pi,
            maxAngularAccelerationRpsSq=4.0 * math.pi,
        )
        self._controller = PPHolonomicDriveController(
            translation_constants=PIDConstants(kP=3.0),
            rotation_constants=PIDConstants(kP=5.0),
        )

    def set_trajectory(
        self, waypoints: list[Pose2d], goal_rotation: Rotation2d
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
        pp_path.preventFlipping = True
        self._trajectory = pp_path.generateTrajectory(
            starting_speeds=ChassisSpeeds(),
            starting_rotation=self.drivetrain.pose().rotation(),
            config=self.drivetrain.pp_robot_config,
        )
        auto_path = self.field.getObject("auto")
        auto_path.setPoses([s.pose for s in self._trajectory.getStates()])

        return self._trajectory is not None

    def follow_trajectory(self, state_tm) -> None:
        target_state = self._trajectory.sample(state_tm)
        target_speeds = self._controller.calculateRobotRelativeSpeeds(
            self.drivetrain.pose(), target_state
        )
        self.drivetrain.drive_robot(
            target_speeds.vx, target_speeds.vy, target_speeds.omega
        )

    def is_trajectory_expired(self, state_tm) -> bool:
        return state_tm > self._trajectory.getTotalTimeSeconds()


class Shoot(AutoBase):
    """
    Basic functionality to drive back by 1 metre, then shoot.
    """

    MODE_NAME = "Shoot"

    @state(first=True)
    def driving_to_shoot(self, initial_call, state_tm) -> None:
        if initial_call:
            # Create a trajectory to the shooting position
            robot_pose = self.drivetrain.pose()
            delta_x = -1.0 if is_blue() else 1.0
            path_heading = (
                Rotation2d.fromDegrees(180.0)
                if is_blue()
                else Rotation2d.fromDegrees(0.0)
            )
            shooting_position = Translation2d(robot_pose.x + delta_x, robot_pose.y)
            shooting_pose = Pose2d(shooting_position, path_heading)

            self.set_trajectory(
                [robot_pose, shooting_pose],
                bearing_to_hub(shooting_position).rotateBy(Rotation2d.fromDegrees(180)),
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
