from typing import NamedTuple

import numpy
from magicbot import feedback, tunable
from wpimath.geometry import Rotation2d, Translation2d

from components.drivetrain import Drivetrain
from utilities.conversion import inch_to_metre
from utilities.positions import hub_position, is_in_alliance_zone


class Solution(NamedTuple):
    flywheel_speed: float
    hood_angle: float
    bearing: float


class Sample(NamedTuple):
    range: float
    hood_angle: float
    flywheel_speed: float
    time_of_flight: float


class Ballistics:
    """
    Keep track of our current position and use it to calculate
    the correct flywheel speed and hood angle.
    """

    # We will need the drivebase so that we can get our current position
    drivetrain: Drivetrain

    # We want to tune the latency of our motion compensation
    latency = tunable(0.15)

    # Ranges measured from hub corner (so starting at 54inch) and to front bumper (add 18inch)
    samples = [
        Sample(inch_to_metre(54 + 18), 70.0, 43.0, 1.0),
        Sample(inch_to_metre(74 + 18), 70.0, 46.0, 1.0),
        Sample(inch_to_metre(94 + 18), 65.0, 47.0, 1.0),
        Sample(inch_to_metre(114 + 18), 60.0, 49.0, 1.0),
        Sample(inch_to_metre(134 + 18), 60.0, 53.0, 1.0),
        Sample(inch_to_metre(154 + 18), 60.0, 55.0, 1.0),
        Sample(inch_to_metre(174 + 18), 60.0, 58.0, 1.0),
        Sample(inch_to_metre(194 + 18), 56.0, 60.0, 1.0),
    ]

    # Split the samples out to make interpolating easier
    ranges = [s.range for s in samples]  # metres
    hood_angles = [s.hood_angle for s in samples]  # degrees from horizontal
    flywheel_speeds = [s.flywheel_speed for s in samples]  # rev/s
    time_of_flight = [s.time_of_flight for s in samples]  # seconds
    horizontal_velocities = [d / t for d, t in zip(ranges, time_of_flight)]

    min_score_range = ranges[0]
    max_score_range = ranges[-1]

    def __init__(self) -> None:
        for pre, post in zip(
            self.horizontal_velocities, self.horizontal_velocities[1:]
        ):
            assert pre <= post
        self._solution = Solution(flywheel_speed=0.0, hood_angle=0.0, bearing=0.0)
        self._in_range = False

    def _tof(self, distance: float) -> float:
        return numpy.interp(distance, self.ranges, self.time_of_flight)

    def _speed(self, horizontal_velocity: float) -> float:
        return numpy.interp(
            horizontal_velocity, self.horizontal_velocities, self.flywheel_speeds
        )

    def _angle(self, horizontal_velocity: float) -> float:
        return numpy.interp(
            horizontal_velocity, self.horizontal_velocities, self.hood_angles
        )

    @feedback
    def solution(self) -> Solution:
        return self._solution

    @feedback
    def is_within_range(self) -> bool:
        """
        Returns True if current robot-to-hub distance is inside the table.
        """
        return self._in_range

    def execute(self) -> None:
        """
        Calculate the required speed and angle and store it in the _solution variable.
        """
        # We are compensating for motion, so we need to project forward to account for latency first

        robot_pose = self.drivetrain.pose()
        robot_position = robot_pose.translation()
        current_swerve_velocity = self.drivetrain.velocity()
        robot_velocity = Translation2d(
            current_swerve_velocity.vx, current_swerve_velocity.vy
        ).rotateBy(robot_pose.rotation())
        future_position = robot_position + (robot_velocity * self.latency)

        # Directions and distances to hub
        robot_to_hub = hub_position() - future_position
        distance_to_hub = robot_to_hub.norm()
        direction_to_hub = robot_to_hub / distance_to_hub

        # Baseline (non-compensated)
        tof = self._tof(distance_to_hub)
        baseline_speed = distance_to_hub / tof

        # Total velocity to get to hub
        target_velocity = direction_to_hub * baseline_speed
        # Remove robot velocity to get shot velocity
        shot_velocity = target_velocity - robot_velocity

        # Extract shot parameters
        # Compensate for rotation of shooter in chassis
        shot_heading = shot_velocity.angle().rotateBy(Rotation2d.fromDegrees(-90.0))
        shot_speed = shot_velocity.norm()

        # Interpolate based on time to get the shot speed/hood angle
        self._solution = Solution(
            self._speed(shot_speed), self._angle(shot_speed), shot_heading.radians()
        )

        self._in_range = (
            self.min_score_range < distance_to_hub < self.max_score_range
            and is_in_alliance_zone(robot_pose)
        )
