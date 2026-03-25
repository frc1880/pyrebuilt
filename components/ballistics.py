from typing import NamedTuple

import numpy
from magicbot import feedback

from components.drivetrain import Drivetrain
from utilities.conversion import inch_to_metre
from utilities.positions import hub_position, is_in_alliance_zone


class Solution(NamedTuple):
    flywheel_speed: float
    hood_angle: float


class Ballistics:
    """
    Keep track of our current position and use it to calculate
    the correct flywheel speed and hood angle.
    """

    # We will need the drivebase so that we can get our current position
    drivetrain: Drivetrain

    # Ranges measured from hub corner (so starting at 54inch) and to front bumper (add 18inch)
    ranges = [
        inch_to_metre(54 + 18),
        inch_to_metre(74 + 18),
        inch_to_metre(94 + 18),
        inch_to_metre(114 + 18),
        inch_to_metre(134 + 18),
        inch_to_metre(154 + 18),
        inch_to_metre(174 + 18),
        inch_to_metre(194 + 18),
    ]  # metres

    flywheel_speeds = [43.0, 46.0, 47.0, 49.0, 53.0, 55.0, 58.0, 60.0]  # rev/s
    hood_angles = [
        70.0,
        70.0,
        65.0,
        60.0,
        60.0,
        60.0,
        60.0,
        56.0,
    ]  # degrees from horizontal

    min_score_range = ranges[0]
    max_score_range = ranges[-1]

    def __init__(self) -> None:
        self._solution = Solution(flywheel_speed=0.0, hood_angle=0.0)
        self._in_range = False

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
        pose = self.drivetrain.pose()
        # First calculate the distance to the target
        distance_to_hub = pose.translation().distance(hub_position())
        self._in_range = (
            self.min_score_range < distance_to_hub < self.max_score_range
            and is_in_alliance_zone(pose)
        )

        # Keep things ticking over but don't power right up in the neutral zone
        speed = numpy.interp(distance_to_hub, self.ranges, self.flywheel_speeds)
        angle = numpy.interp(distance_to_hub, self.ranges, self.hood_angles)

        self._solution = Solution(flywheel_speed=speed, hood_angle=angle)
