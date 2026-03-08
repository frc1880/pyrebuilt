from collections import namedtuple

import numpy
from magicbot import feedback
from utilities.conversion import inch_to_metre

from components.drivetrain import Drivetrain
from utilities.positions import hub_position

Solution = namedtuple("Solution", ("flywheel_speed", "hood_angle"))


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

    flywheel_speeds = [43, 46, 47, 49, 53, 55, 58, 60]  # rev/s
    hood_angles = [70.0, 70, 65.0, 60.0, 60.0, 60.0, 60, 56]  # degrees from horizontal

    def __init__(self) -> None:
        self._solution = Solution(flywheel_speed=0.0, hood_angle=0.0)

    @feedback
    def solution(self) -> Solution:
        return self._solution

    def execute(self) -> None:
        """
        Calculate the required speed and angle and store it in the _solution variable.
        """
        # First calculate the distance to the target
        distance_to_hub = self.drivetrain.pose().translation().distance(hub_position())

        speed = numpy.interp(distance_to_hub, self.ranges, self.flywheel_speeds)
        angle = numpy.interp(distance_to_hub, self.ranges, self.hood_angles)

        self._solution = Solution(flywheel_speed=speed, hood_angle=angle)
