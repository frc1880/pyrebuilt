from typing import NamedTuple

import numpy
from magicbot import feedback, will_reset_to

from components.drivetrain import Drivetrain
from utilities.conversion import inch_to_metre
from utilities.positions import hub_position


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

    flywheel_speeds = [43, 46, 47, 49, 53, 55, 58, 60]  # rev/s
    hood_angles = [70.0, 70, 65.0, 60.0, 60.0, 60.0, 60, 56]  # degrees from horizontal

    _should_dump = will_reset_to(False)

    def __init__(self) -> None:
        self._solution = Solution(flywheel_speed=0.0, hood_angle=0.0)

    def dump(self) -> None:
        self._should_dump = True

    @feedback
    def solution(self) -> Solution:
        return self._solution

    def execute(self) -> None:
        """
        Calculate the required speed and angle and store it in the _solution variable.
        """
        # First calculate the distance to the target
        distance_to_hub = self.drivetrain.pose().translation().distance(hub_position())

        if self._should_dump:
            # Shoot across field to move fuel to our alliance zone
            speed = 90.0
            angle = 45.0
        else:
            # Proper shot into hub
            speed = numpy.interp(distance_to_hub, self.ranges, self.flywheel_speeds)
            angle = numpy.interp(distance_to_hub, self.ranges, self.hood_angles)

        self._solution = Solution(flywheel_speed=speed, hood_angle=angle)
