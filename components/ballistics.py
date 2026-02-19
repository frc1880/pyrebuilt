from collections import namedtuple

import numpy
from magicbot import feedback

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

    ranges = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]  # metres
    flywheel_speeds = [50, 60, 70, 80, 90, 100]  # rev/s
    hood_angles = [70.0, 65.0, 60.0, 55.0, 50.0, 45.0]  # degrees from horizontal

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
