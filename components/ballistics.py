from collections import namedtuple

import numpy
from magicbot import feedback
from wpimath.geometry import Pose2d

from components.drivetrain import Drivetrain
from utilities.game import is_in_alliance_zone
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

    min_score_range = ranges[0]
    max_score_range = ranges[-1]

    def __init__(self) -> None:
        self._solution = Solution(flywheel_speed=0.0, hood_angle=0.0)
        self._distance_to_hub = 0.0
        self._in_range = False

    @feedback
    def solution(self) -> Solution:
        return self._solution

    @feedback
    def distance_to_hub(self) -> float:
        return self._distance_to_hub

    def robot_pose(self) -> Pose2d:
        return self.drivetrain.current_pose()

    def calc_hub_distance(self) -> float:
        robot_pose = self.robot_pose().translation()
        hub_pos = hub_position()
        return robot_pose.distance(hub_pos)

    @feedback
    def is_in_range_to_score(self):
        """
        Returns True if current robot-to-hub distance is inside the table.
        """
        d = self._distance_to_hub

        # Must be within table
        if d < self.min_score_range or (
            d > self.max_score_range and is_in_alliance_zone
        ):
            return False

    def execute(self) -> None:
        """
        Calculate the required speed and angle and store it in the _solution variable.
        """
        # First calculate the distance to the target

        self._distance_to_hub = self.calc_hub_distance()
        self._in_range = self.is_in_range_to_score()

        speed = numpy.interp(self._distance_to_hub, self.ranges, self.flywheel_speeds)
        angle = numpy.interp(self._distance_to_hub, self.ranges, self.hood_angles)

        self._solution = Solution(flywheel_speed=speed, hood_angle=angle)
