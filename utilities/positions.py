import math

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import inchesToMeters

from utilities.game import (
    apriltag_layout,
    field_flip_pose2d,
    field_flip_translation2d,
    is_blue,
)


class TeamPoses:
    BLUE_TEST_POSE = Pose2d(3.6, 0.75, math.radians(0.0))
    RED_TEST_POSE = field_flip_pose2d(BLUE_TEST_POSE)


class HubPosition:
    BLUE = Translation2d(
        inchesToMeters(182.11), inchesToMeters(158.84)
    )  # converted from inches on the field drawings
    RED = field_flip_translation2d(BLUE)


class AllianceZone:
    BLUE = inchesToMeters(158.06)
    RED = apriltag_layout.getFieldLength() - BLUE


def hub_position() -> Translation2d:
    if is_blue():
        return HubPosition.BLUE
    else:
        return HubPosition.RED


def shooter_to_hub(robot_pose: Pose2d) -> Rotation2d:
    hub = hub_position()
    desired_heading = (
        math.atan2(
            hub.y - robot_pose.translation().y, hub.x - robot_pose.translation().x
        )
        - math.pi
    )  # Shooter is at rear of robot facing on the -ve x axis

    return Rotation2d(desired_heading)


def is_in_alliance_zone(robot_pose: Pose2d) -> bool:
    # Returns True if in alliance zone.
    robot_x = robot_pose.translation().x

    if is_blue():
        return robot_x <= AllianceZone.BLUE
    else:
        return robot_x >= AllianceZone.RED
