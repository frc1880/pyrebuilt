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
    dx = -8.225 * 0.0254  # shooter offset in CAD
    dy = 8.527 * 0.0254
    sin = robot_pose.rotation().sin()
    cos = robot_pose.rotation().cos()
    shooter_offset_in_world = Translation2d(cos * dx - sin * dy, sin * dx + cos * dy)
    shooter_position = robot_pose.translation() + shooter_offset_in_world
    hub = hub_position()
    # The desired heading is for the shooter where it is now, and it will move as we rotate
    # This will still converge because we keep updating the setpoint as the shooter moves around
    desired_heading = (
        math.atan2(hub.y - shooter_position.y, hub.x - shooter_position.x) - math.pi
    )  # Shooter is at rear of robot facing on the -ve x axis

    return Rotation2d(desired_heading)


def is_in_alliance_zone(robot_pose: Pose2d) -> bool:
    # Returns True if in alliance zone.
    robot_x = robot_pose.translation().x

    if is_blue():
        return robot_x <= AllianceZone.BLUE
    else:
        return robot_x >= AllianceZone.RED
