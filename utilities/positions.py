import math

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from utilities.game import field_flip_pose2d, field_flip_translation2d, is_blue


class TeamPoses:
    BLUE_TEST_POSE = Pose2d(3.5, 5.0, math.radians(180.0))
    RED_TEST_POSE = field_flip_pose2d(BLUE_TEST_POSE)


class HubPosition:
    BLUE = Translation2d(
        182.11 * 25.4 / 1000, 158.84 * 25.4 / 1000
    )  # converted from inches on the field drawings
    RED = field_flip_translation2d(BLUE)


def hub_position() -> Translation2d:
    if is_blue():
        return HubPosition.BLUE
    else:
        return HubPosition.RED


def bearing_to_hub(pose: Translation2d) -> Rotation2d:
    hub = hub_position()
    return Rotation2d(math.atan2(hub.y - pose.y, hub.x - pose.x))
