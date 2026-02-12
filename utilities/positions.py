from wpimath.geometry import Pose2d, Translation2d

from utilities.game import field_flip_pose2d, field_flip_translation2d, is_blue


class TeamPoses:
    BLUE_TEST_POSE = Pose2d(3.5, 5.0, 0.0)
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
