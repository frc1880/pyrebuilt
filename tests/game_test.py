from wpimath.geometry import Pose2d, Rotation2d

from utilities import game
from utilities.game import ShiftInfo, _shift_info_with_args


def test_hub_active() -> None:
    # In transition both are active
    assert _shift_info_with_args(135, True) == ShiftInfo(True, 5)
    assert _shift_info_with_args(135, False) == ShiftInfo(True, 5)

    # First shift
    assert _shift_info_with_args(115, True) == ShiftInfo(False, 10)
    assert _shift_info_with_args(115, False) == ShiftInfo(True, 10)

    # Second shift
    assert _shift_info_with_args(90, True) == ShiftInfo(True, 10)
    assert _shift_info_with_args(90, False) == ShiftInfo(False, 10)

    # Third shift
    assert _shift_info_with_args(65, True) == ShiftInfo(False, 10)
    assert _shift_info_with_args(65, False) == ShiftInfo(True, 10)

    # Fourth shift
    assert _shift_info_with_args(40, True) == ShiftInfo(True, 10)
    assert _shift_info_with_args(40, False) == ShiftInfo(False, 10)

    # End game both active
    assert _shift_info_with_args(20, True) == ShiftInfo(True, 20)
    assert _shift_info_with_args(20, False) == ShiftInfo(True, 20)


def test_field_flip() -> None:
    p = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))
    flipped = game.field_flip_pose2d(p)
    mirrored = game.field_mirror_pose2d(p)
    assert abs((flipped.rotation() - Rotation2d.fromDegrees(180.0)).degrees()) < 0.5
    assert abs((mirrored.rotation() - Rotation2d.fromDegrees(0.0)).degrees()) < 0.5

    p = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0))
    flipped = game.field_flip_pose2d(p)
    mirrored = game.field_mirror_pose2d(p)
    assert abs((flipped.rotation() - Rotation2d.fromDegrees(-90.0)).degrees()) < 0.5
    assert abs((mirrored.rotation() - Rotation2d.fromDegrees(-90.0)).degrees()) < 0.5

    p = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45.0))
    flipped = game.field_flip_pose2d(p)
    mirrored = game.field_mirror_pose2d(p)
    assert abs((flipped.rotation() - Rotation2d.fromDegrees(-135.0)).degrees()) < 0.5
    assert abs((mirrored.rotation() - Rotation2d.fromDegrees(-45.0)).degrees()) < 0.5
