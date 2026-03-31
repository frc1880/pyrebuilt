from typing import NamedTuple

import robotpy_apriltag
import wpilib
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


class ShiftInfo(NamedTuple):
    hub_active: bool
    time_remaining: float


def is_blue() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue


def is_red() -> bool:
    return not is_blue()


apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
    robotpy_apriltag.AprilTagField.k2026RebuiltWelded
)


def is_hub_active() -> bool:
    # Allow shooting early because fuel is in flight for a second
    info = shift_info()
    return info.hub_active or info.time_remaining <= 1


def _shift_info_with_args(
    match_time: float, is_auto_winning_alliance: bool
) -> ShiftInfo:
    if match_time > 130:
        # Transition shift
        return ShiftInfo(True, match_time - 130)
    elif match_time > 105:
        # Shift 1
        # Winning alliance goes *inactive* first
        return ShiftInfo(not is_auto_winning_alliance, match_time - 105)
    elif match_time > 80:
        # Shift 2
        return ShiftInfo(is_auto_winning_alliance, match_time - 80)
    elif match_time > 55:
        # Shift 3
        return ShiftInfo(not is_auto_winning_alliance, match_time - 55)
    elif match_time > 30:
        # Shift 4
        return ShiftInfo(is_auto_winning_alliance, match_time - 30)
    else:
        # End game
        return ShiftInfo(True, match_time)


def is_auto_winner() -> bool | None:
    game_data = DriverStation.getGameSpecificMessage()

    # Game data gives the first hub to go *inactive* ie the winning alliance
    if game_data == "R":
        return is_red()
    elif game_data == "B":
        return is_blue()
    else:
        # No game data yet, or invalid
        return None


def shift_info() -> ShiftInfo:
    # Use the Game Data documented here to determine if we are active:
    # https://frc-docs--3246.org.readthedocs.build/en/3246/docs/yearly-overview/2026-game-data.html#c-java-python
    # Returns True if alliance hub is active.

    # HUB always active during AUTO
    # Also enable in test mode
    if DriverStation.isAutonomousEnabled() or DriverStation.isTestEnabled():
        return ShiftInfo(True, 999)

    match_time = DriverStation.getMatchTime()

    # Game data gives the first hub to go *inactive* ie the winning alliance
    is_winning_alliance = is_auto_winner()
    if is_winning_alliance is None:
        return ShiftInfo(True, 999)

    return _shift_info_with_args(match_time, is_winning_alliance)


def field_flip_pose2d(p: Pose2d) -> Pose2d:
    return Pose2d(
        field_flip_translation2d(p.translation()),
        field_flip_rotation2d(p.rotation()),
    )


def field_flip_rotation2d(r: Rotation2d) -> Rotation2d:
    return Rotation2d(-r.cos(), r.sin())


def field_flip_translation2d(t: Translation2d) -> Translation2d:
    return Translation2d(
        apriltag_layout.getFieldLength() - t.x, apriltag_layout.getFieldWidth() - t.y
    )


def field_mirror_translation2d(t: Translation2d) -> Translation2d:
    return Translation2d(t.x, apriltag_layout.getFieldWidth() - t.y)


def field_mirror_rotation2d(r: Rotation2d) -> Rotation2d:
    return Rotation2d(r.cos(), r.sin())


def field_mirror_pose2d(p: Pose2d) -> Pose2d:
    return Pose2d(
        field_mirror_translation2d(p.translation()),
        field_mirror_rotation2d(p.rotation()),
    )
