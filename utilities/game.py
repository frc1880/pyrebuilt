import robotpy_apriltag
import wpilib
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


def is_blue() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue


def is_red() -> bool:
    return not is_blue()


apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
    robotpy_apriltag.AprilTagField.k2026RebuiltWelded
)


def is_hub_active() -> bool:
    return time_to_hub_active() == 0


def _time_to_hub_active_with_args(
    match_time: float, is_auto_winning_alliance: bool
) -> float:
    if match_time > 130:
        # Transition shift
        return 0
    elif match_time > 105:
        # Shift 1
        return 0 if is_auto_winning_alliance else match_time - 105
    elif match_time > 80:
        # Shift 2
        return 0 if not is_auto_winning_alliance else match_time - 80
    elif match_time > 55:
        # Shift 3
        return 0 if is_auto_winning_alliance else match_time - 55
    elif match_time > 30:
        # Shift 4
        return 0 if not is_auto_winning_alliance else match_time - 30
    else:
        # End game
        return 0


def time_to_hub_active() -> float:
    # Use the Game Data documented here to determine if we are active:
    # https://frc-docs--3246.org.readthedocs.build/en/3246/docs/yearly-overview/2026-game-data.html#c-java-python
    # Returns True if alliance hub is active.

    # HUB always active during AUTO
    # Also enable in test mode
    if DriverStation.isAutonomousEnabled() or DriverStation.isTestEnabled():
        return 0

    match_time = DriverStation.getMatchTime()
    game_data = DriverStation.getGameSpecificMessage()

    # Game data gives the first hub to go *inactive* ie the losing alliance
    if game_data == "R":
        is_winning_alliance = is_blue()
    elif game_data == "B":
        is_winning_alliance = is_red()
    else:
        return 0

    return _time_to_hub_active_with_args(match_time, is_winning_alliance)


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