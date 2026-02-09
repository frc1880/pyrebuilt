import robotpy_apriltag
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


def is_blue() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue


apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
    robotpy_apriltag.AprilTagField.k2026RebuiltWelded
)


def is_hub_active() -> bool:
    # Use the Game Data documented here to determine if we are active:
    # https://frc-docs--3246.org.readthedocs.build/en/3246/docs/yearly-overview/2026-game-data.html#c-java-python
    return False


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
