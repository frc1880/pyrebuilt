import robotpy_apriltag
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import DriverStation


def is_blue() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue


apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
    robotpy_apriltag.AprilTagField.k2026RebuiltWelded
)


def is_hub_active() -> bool:
    # Use the Game Data documented here to determine if we are active:
    # https://frc-docs--3246.org.readthedocs.build/en/3246/docs/yearly-overview/2026-game-data.html#c-java-python
    # Returns True if alliance hub is active. 

    alliance = DriverStation.getAlliance()

    # Team not in match
    if alliance is None:
        return False
    
    # HUB always active during AUTO
    if DriverStation.isAutonomousEnabled():
        return True

    # If not Teleop, dont compute
    if not DriverStation.isTeleopEnabled():
        return False

    match_time = DriverStation.getMatchTime()  
    game_data = DriverStation.getGameSpecificMessage()

    
    if game_data == "R":
        red_inactive_first = True
    elif game_data == "B":
        red_inactive_first = False
    else:
        return True

    # Check which alliance hub is Active. If R inactive, B starts S1
    shift1_active = (
        not red_inactive_first) if alliance == DriverStation.Alliance.kRed else red_inactive_first

    
    if match_time > 130:
        # Transition shift
        return True          
    elif match_time > 105:
        # Shift 1
        return shift1_active 
    elif match_time > 80:
        # Shift 2
        return not shift1_active  
    elif match_time > 55:
        # Shift 3
        return shift1_active 
    elif match_time > 30:
        # Shift 4
        return not shift1_active  
    else:
        # End game
        return True          



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
