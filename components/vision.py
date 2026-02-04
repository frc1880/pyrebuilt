from wpimath.geometry import Translation3d

from components.drivetrain import Drivetrain


class Vision:
    """
    Wrap photonvision and add measurements to the drivebase odometry.
    """

    # We need to access the drivetrain to add measurements
    drivetrain: Drivetrain

    def __init__(self, name: str, translation: Translation3d) -> None:
        # Instantiate the camera/photonvision
        pass

    def execute(self) -> None:
        # Get any observations from photonvision and add them to the drivetrain
        pass
