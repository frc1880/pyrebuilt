# TODO Remove noqa when code is working
from photonlibpy import PhotonCamera, PhotonPoseEstimator  # noqa: F401
from wpimath.geometry import Transform3d

from components.drivetrain import Drivetrain
from utilities.game import apriltag_layout  # noqa: F401


class Vision:
    """
    Wrap photonvision and add measurements to the drivebase odometry.
    """

    # We need to access the drivetrain to add measurements
    drivetrain: Drivetrain

    def __init__(self, camera_name: str, transform: Transform3d) -> None:
        # Instantiate the camera/photonvision
        pass

    def execute(self) -> None:
        # Get any observations from photonvision and add them to the drivetrain
        pass
