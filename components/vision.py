from photonlibpy import PhotonCamera, PhotonPoseEstimator
from wpimath.geometry import Transform3d

from components.drivetrain import Drivetrain
from utilities.game import apriltag_layout


class Vision:
    """
    Wrap photonvision and add measurements to the drivebase odometry.
    """

    # We need to access the drivetrain to add measurements
    drivetrain: Drivetrain

    def __init__(self, camera_name: str, transform: Transform3d) -> None:
        # Instantiate the camera/photonvision
        self.camera = PhotonCamera(camera_name)

        self.estimator = PhotonPoseEstimator(apriltag_layout, transform)

    def execute(self) -> None:
        # Get any observations from photonvision and add them to the drivetrain
        for result in self.camera.getAllUnreadResults():
            pose = self.estimator.estimateCoprocMultiTagPose(result)
            if pose is None:
                pose = self.estimator.estimateLowestAmbiguityPose(result)
            if pose is not None:
                self.drivetrain.add_vision_measurement(
                    pose.estimatedPose.toPose2d(), pose.timestampSeconds
                )
