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
        self.frontcam = PhotonCamera("Front Camera")
        kRobotToCam = wpimath.geometry.Transform3d(
            wpimath.geometry.Translation3d(0.5, 0.0, 0.5),
            wpimath.geometry.Rotation3d.fromDegrees(0.0, -30.0, 0.0),
        )       
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            kRobotToCam,
        )
        self.turretcam= = PhotonCamera("Turret Camera")


    def execute(self) -> None:
            # Get any observations from photonvision and add them to the drivetrain
        for result in self.frontcam.getAllUnreadResults():
            camEstPose = self.camPoseEst.estimateCoprocMultiTagPose(result)
            if camEstPose is None:
                camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(result)
            self.drivetrain.addVisionPoseEstimate(
                camEstPose.estimatedPose, camEstPose.timestampSeconds
            )

        results = self.turretcam.getLatestResult()
        if (result.hasTargets()):
            best= result.getBestTarget()
            # check if scoring april tag
            # update yaw from scoring tag