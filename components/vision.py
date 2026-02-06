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
        self.cam = PhotonCamera("Front Camera")
        kRobotToCam = wpimath.geometry.Transform3d(
            wpimath.geometry.Translation3d(0.5, 0.0, 0.5),
            wpimath.geometry.Rotation3d.fromDegrees(0.0, -30.0, 0.0),
        )       
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            kRobotToCam,
        )



    def execute(self) -> None:
            # Get any observations from photonvision and add them to the drivetrain
        for result in self.cam.getAllUnreadResults():
            camEstPose = self.camPoseEst.estimateCoprocMultiTagPose(result)
            if camEstPose is None:
                camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(result)
