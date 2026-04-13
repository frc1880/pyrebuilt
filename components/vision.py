import math

import wpilib
from magicbot import feedback, tunable
from phoenix6.utils import fpga_to_current_time
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from wpimath.geometry import Pose2d, Transform3d

from components.drivetrain import Drivetrain
from utilities.game import apriltag_layout


class Vision:
    """
    Wrap photonvision and add measurements to the drivebase odometry.
    """

    # We need to access the drivetrain to add measurements
    drivetrain: Drivetrain
    field: wpilib.Field2d
    use_single_tag = tunable(False)

    def __init__(self, camera_name: str, transform: Transform3d) -> None:
        # Instantiate the camera/photonvision
        self.camera = PhotonCamera(camera_name)
        self.estimator = PhotonPoseEstimator(apriltag_layout, transform)

        self._targets: list[PhotonTrackedTarget] = []

        self._has_seen_multitag = False
        self._last_multitag = wpilib.Timer.getFPGATimestamp()

    def setup(self) -> None:
        self._field_obj = self.field.getObject(self.camera.getName() + "_vision")

    @feedback
    def targets(self) -> list[int]:
        ids = [target.fiducialId for target in self._targets]
        ids.sort()
        return ids

    @feedback
    def alive(self) -> bool:
        return wpilib.Timer.getFPGATimestamp() - self._last_multitag < 2.0

    def is_initialized(self) -> bool:
        return self._has_seen_multitag

    def _is_innovation_ok(self, pose: Pose2d) -> bool:
        innovation = pose.translation().distance(self.drivetrain.pose().translation())
        return innovation < 0.5

    def execute(self) -> None:
        if abs(self.drivetrain.velocity_robot().omega) > 0.5:
            return
        # Get any observations from photonvision and add them to the drivetrain
        if self._has_seen_multitag:
            self.estimator.addHeadingData(
                wpilib.Timer.getFPGATimestamp(), self.drivetrain.pose().rotation()
            )
        result = self.camera.getLatestResult()
        self._targets = result.getTargets()
        pose = self.estimator.estimateCoprocMultiTagPose(result)
        if pose:
            # Multitag successful
            self._field_obj.setPose(pose.estimatedPose.toPose2d())
            if not self._has_seen_multitag or self._is_innovation_ok(
                pose.estimatedPose.toPose2d()
            ):
                self.drivetrain.add_vision_measurement(
                    pose.estimatedPose.toPose2d(),
                    fpga_to_current_time(pose.timestampSeconds),
                    (0.05, 0.05, math.radians(3)),
                )
                self._has_seen_multitag = True
                self._last_multitag = wpilib.Timer.getFPGATimestamp()

        elif self._has_seen_multitag and self.use_single_tag:
            # Don't fuse single tags until multitag has given us a proper heading
            # We don't have multitag result, so try single tag
            pose = self.estimator.estimatePnpDistanceTrigSolvePose(result)
            if pose and self._is_innovation_ok(pose.estimatedPose.toPose2d()):
                # Check for innovation and gate
                self.drivetrain.add_vision_measurement(
                    pose.estimatedPose.toPose2d(),
                    fpga_to_current_time(pose.timestampSeconds),
                    (0.2, 0.2, math.radians(5)),
                )
        self.drivetrain.update_odometry()
