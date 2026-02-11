import wpilib
from magicbot import feedback, tunable
from phoenix6.swerve import requests
from phoenix6.swerve.swerve_module import SwerveModule
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import rotationsToRadians

from ids import RioSerialNumber
from utilities import game
from utilities.positions import TeamPoses


class Drivetrain:
    field: wpilib.Field2d

    max_speed = tunable(0.0)
    max_angular_rate = tunable(rotationsToRadians(0.75))

    def __init__(self) -> None:
        if wpilib.RobotController.getSerialNumber() == RioSerialNumber.STUMPY_BOT:
            from generated.stumpy import TunerConstants, TunerSwerveDrivetrain
        elif wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
            from generated.test import (  # type: ignore
                TunerConstants,
                TunerSwerveDrivetrain,
            )
        else:
            from generated.comp import (  # type: ignore
                TunerConstants,
                TunerSwerveDrivetrain,
            )

        tuner_constants = TunerConstants()
        modules = [
            tuner_constants.front_left,
            tuner_constants.front_right,
            tuner_constants.back_left,
            tuner_constants.back_right,
        ]
        self._phoenix_swerve = TunerSwerveDrivetrain(
            tuner_constants.drivetrain_constants, modules
        )
        self.tuner_constants = tuner_constants

        self._field_drive_request = requests.FieldCentric()
        self._robot_drive_request = requests.RobotCentric()

        # Use open-loop control for drive motors
        self._field_drive_request.drive_request_type = (
            SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self._robot_drive_request.drive_request_type = (
            SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        self._request: requests.SwerveRequest = requests.Idle()

        self.on_blue_alliance = game.is_blue()

    def setup(self) -> None:
        self.max_speed = self.tuner_constants.speed_at_12_volts
        # speed_at_12_volts desired top speed
        self.field_obj = self.field.getObject("odometry")
        # pass through required methods from phoenix swerve object
        self.kinematics = self._phoenix_swerve.kinematics
        self.modules = self._phoenix_swerve.modules
        self.pigeon2 = self._phoenix_swerve.pigeon2
        self.get_state = self._phoenix_swerve.get_state
        self.add_vision_measurement = self._phoenix_swerve.add_vision_measurement
        self.set_state_std_devs = self._phoenix_swerve.set_state_std_devs
        self.set_vision_measurement_std_devs = (
            self._phoenix_swerve.set_vision_measurement_std_devs
        )

        self.set_pose(
            TeamPoses.BLUE_TEST_POSE if game.is_blue() else TeamPoses.RED_TEST_POSE
        )

    def on_enable(self) -> None:
        self._phoenix_swerve.set_operator_perspective_forward(
            Rotation2d.fromDegrees(0) if game.is_blue() else Rotation2d.fromDegrees(180)
        )
        self.update_alliance()

    @feedback
    def roborio_serial(self) -> str:
        return wpilib.RobotController.getSerialNumber()

    def update_alliance(self) -> None:
        # Check whether our alliance has "changed"
        # If so, it means we have an update from the FMS and need to re-init the odom
        if game.is_blue() != self.on_blue_alliance:
            self.on_blue_alliance = game.is_blue()
            # TODO update with new game info
            if self.on_blue_alliance:
                self.set_pose(TeamPoses.BLUE_TEST_POSE)
            else:
                self.set_pose(TeamPoses.RED_TEST_POSE)

    def set_pose(self, pose: Pose2d) -> None:
        self._phoenix_swerve.reset_pose(pose)
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)

    def drive_field(self, vx: float, vy: float, vz: float) -> None:
        self._set_request_velocities(self._field_drive_request, vx, vy, vz)

    def drive_robot(self, vx: float, vy: float, vz: float) -> None:
        self._set_request_velocities(self._robot_drive_request, vx, vy, vz)

    def _set_request_velocities(
        self,
        request: requests.FieldCentric | requests.RobotCentric,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        request.velocity_x = vx
        request.velocity_y = vy
        request.rotational_rate = vz
        # 10% deadband
        request.deadband = self.max_speed * 0.02
        request.rotational_deadband = self.max_angular_rate * 0.02

        self.set_control(request)

    def set_control(self, request: requests.SwerveRequest) -> None:
        self._request = request

    def execute(self) -> None:
        self._phoenix_swerve.set_control(self._request)
        self.field_obj.setPose(self._phoenix_swerve.get_state().pose)
