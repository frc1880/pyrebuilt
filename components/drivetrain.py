from typing import Any

import wpilib
from magicbot import tunable
from phoenix6.hardware.pigeon2 import Pigeon2
from phoenix6.swerve import requests
from phoenix6.swerve.swerve_drivetrain import SwerveDrivetrain
from phoenix6.swerve.swerve_module import SwerveModule
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.units import rotationsToRadians

from generated.tuner_constants import TunerConstants, TunerSwerveDrivetrain
from utilities import game


class Drivetrain:
    field: wpilib.Field2d

    max_speed = tunable(0.0)
    max_angular_rate = tunable(rotationsToRadians(0.75))

    def __init__(self) -> None:
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

    def setup(self) -> None:
        self.max_speed = TunerConstants.speed_at_12_volts
        # speed_at_12_volts desired top speed
        self.field_obj = self.field.getObject("odometry")

    def on_enable(self) -> None:
        self._phoenix_swerve.set_operator_perspective_forward(
            Rotation2d.fromDegrees(0) if game.is_blue() else Rotation2d.fromDegrees(180)
        )

    @property
    def modules(self) -> list[SwerveModule[Any, Any, Any]]:
        return self._phoenix_swerve.modules

    @property
    def pigeon2(self) -> Pigeon2:
        return self._phoenix_swerve.pigeon2

    @property
    def kinematics(
        self,
    ) -> SwerveDrive4Kinematics:
        assert isinstance(self._phoenix_swerve.kinematics, SwerveDrive4Kinematics)
        return self._phoenix_swerve.kinematics

    def get_state(self) -> SwerveDrivetrain.SwerveDriveState:
        return self._phoenix_swerve.get_state()

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

    def set_control(self, request: requests.SwerveRequest):
        self._request = request

    def execute(self) -> None:
        self._phoenix_swerve.set_control(self._request)
        self.field_obj.setPose(self._phoenix_swerve.get_state().pose)
