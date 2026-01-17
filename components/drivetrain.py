from phoenix6.swerve import requests
from generated.tuner_constants import TunerSwerveDrivetrain, TunerConstants


class Drivetrain:

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

        self._request = requests.Idle()

    def set_control(self, request: requests.SwerveRequest):
        self._request = request

    def execute(self) -> None:
        self._phoenix_swerve.set_control(self._request)
