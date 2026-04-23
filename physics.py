import math

import phoenix6
from phoenix6.swerve.sim_swerve_drivetrain import SimSwerveDrivetrain
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from pyfrc.physics.core import PhysicsInterface
from wpilib import DriverStation, RobotController
from wpilib.simulation import DCMotorSim, RoboRioSim
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations

from generated.comp import TunerConstants
from robot import MyRobot
from sysid_robot import SysIdRobot
from utilities import game


class PhysicsEngine:
    def __init__(
        self,
        physics_controller: PhysicsInterface,
        robot: MyRobot | SysIdRobot,
    ) -> None:
        self.robot = robot
        if isinstance(self.robot, SysIdRobot):
            return

        self.physics_controller = physics_controller

        self.roborio = RoboRioSim()

        swerve_constants = TunerConstants()
        module_constants = [
            swerve_constants.front_left,
            swerve_constants.front_right,
            swerve_constants.back_left,
            swerve_constants.back_right,
        ]
        positions = [
            Translation2d(
                mc.location_x,
                mc.location_y,
            )
            for mc in module_constants
        ]
        self.swerve = SimSwerveDrivetrain(
            positions,
            self.robot.drivetrain.pigeon2.sim_state,
            module_constants,
        )

        self.vision_sim = VisionSystemSim("main")
        self.vision_sim.addAprilTags(game.apriltag_layout)
        properties = SimCameraProperties.OV9281_1280_720()
        self.shooter_camera = PhotonCameraSim(robot.shooter_vision.camera, properties)  # type: ignore
        self.shooter_camera.setMaxSightRange(5.0)
        self.red_camera = PhotonCameraSim(robot.red_vision.camera, properties)  # type: ignore
        self.red_camera.setMaxSightRange(5.0)
        self.white_camera = PhotonCameraSim(robot.white_vision.camera, properties)  # type: ignore
        self.white_camera.setMaxSightRange(5.0)
        self.vision_sim.addCamera(
            self.shooter_camera,
            self.robot.shooter_vision_transform,
        )
        self.vision_sim.addCamera(
            self.red_camera,
            self.robot.red_vision_transform,
        )
        self.vision_sim.addCamera(
            self.white_camera,
            self.robot.white_vision_transform,
        )

        gearbox = DCMotor.krakenX60(2)
        self.shooter_motor_sim = DCMotorSim(
            LinearSystemId.DCMotorSystem(gearbox, 0.005, 1.0),
            gearbox,
        )
        # Keep a reference to the motor sim state so we can update it
        self.shooter_talon_sim = self.robot.shooter._shooter_motor.sim_state

        self.pigeon = self.robot.drivetrain.pigeon2.sim_state

    def update_sim(self, now: float, tm_diff: float) -> None:
        if isinstance(self.robot, SysIdRobot):
            return

        # If the driver station is enabled, then feed enable for phoenix devices
        if DriverStation.isEnabled():
            phoenix6.unmanaged.feed_enable(100)

        self.swerve.update(
            tm_diff, self.roborio.getVInVoltage(), self.robot.drivetrain.modules
        )
        states = tuple([m.get_current_state() for m in self.robot.drivetrain.modules])
        # Confirm that we have 4 states in a tuple so type checking by mypy works
        assert len(states) == 4
        assert isinstance(self.robot.drivetrain.kinematics, SwerveDrive4Kinematics)
        speeds = self.robot.drivetrain.kinematics.toChassisSpeeds(states)
        self.pigeon.add_yaw(math.degrees(speeds.omega * tm_diff))
        self.physics_controller.drive(speeds, tm_diff)

        self.vision_sim.update(self.physics_controller.get_pose())

        self.shooter_talon_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.shooter_motor_sim.setInputVoltage(self.shooter_talon_sim.motor_voltage)
        self.shooter_motor_sim.update(tm_diff)
        self.shooter_talon_sim.set_raw_rotor_position(
            radiansToRotations(self.shooter_motor_sim.getAngularPosition())
        )
        self.shooter_talon_sim.set_rotor_velocity(
            radiansToRotations(self.shooter_motor_sim.getAngularVelocity())
        )
