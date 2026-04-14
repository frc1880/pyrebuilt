import math
from dataclasses import dataclass

import numpy as np
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from utilities import game


@dataclass
class MotionParameters:
    translation_tolerance: float
    rotation_tolerance: float
    max_linear_speed: float
    max_linear_acceleration: float
    max_angular_speed: float
    max_angular_acceleration: float


@dataclass
class Waypoint:
    translation: Translation2d
    rotation: Rotation2d | None = None


def cumulative_distance(subpath: list[Waypoint] | list[Pose2d]) -> float:
    d = 0.0
    for wp1, wp2 in zip(subpath, subpath[1:]):
        if isinstance(wp1, Waypoint) and isinstance(wp2, Waypoint):
            d += wp1.translation.distance(wp2.translation)
        elif isinstance(wp1, Pose2d) and isinstance(wp2, Pose2d):
            d += wp1.translation().distance(wp2.translation())
    return d


def unwind(rotation: Rotation2d) -> Rotation2d:
    r = rotation.radians()
    if r > math.pi:
        r -= math.tau
    if r < -math.pi:
        r += math.tau
    return Rotation2d(r)


def slerp(r1: Rotation2d, r2: Rotation2d, t: float) -> Rotation2d:
    t = np.clip(t, 0.0, 1.0)
    delta = unwind(r2 - r1)
    return r1 + (delta * t)


def crosstrack_error(
    position: Translation2d, wp1: Translation2d, wp2: Translation2d
) -> float:
    # Positive to right of path, negative to the left
    segment = wp2 - wp1

    position_from_start = position - wp1
    t: float = np.clip(
        (position_from_start.X() * segment.X() + position_from_start.Y() * segment.Y())
        / segment.squaredNorm(),
        0.0,
        1.0,
    )

    closest_point = wp1 + segment * t

    cross_product = (
        segment.X() * position_from_start.Y() - segment.Y() * position_from_start.X()
    )

    d = (position - closest_point).norm()

    return d if cross_product > 0 else -d


class BLine:
    """
    Point-to-point path follower based on Java BLine library
    https://github.com/edanliahovetsky/BLine-Lib
    """

    def __init__(
        self,
        translation_controller: PIDController,
        rotation_controller: PIDController,
        cross_track_controller: PIDController,
        motion_parameters: MotionParameters,
    ) -> None:
        self._translation_controller = translation_controller
        self._rotation_controller = rotation_controller
        self._cross_track_controller = cross_track_controller
        self._motion_parameters = motion_parameters
        self._translation_controller.setTolerance(
            motion_parameters.translation_tolerance
        )
        self._cross_track_controller.setTolerance(
            motion_parameters.translation_tolerance
        )
        self._rotation_controller.setTolerance(motion_parameters.rotation_tolerance)
        self._rotation_controller.enableContinuousInput(-math.pi, math.pi)
        self._translation_controller.setSetpoint(0.0)
        self._cross_track_controller.setSetpoint(0.0)
        self._current_idx: int = 0
        self._waypoints: list[Pose2d] = []
        self._last_command = ChassisSpeeds()

    def is_at_goal(self) -> bool:
        return (
            self._current_idx == len(self._waypoints) - 1
            and self._translation_controller.atSetpoint()
            and self._cross_track_controller.atSetpoint()
            and self._rotation_controller.atSetpoint()
        )

    def waypoints(self) -> list[Pose2d]:
        return self._waypoints

    def set_path(
        self,
        current_pose: Pose2d,
        waypoints: list[Waypoint],
        should_flip: bool = False,
        should_mirror: bool = False,
    ) -> None:
        self._waypoints = []

        flipped_waypoints: list[Waypoint] = []
        for wp in waypoints:
            t = wp.translation
            if should_mirror:
                t = game.field_mirror_translation2d(t)
            if should_flip:
                t = game.field_flip_translation2d(t)

            r = wp.rotation
            if r:
                if should_mirror:
                    r = game.field_mirror_rotation2d(r)
                if should_flip:
                    r = game.field_flip_rotation2d(r)
            flipped_waypoints.append(Waypoint(t, r))

        # Treat the current pose as the first waypoint to make things easier
        flipped_waypoints = [
            Waypoint(current_pose.translation(), current_pose.rotation())
        ] + flipped_waypoints

        # Interpolate all the rotations to make life easier
        # Forward pass to make sure we have a final rotation
        r = current_pose.rotation()
        waypoints_with_rotations = []
        for idx, wp in enumerate(flipped_waypoints):
            if wp.rotation:
                r = wp.rotation
                waypoints_with_rotations.append(idx)
        if not flipped_waypoints[-1].rotation:
            flipped_waypoints[-1].rotation = r
            waypoints_with_rotations.append(len(flipped_waypoints) - 1)

        for start_idx, end_idx in zip(
            waypoints_with_rotations, waypoints_with_rotations[1:]
        ):
            if end_idx - start_idx == 1:
                # Consecutive waypoints with rotations, so no interpolation required
                continue
            start_rotation = flipped_waypoints[start_idx].rotation
            end_rotation = flipped_waypoints[end_idx].rotation
            assert start_rotation
            assert end_rotation

            total_d = cumulative_distance(flipped_waypoints[start_idx : end_idx + 1])
            for idx in range(start_idx + 1, end_idx):
                cum_d = cumulative_distance(flipped_waypoints[start_idx : idx + 1])
                flipped_waypoints[idx].rotation = slerp(
                    start_rotation, end_rotation, cum_d / total_d
                )

        for wp in flipped_waypoints:
            assert wp.rotation
            self._waypoints.append(Pose2d(wp.translation, wp.rotation))

        self._current_idx = 1
        # First waypoint is the starting pose

        self._last_command = ChassisSpeeds()

    def _distance_remaining(self, pose: Pose2d) -> float:
        d = pose.translation().distance(
            self._waypoints[self._current_idx].translation()
        )
        d += cumulative_distance(self._waypoints[self._current_idx :])
        return d

    def calculate(self, pose: Pose2d, dt=0.02) -> ChassisSpeeds:
        if len(self._waypoints) == 0:
            self._last_command = ChassisSpeeds()
            return ChassisSpeeds()

        # Advance waypoints if required
        while True:
            if (
                pose.translation().distance(
                    self._waypoints[self._current_idx].translation()
                )
                < self._motion_parameters.translation_tolerance
                and self._current_idx < len(self._waypoints) - 1
            ):
                self._current_idx += 1
            else:
                break
        self._current_idx = min(self._current_idx, len(self._waypoints) - 1)

        current_wp = self._waypoints[self._current_idx]
        prev_wp = self._waypoints[self._current_idx - 1]

        # Compute translation command vector
        delta = current_wp.translation() - pose.translation()
        direction: Translation2d = (
            delta
            / delta.norm()
            * np.clip(
                -self._translation_controller.calculate(self._distance_remaining(pose)),
                -self._motion_parameters.max_linear_speed,
                self._motion_parameters.max_linear_speed,
            )
        )
        # This is clamped so that the cross track error can be effective when we are a long way from the end of the path

        segment = current_wp.translation() - prev_wp.translation()
        cross_track = (segment / segment.norm()).rotateBy(
            Rotation2d.fromDegrees(90.0)
        ) * self._cross_track_controller.calculate(
            crosstrack_error(
                pose.translation(), prev_wp.translation(), current_wp.translation()
            )
        )
        # cross_track = (segment / segment.norm()).rotateBy(
        #     Rotation2d.fromDegrees(90.0)
        # ) * self._cross_track_controller.calculate(
        #     crosstrack_error(
        #         pose.translation(), prev_wp.translation(), current_wp.translation()
        #     )
        # )
        translation = direction + cross_track

        # Compute rotation command
        segment_distance = (
            current_wp.translation().distance(prev_wp.translation())
            - self._motion_parameters.translation_tolerance
        )
        t = np.clip(1.0 - delta.norm() / segment_distance, 0.0001, 1.0)
        self._rotation_controller.setSetpoint(
            slerp(prev_wp.rotation(), current_wp.rotation(), t).radians()
        )
        omega = np.clip(
            self._rotation_controller.calculate(pose.rotation().radians()),
            -self._motion_parameters.max_angular_speed,
            self._motion_parameters.max_angular_speed,
        )

        # Apply velocity/acceleration limits
        v = translation.norm()
        if v > self._motion_parameters.max_linear_speed:
            translation = translation / v * self._motion_parameters.max_linear_speed

        if self._motion_parameters.max_linear_acceleration > 0.0:
            current_vel = Translation2d(self._last_command.vx, self._last_command.vy)
            desired_accel = (translation - current_vel) / dt
            if desired_accel.norm() > self._motion_parameters.max_linear_acceleration:
                desired_accel = (
                    desired_accel
                    / desired_accel.norm()
                    * self._motion_parameters.max_linear_acceleration
                )
                translation = current_vel + desired_accel * dt
        if self._motion_parameters.max_angular_acceleration > 0.0:
            current_omega = self._last_command.omega
            desired_accel = np.clip(
                (omega - current_omega) / dt,
                -self._motion_parameters.max_angular_acceleration,
                self._motion_parameters.max_angular_acceleration,
            )
            omega = current_omega + desired_accel * dt

        vx = translation.X()
        vy = translation.Y()

        self._last_command = ChassisSpeeds(vx, vy, omega)
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            self._last_command, pose.rotation()
        )
