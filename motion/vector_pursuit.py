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
    max_linear_acceleration: float | None = None
    max_angular_speed: float | None = None


@dataclass
class PathPoint:
    translation: Translation2d
    rotation: Rotation2d | None = None
    speed: float = 0.0


@dataclass
class SegmentPoint:
    pose: Pose2d
    speed: float = 0.0
    tolerance: float = 0.0


def cumulative_distance(subpath: list[PathPoint] | list[Pose2d]) -> float:
    d = 0.0
    for wp1, wp2 in zip(subpath, subpath[1:]):
        if isinstance(wp1, PathPoint) and isinstance(wp2, PathPoint):
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


def parametric_distance(
    position: Translation2d, segment: tuple[Translation2d, Translation2d]
) -> float:
    delta = segment[1] - segment[0]

    position_from_start = position - segment[0]
    return (
        position_from_start.X() * delta.X() + position_from_start.Y() * delta.Y()
    ) / delta.squaredNorm()


def crosstrack_error(
    position: Translation2d, segment: tuple[Translation2d, Translation2d]
) -> float:
    # Positive to right of path, negative to the left
    delta = segment[1] - segment[0]

    position_from_start = position - segment[0]
    t = parametric_distance(position, segment)

    closest_point = segment[0] + delta * t

    cross_product = (
        delta.X() * position_from_start.Y() - delta.Y() * position_from_start.X()
    )

    d = (position - closest_point).norm()

    return d if cross_product > 0 else -d


class VectorPursuitController:
    """
    Point-to-point path follower based on vector pursuit algorithm.
    Align motion with segment direction, and account for cross-track error.
    """

    def __init__(
        self,
        rotation_controller: PIDController,
        cross_track_controller: PIDController,
        motion_parameters: MotionParameters,
    ) -> None:
        # Get a decent gain for the PID controller based on max accel and speed
        if motion_parameters.max_linear_acceleration:
            s = motion_parameters.max_linear_speed**2 / (
                2.0 * motion_parameters.max_linear_acceleration
            )
            self._translation_controller = PIDController(
                Kp=motion_parameters.max_linear_speed / s, Ki=0.0, Kd=0.0
            )
        else:
            self._translation_controller = PIDController(
                Kp=motion_parameters.max_linear_speed, Ki=0.0, Kd=0.0
            )
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
        self._translation_controller.setTolerance(
            motion_parameters.translation_tolerance
        )
        self._cross_track_controller.setTolerance(
            motion_parameters.translation_tolerance
        )
        self._cross_track_controller.setSetpoint(0.0)
        self._current_idx: int = 0
        self._segment_points: list[SegmentPoint] = []
        self._last_command = ChassisSpeeds()

    def is_at_goal(self) -> bool:
        return (
            self._current_idx == len(self._segment_points) - 1
            and self._translation_controller.atSetpoint()
            and self._cross_track_controller.atSetpoint()
            and self._rotation_controller.atSetpoint()
        )

    def waypoints(self) -> list[Pose2d]:
        return [sp.pose for sp in self._segment_points]

    def set_path(
        self,
        current_pose: Pose2d,
        path_points: list[PathPoint],
        should_flip: bool = False,
        should_mirror: bool = False,
    ) -> None:
        self._segment_points = []

        flipped_waypoints: list[PathPoint] = []
        for wp in path_points:
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
            flipped_waypoints.append(PathPoint(t, r))

        # Treat the current pose as the first waypoint to make things easier
        flipped_waypoints = [
            PathPoint(current_pose.translation(), current_pose.rotation())
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

            total_d = (
                cumulative_distance(flipped_waypoints[start_idx : end_idx + 1])
                - self._motion_parameters.translation_tolerance
            )
            for idx in range(start_idx + 1, end_idx):
                cum_d = cumulative_distance(flipped_waypoints[start_idx : idx + 1])
                flipped_waypoints[idx].rotation = slerp(
                    start_rotation, end_rotation, cum_d / total_d
                )

        for wp in flipped_waypoints:
            assert wp.rotation
            self._segment_points.append(
                SegmentPoint(
                    Pose2d(wp.translation, wp.rotation),
                    wp.speed
                    if wp.speed > 0.0
                    else self._motion_parameters.max_linear_speed,
                    self._motion_parameters.translation_tolerance,
                )
            )

        # Calculate tolerances if required
        if self._motion_parameters.max_angular_speed:
            for start, mid, end in zip(
                self._segment_points, self._segment_points[1:], self._segment_points[2:]
            ):
                s1 = mid.pose.translation() - start.pose.translation()
                s2 = end.pose.translation() - mid.pose.translation()
                turning_radius = mid.speed / self._motion_parameters.max_angular_speed
                turning_angle = unwind(s2.angle() - s1.angle()).radians()
                offset_distance = turning_radius * abs(math.tan(turning_angle / 2.0))
                if offset_distance > mid.tolerance:
                    mid.tolerance = offset_distance

        self._current_idx = 1
        # First waypoint is the starting pose

        self._last_command = ChassisSpeeds()

    def _distance_remaining(self, pose: Pose2d) -> float:
        d = pose.translation().distance(
            self._segment_points[self._current_idx].pose.translation()
        )
        d += cumulative_distance(
            [sp.pose for sp in self._segment_points[self._current_idx :]]
        )
        return d

    def calculate(self, pose: Pose2d, dt=0.02) -> ChassisSpeeds:
        if len(self._segment_points) == 0:
            self._last_command = ChassisSpeeds()
            return ChassisSpeeds()

        # Advance waypoints if required
        while True:
            current_segment_point = self._segment_points[self._current_idx]
            previous_segment_point = self._segment_points[self._current_idx - 1]
            t = parametric_distance(
                pose.translation(),
                (
                    previous_segment_point.pose.translation(),
                    current_segment_point.pose.translation(),
                ),
            )
            segment_d = (
                current_segment_point.pose.translation()
                - previous_segment_point.pose.translation()
            ).norm()
            if (
                t > 1.0 - current_segment_point.tolerance / segment_d
                and self._current_idx < len(self._segment_points) - 1
            ):
                self._current_idx += 1
            else:
                break
        self._current_idx = min(self._current_idx, len(self._segment_points) - 1)

        current_wp = self._segment_points[self._current_idx]
        prev_wp = self._segment_points[self._current_idx - 1]

        # Compute translation command vector
        segment = (prev_wp.pose.translation(), current_wp.pose.translation())
        segment_delta = segment[1] - segment[0]

        if self._current_idx == len(self._segment_points) - 1:
            speed = np.clip(
                -self._translation_controller.calculate(
                    (1.0 - parametric_distance(pose.translation(), segment))
                    * segment_delta.norm()
                ),
                -current_wp.speed,
                current_wp.speed,
            )
        else:
            speed = current_wp.speed
        direction: Translation2d = segment_delta / segment_delta.norm() * speed

        cross_track = (segment_delta / segment_delta.norm()).rotateBy(
            Rotation2d.fromDegrees(90.0)
        ) * self._cross_track_controller.calculate(
            crosstrack_error(
                pose.translation(),
                segment,
            )
        )

        translation = direction + cross_track

        # Compute rotation command
        t = np.clip(
            1.0 - parametric_distance(pose.translation(), segment),
            0.0001,
            1.0,
        )
        self._rotation_controller.setSetpoint(
            slerp(prev_wp.pose.rotation(), current_wp.pose.rotation(), t).radians()
        )
        omega = self._rotation_controller.calculate(pose.rotation().radians())

        # Apply velocity/acceleration limits
        # Only apply limits if not on the last segment, otherwise let the controllers get us to the goal
        if self._current_idx != len(self._segment_points) - 1:
            v = translation.norm()
            if v > self._motion_parameters.max_linear_speed:
                translation = translation / v * self._motion_parameters.max_linear_speed

            if self._motion_parameters.max_linear_acceleration:
                current_speed = Translation2d(
                    self._last_command.vx, self._last_command.vy
                ).norm()
                desired_accel = (translation.norm() - current_speed) / dt
                if abs(desired_accel) > self._motion_parameters.max_linear_acceleration:
                    translation = (
                        translation
                        / translation.norm()
                        * (current_speed + desired_accel * dt)
                    )

        vx = translation.X()
        vy = translation.Y()

        self._last_command = ChassisSpeeds(vx, vy, omega)
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            self._last_command, pose.rotation()
        )
