"""Dependency-free point-to-point ICP for a known occupancy map.

This module is a deliberately small registration baseline, not a SLAM
implementation.  It aligns valid LiDAR endpoints with occupied-cell centres
from a static map, starting from a supplied pose estimate.  The iterative
closest-point update is useful here as a model comparison: it has a different
local objective from the particle likelihood-field model, while keeping the
map, scan, and initial odometry hypothesis unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from robotics_nav.mcl_localization import OccupancyGridMap, Pose2D, wrap_angle


@dataclass(frozen=True)
class ICPConfig:
    """Numerical stopping limits for the registration iteration."""

    max_iterations: int = 20
    maximum_correspondence_distance_m: float = 0.35
    minimum_correspondences: int = 6
    translation_tolerance_m: float = 1.0e-3
    rotation_tolerance_rad: float = 1.0e-3

    def __post_init__(self) -> None:
        if self.max_iterations < 1:
            raise ValueError("max_iterations must be positive")
        if self.maximum_correspondence_distance_m <= 0.0:
            raise ValueError("correspondence distance must be positive")
        if self.minimum_correspondences < 3:
            raise ValueError("at least three correspondences are required")
        if self.translation_tolerance_m <= 0.0:
            raise ValueError("translation tolerance must be positive")
        if self.rotation_tolerance_rad <= 0.0:
            raise ValueError("rotation tolerance must be positive")


@dataclass(frozen=True)
class ICPResult:
    """Auditable output of one point-to-point registration attempt."""

    pose: Pose2D
    initial_pose: Pose2D
    mean_residual_m: float
    rms_residual_m: float
    correspondence_count: int
    iterations: int
    converged: bool
    status: str

    @property
    def correction_x_m(self) -> float:
        return self.pose[0] - self.initial_pose[0]

    @property
    def correction_y_m(self) -> float:
        return self.pose[1] - self.initial_pose[1]

    @property
    def correction_m(self) -> float:
        return math.hypot(self.correction_x_m, self.correction_y_m)

    @property
    def correction_yaw_rad(self) -> float:
        return wrap_angle(self.pose[2] - self.initial_pose[2])


def scan_points_from_ranges(
    ranges_m: Sequence[float],
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
    stride: int = 1,
) -> list[tuple[float, float]]:
    """Convert valid scan returns to points in the LiDAR frame."""

    points: list[tuple[float, float]] = []
    step = max(1, int(stride))
    for index in range(0, len(ranges_m), step):
        range_m = float(ranges_m[index])
        if not math.isfinite(range_m) or not (range_min_m < range_m < range_max_m):
            continue
        angle = angle_min_rad + index * angle_increment_rad
        points.append((range_m * math.cos(angle), range_m * math.sin(angle)))
    return points


def transform_point(point: tuple[float, float], pose: Pose2D) -> tuple[float, float]:
    """Transform one LiDAR-frame point into the map frame."""

    x, y = point
    px, py, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return px + cosine * x - sine * y, py + sine * x + cosine * y


def _nearest_correspondences(
    transformed: Sequence[tuple[float, float]],
    map_points: Sequence[tuple[float, float]],
    maximum_distance_m: float,
) -> tuple[list[tuple[tuple[float, float], tuple[float, float], float]], float]:
    """Find bounded nearest-neighbour pairs using the static map points."""

    maximum_distance_sq = maximum_distance_m * maximum_distance_m
    pairs: list[tuple[tuple[float, float], tuple[float, float], float]] = []
    for source in transformed:
        target, distance_sq = min(
            ((target, (source[0] - target[0]) ** 2 + (source[1] - target[1]) ** 2)
             for target in map_points),
            key=lambda item: item[1],
        )
        if distance_sq <= maximum_distance_sq:
            pairs.append((source, target, math.sqrt(distance_sq)))
    return pairs, maximum_distance_m


def _best_fit_delta(
    pairs: Sequence[tuple[tuple[float, float], tuple[float, float], float]],
) -> tuple[float, float, float]:
    """Solve the 2D rigid point-to-point least-squares increment."""

    source_x = sum(pair[0][0] for pair in pairs) / len(pairs)
    source_y = sum(pair[0][1] for pair in pairs) / len(pairs)
    target_x = sum(pair[1][0] for pair in pairs) / len(pairs)
    target_y = sum(pair[1][1] for pair in pairs) / len(pairs)
    dot = 0.0
    cross = 0.0
    for source, target, _ in pairs:
        sx = source[0] - source_x
        sy = source[1] - source_y
        tx = target[0] - target_x
        ty = target[1] - target_y
        dot += sx * tx + sy * ty
        cross += sx * ty - sy * tx
    delta_yaw = math.atan2(cross, dot)
    cosine = math.cos(delta_yaw)
    sine = math.sin(delta_yaw)
    delta_x = target_x - (cosine * source_x - sine * source_y)
    delta_y = target_y - (sine * source_x + cosine * source_y)
    return delta_x, delta_y, delta_yaw


def point_to_point_icp(
    scan_points: Sequence[tuple[float, float]],
    occupancy_map: OccupancyGridMap,
    initial_pose: Pose2D,
    *,
    config: ICPConfig | None = None,
) -> ICPResult:
    """Align scan endpoints to occupied map cells from a local prior pose."""

    settings = config or ICPConfig()
    map_points = occupancy_map.occupied_centres
    if len(scan_points) < settings.minimum_correspondences:
        return ICPResult(
            pose=initial_pose,
            initial_pose=initial_pose,
            mean_residual_m=float("inf"),
            rms_residual_m=float("inf"),
            correspondence_count=0,
            iterations=0,
            converged=False,
            status="insufficient_scan_points",
        )
    if not map_points:
        return ICPResult(
            pose=initial_pose,
            initial_pose=initial_pose,
            mean_residual_m=float("inf"),
            rms_residual_m=float("inf"),
            correspondence_count=0,
            iterations=0,
            converged=False,
            status="empty_map",
        )

    pose = initial_pose
    final_pairs: list[tuple[tuple[float, float], tuple[float, float], float]] = []
    converged = False
    status = "maximum_iterations"
    completed_iterations = 0
    for iteration in range(1, settings.max_iterations + 1):
        transformed = [transform_point(point, pose) for point in scan_points]
        pairs, _ = _nearest_correspondences(
            transformed,
            map_points,
            settings.maximum_correspondence_distance_m,
        )
        final_pairs = pairs
        completed_iterations = iteration
        if len(pairs) < settings.minimum_correspondences:
            status = "insufficient_correspondences"
            break
        delta_x, delta_y, delta_yaw = _best_fit_delta(pairs)
        cosine = math.cos(delta_yaw)
        sine = math.sin(delta_yaw)
        pose = (
            cosine * pose[0] - sine * pose[1] + delta_x,
            sine * pose[0] + cosine * pose[1] + delta_y,
            wrap_angle(pose[2] + delta_yaw),
        )
        if (
            math.hypot(delta_x, delta_y) <= settings.translation_tolerance_m
            and abs(delta_yaw) <= settings.rotation_tolerance_rad
        ):
            converged = True
            status = "converged"
            break

    if final_pairs:
        residuals = [pair[2] for pair in final_pairs]
        mean_residual = sum(residuals) / len(residuals)
        rms_residual = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    else:
        mean_residual = float("inf")
        rms_residual = float("inf")
    return ICPResult(
        pose=pose,
        initial_pose=initial_pose,
        mean_residual_m=mean_residual,
        rms_residual_m=rms_residual,
        correspondence_count=len(final_pairs),
        iterations=completed_iterations,
        converged=converged,
        status=status,
    )
