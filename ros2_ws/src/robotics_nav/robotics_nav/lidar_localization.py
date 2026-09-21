"""Small dependency-free LiDAR-to-map position matcher."""

from __future__ import annotations

import math
from typing import Iterable, Sequence


Pose2D = tuple[float, float, float]


def _finite_range(value: float, lower: float, upper: float) -> bool:
    return math.isfinite(value) and lower < value < upper


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def compose_pose(first: Pose2D, second: Pose2D) -> Pose2D:
    """Compose two planar poses represented as ``(x, y, yaw)``.

    ``first`` is the transform from an outer frame to an intermediate frame,
    while ``second`` is the pose in that intermediate frame.  The result is
    the same pose expressed in the outer frame.  This is the two-dimensional
    equivalent of multiplying homogeneous SE(2) transforms.
    """
    first_x, first_y, first_yaw = first
    second_x, second_y, second_yaw = second
    cosine = math.cos(first_yaw)
    sine = math.sin(first_yaw)
    return (
        first_x + cosine * second_x - sine * second_y,
        first_y + sine * second_x + cosine * second_y,
        wrap_angle(first_yaw + second_yaw),
    )


def inverse_pose(pose: Pose2D) -> Pose2D:
    """Return the inverse of a planar pose."""
    x, y, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        -cosine * x - sine * y,
        sine * x - cosine * y,
        wrap_angle(-yaw),
    )


def transform_pose(transform: Pose2D, pose: Pose2D) -> Pose2D:
    """Apply a planar transform to a planar pose."""
    return compose_pose(transform, pose)


def map_odom_from_poses(map_pose: Pose2D, odom_pose: Pose2D) -> Pose2D:
    """Estimate ``map→odom`` from corresponding map and odom poses.

    If the same robot pose is known in both frames, then
    ``T_map_odom = T_map_pose * inverse(T_odom_pose)``.  The localizer stores
    this transform instead of overwriting odometry, so later rejected scans do
    not erase the last accepted map correction.
    """
    return compose_pose(map_pose, inverse_pose(odom_pose))


def interpolate_pose(first: Pose2D, second: Pose2D, fraction: float) -> Pose2D:
    """Interpolate a planar transform without crossing the yaw wrap boundary."""
    alpha = min(1.0, max(0.0, float(fraction)))
    return (
        first[0] + alpha * (second[0] - first[0]),
        first[1] + alpha * (second[1] - first[1]),
        wrap_angle(first[2] + alpha * wrap_angle(second[2] - first[2])),
    )


class LidarMapMatcher:
    """Estimate a local x/y correction by matching scan endpoints to a map.

    The matcher deliberately keeps the local model transparent: heading is
    supplied by the wheel/IMU estimator, while only x/y are searched locally.
    It is a local scan-to-map correction, not a full SLAM system.
    """

    def __init__(
        self,
        *,
        search_radius_m: float = 0.35,
        search_step_m: float = 0.025,
        scan_stride: int = 6,
        max_match_distance_m: float = 0.25,
        prior_weight: float = 0.08,
        yaw_search_radius_rad: float = 0.15,
        yaw_search_step_rad: float = 0.05,
        yaw_prior_weight: float = 0.02,
        minimum_points: int = 6,
        ignore_outer_boundary: bool = True,
    ) -> None:
        self.search_radius_m = max(0.0, float(search_radius_m))
        self.search_step_m = max(1.0e-3, float(search_step_m))
        self.scan_stride = max(1, int(scan_stride))
        self.max_match_distance_m = max(1.0e-3, float(max_match_distance_m))
        self.prior_weight = max(0.0, float(prior_weight))
        self.yaw_search_radius_rad = max(0.0, float(yaw_search_radius_rad))
        self.yaw_search_step_rad = max(1.0e-3, float(yaw_search_step_rad))
        self.yaw_prior_weight = max(0.0, float(yaw_prior_weight))
        self.minimum_points = max(1, int(minimum_points))
        self.ignore_outer_boundary = bool(ignore_outer_boundary)

        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.occupied_centres: list[tuple[float, float]] = []
        self.occupied_cells: set[tuple[int, int]] = set()

    @property
    def ready(self) -> bool:
        return bool(self.occupied_cells) and self.resolution > 0.0

    def update_map(
        self,
        *,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: Sequence[int],
        occupied_threshold: int = 50,
    ) -> None:
        """Cache occupied cells from a ROS ``OccupancyGrid`` message.

        The matcher raycasts against cell indices, not directly against Gazebo
        geometry.  The map origin and resolution therefore define the
        conversion between continuous metres and discrete grid cells.  The
        outer boundary may be ignored because it is a planning limit rather
        than a physical LiDAR landmark in this experiment.
        """
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.occupied_centres = []
        self.occupied_cells = set()
        for row in range(self.height):
            for column in range(self.width):
                index = row * self.width + column
                if index >= len(data) or int(data[index]) < occupied_threshold:
                    continue
                if self.ignore_outer_boundary and (
                    row in {0, self.height - 1}
                    or column in {0, self.width - 1}
                ):
                    continue
                self.occupied_centres.append(
                    (
                        self.origin_x + (column + 0.5) * self.resolution,
                        self.origin_y + (row + 0.5) * self.resolution,
                    )
                )
                self.occupied_cells.add((column, row))

    def _scan_measurements(
        self,
        ranges: Sequence[float],
        *,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> list[tuple[float, float | None]]:
        # Keep invalid/no-return rays as ``None``.  They still carry a negative
        # observation: a candidate should not predict an obstacle where the
        # sensor reported no valid return.
        measurements: list[tuple[float, float | None]] = []
        for index in range(0, len(ranges), self.scan_stride):
            distance = float(ranges[index])
            angle = float(angle_min) + index * float(angle_increment)
            if _finite_range(distance, range_min, range_max * 0.995):
                measurements.append((angle, distance))
            else:
                measurements.append((angle, None))
        return measurements

    def _raycast_range(
        self,
        x: float,
        y: float,
        world_angle: float,
        *,
        range_max: float,
    ) -> float | None:
        """Return the first occupied cell encountered along one map ray.

        This transparent grid raycast approximates a continuous LiDAR beam by
        stepping through the map at a fraction of one cell.  The returned
        distance is measured from the candidate LiDAR origin, so it can be
        compared directly with a ``LaserScan`` range.
        """
        step = max(0.01, 0.25 * self.resolution)
        distance = 0.0
        while distance <= range_max:
            world_x = x + distance * math.cos(world_angle)
            world_y = y + distance * math.sin(world_angle)
            column = math.floor((world_x - self.origin_x) / self.resolution)
            row = math.floor((world_y - self.origin_y) / self.resolution)
            if not (0 <= column < self.width and 0 <= row < self.height):
                return None
            # Reconstructing the occupied set on every ray would make the
            # matcher unnecessarily expensive. The cache is populated by
            # update_map and indexed below.
            if (column, row) in self.occupied_cells:
                return distance
            distance += step
        return None

    def _range_error(
        self,
        x: float,
        y: float,
        yaw: float,
        measurements: Iterable[tuple[float, float | None]],
        *,
        range_max: float,
    ) -> float:
        # Cap each ray residual.  A missed return should influence the score,
        # but one bad ray must not dominate all other geometric evidence.
        total = 0.0
        count = 0
        for scan_angle, measured_range in measurements:
            predicted_range = self._raycast_range(
                x,
                y,
                float(yaw) + scan_angle,
                range_max=range_max,
            )
            if measured_range is None:
                residual = (
                    self.max_match_distance_m
                    if predicted_range is not None
                    else 0.0
                )
            elif predicted_range is None:
                residual = self.max_match_distance_m
            else:
                residual = min(
                    abs(predicted_range - measured_range),
                    self.max_match_distance_m,
                )
            total += residual
            count += 1
        if count == 0:
            return float("inf")
        return total / count

    def match(
        self,
        prior_x: float,
        prior_y: float,
        yaw: float,
        ranges: Sequence[float],
        *,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> tuple[float, float, float, int]:
        """Return corrected x/y, score, and number of usable scan points.

        This backward-compatible API keeps the supplied heading fixed. The
        localizer uses :meth:`match_pose` to search a small heading window.
        """
        x, y, _, score, points = self.match_pose(
            prior_x,
            prior_y,
            yaw,
            ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_min=range_min,
            range_max=range_max,
            yaw_search_radius_rad=0.0,
        )
        return x, y, score, points

    def match_pose(
        self,
        prior_x: float,
        prior_y: float,
        prior_yaw: float,
        ranges: Sequence[float],
        *,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        yaw_search_radius_rad: float | None = None,
    ) -> tuple[float, float, float, float, int]:
        """Return corrected ``x/y/yaw``, score, and valid-point count.

        The input pose is the current estimate in the map frame.  The search
        explores a bounded local window around it, so this is a local
        correction mechanism rather than global localization.  The prior
        penalty prefers a nearby solution when several scan matches are
        similarly plausible.
        """
        measurements = self._scan_measurements(
            ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_min=range_min,
            range_max=range_max,
        )
        valid_points = sum(
            measured_range is not None
            for _, measured_range in measurements
        )
        if not self.ready or valid_points < self.minimum_points:
            return (
                float(prior_x),
                float(prior_y),
                float(prior_yaw),
                float("inf"),
                valid_points,
            )

        steps = int(math.ceil(self.search_radius_m / self.search_step_m))
        yaw_radius = self.yaw_search_radius_rad
        if yaw_search_radius_rad is not None:
            yaw_radius = max(0.0, float(yaw_search_radius_rad))
        yaw_steps = int(math.ceil(yaw_radius / self.yaw_search_step_rad))
        best_x = float(prior_x)
        best_y = float(prior_y)
        best_yaw = float(prior_yaw)
        best_score = float("inf")
        best_residual = float("inf")
        # Search heading locally first, then translation.  The fused wheel/IMU
        # yaw is the strongest short-term orientation cue, so this is not a
        # global orientation solve.
        for yaw_step in range(-yaw_steps, yaw_steps + 1):
            candidate_yaw = float(prior_yaw) + yaw_step * self.yaw_search_step_rad
            for x_step in range(-steps, steps + 1):
                candidate_x = float(prior_x) + x_step * self.search_step_m
                for y_step in range(-steps, steps + 1):
                    candidate_y = float(prior_y) + y_step * self.search_step_m
                    residual = self._range_error(
                        candidate_x,
                        candidate_y,
                        candidate_yaw,
                        measurements,
                        range_max=range_max,
                    )
                    if not math.isfinite(residual):
                        continue
                    dx = candidate_x - float(prior_x)
                    dy = candidate_y - float(prior_y)
                    dyaw = wrap_angle(candidate_yaw - float(prior_yaw))
                    # ``residual`` measures scan/map agreement.  The prior
                    # terms regularize weakly observable scenes and keep the
                    # chosen candidate near odometry.
                    score = (
                        residual
                        + self.prior_weight * (dx * dx + dy * dy)
                        + self.yaw_prior_weight * (dyaw * dyaw)
                    )
                    if score < best_score:
                        best_score = score
                        best_residual = residual
                        best_x = candidate_x
                        best_y = candidate_y
                        best_yaw = wrap_angle(candidate_yaw)

        return best_x, best_y, best_yaw, best_residual, valid_points

    def score_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        ranges: Sequence[float],
        *,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> tuple[float, int]:
        """Return the unregularized scan residual at one pose."""
        measurements = self._scan_measurements(
            ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_min=range_min,
            range_max=range_max,
        )
        valid_points = sum(
            measured_range is not None
            for _, measured_range in measurements
        )
        if not self.ready or valid_points < self.minimum_points:
            return float("inf"), valid_points
        return (
            self._range_error(
                float(x),
                float(y),
                float(yaw),
                measurements,
                range_max=range_max,
            ),
            valid_points,
        )
