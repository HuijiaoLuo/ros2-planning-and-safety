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
        score_mode: str = "range",
        optimizer_mode: str = "grid",
        refine_top_k: int = 5,
        # The production localizer applies x/y corrections only.  Keep the
        # score geometry consistent with that state update by holding heading
        # fixed unless a caller explicitly enables yaw diagnostics.
        yaw_search_radius_rad: float = 0.0,
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
        self.score_mode = str(score_mode).strip().lower()
        if self.score_mode not in {
            "range",
            "endpoint",
            "boundary",
            "point_to_line",
        }:
            raise ValueError(
                "score_mode must be 'range', 'endpoint', 'boundary', "
                "or 'point_to_line'"
            )
        self.optimizer_mode = str(optimizer_mode).strip().lower()
        if self.optimizer_mode not in {"grid", "coarse_to_fine"}:
            raise ValueError(
                "optimizer_mode must be 'grid' or 'coarse_to_fine'"
            )
        self.refine_top_k = max(1, int(refine_top_k))
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
        self.occupied_boundary_segments: list[
            tuple[float, float, float, float]
        ] = []
        # Each feature stores a boundary segment followed by its outward
        # normal from the occupied cell into the neighbouring free cell:
        # (x1, y1, x2, y2, nx, ny).  The point-to-line score uses these
        # normals after associating each scan endpoint with its nearest
        # segment.  This is ICP-style local registration, not a global SLAM
        # optimizer: the pose search remains the deterministic bounded grid.
        self.occupied_boundary_features: list[
            tuple[float, float, float, float, float, float]
        ] = []
        self.last_match_diagnostics: dict[str, object] = {}

    @property
    def ready(self) -> bool:
        return bool(self.occupied_cells) and self.resolution > 0.0

    def is_free(self, x: float, y: float, clearance_radius_m: float = 0.0) -> bool:
        """Return whether a candidate robot centre has sufficient map clearance.

        The scan score can be low for a geometrically plausible pose that is
        nevertheless inside an occupied cell, too close to an occupied cell,
        or outside the known map.  The optional square-cell clearance check
        mirrors the planner's conservative obstacle inflation.
        """
        if not self.ready:
            return False
        column = math.floor((float(x) - self.origin_x) / self.resolution)
        row = math.floor((float(y) - self.origin_y) / self.resolution)
        if not (0 <= column < self.width and 0 <= row < self.height):
            return False
        radius_cells = math.ceil(
            max(0.0, float(clearance_radius_m)) / self.resolution
        )
        for row_offset in range(-radius_cells, radius_cells + 1):
            for column_offset in range(-radius_cells, radius_cells + 1):
                if (
                    column + column_offset,
                    row + row_offset,
                ) in self.occupied_cells:
                    return False
        return True

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
        self.occupied_boundary_segments = []
        self.occupied_boundary_features = []
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

        # Cache only obstacle/free interfaces.  A measured LiDAR return is a
        # point on the visible surface, not a point somewhere inside an
        # occupied raster cell.  These continuous cell-edge segments give the
        # diagnostic boundary score sub-cell geometric information while
        # retaining the same static occupancy map as the range model.
        for column, row in self.occupied_cells:
            minimum_x = self.origin_x + column * self.resolution
            maximum_x = minimum_x + self.resolution
            minimum_y = self.origin_y + row * self.resolution
            maximum_y = minimum_y + self.resolution
            neighbours = (
                (
                    (column - 1, row),
                    (minimum_x, minimum_y, minimum_x, maximum_y),
                    (-1.0, 0.0),
                ),
                (
                    (column + 1, row),
                    (maximum_x, minimum_y, maximum_x, maximum_y),
                    (1.0, 0.0),
                ),
                (
                    (column, row - 1),
                    (minimum_x, minimum_y, maximum_x, minimum_y),
                    (0.0, -1.0),
                ),
                (
                    (column, row + 1),
                    (minimum_x, maximum_y, maximum_x, maximum_y),
                    (0.0, 1.0),
                ),
            )
            for neighbour, segment, normal in neighbours:
                if neighbour not in self.occupied_cells:
                    self.occupied_boundary_segments.append(segment)
                    self.occupied_boundary_features.append(
                        (*segment, *normal)
                    )

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
        score_upper_bound: float | None = None,
    ) -> float:
        # Cap each ray residual.  A missed return should influence the score,
        # but one bad ray must not dominate all other geometric evidence.
        measurement_list = list(measurements)
        total_measurements = len(measurement_list)
        if total_measurements == 0:
            return float("inf")
        total = 0.0
        for scan_angle, measured_range in measurement_list:
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
            # The accumulated residual is a lower bound on the final mean
            # residual because all remaining ray residuals are non-negative.
            # If that lower bound already exceeds the best complete candidate
            # score, this candidate cannot win.  Returning infinity is safe:
            # the caller only uses the result to reject a provably worse
            # candidate, so the selected pose and score remain unchanged.
            if (
                score_upper_bound is not None
                and math.isfinite(score_upper_bound)
                and total > score_upper_bound * total_measurements
            ):
                return float("inf")
        return total / total_measurements

    def _point_to_occupied_distance(self, x: float, y: float) -> float:
        """Return distance from a point to the nearest occupied cell.

        A LiDAR return is a point on an obstacle boundary, not generally the
        centre of an occupied grid cell.  Measuring to the cell rectangle
        avoids adding half a cell of systematic error that a centre-only
        distance would introduce.
        """
        best_distance = float("inf")
        for column, row in self.occupied_cells:
            minimum_x = self.origin_x + column * self.resolution
            maximum_x = minimum_x + self.resolution
            minimum_y = self.origin_y + row * self.resolution
            maximum_y = minimum_y + self.resolution
            distance_x = max(minimum_x - x, 0.0, x - maximum_x)
            distance_y = max(minimum_y - y, 0.0, y - maximum_y)
            distance = math.hypot(distance_x, distance_y)
            best_distance = min(best_distance, distance)
        return best_distance

    @staticmethod
    def _point_to_segment_distance(
        x: float,
        y: float,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
    ) -> float:
        """Return Euclidean distance from a point to a line segment."""
        segment_x = end_x - start_x
        segment_y = end_y - start_y
        segment_length_squared = segment_x * segment_x + segment_y * segment_y
        if segment_length_squared <= 0.0:
            return math.hypot(x - start_x, y - start_y)
        projection = (
            (x - start_x) * segment_x + (y - start_y) * segment_y
        ) / segment_length_squared
        projection = min(1.0, max(0.0, projection))
        closest_x = start_x + projection * segment_x
        closest_y = start_y + projection * segment_y
        return math.hypot(x - closest_x, y - closest_y)

    def _point_to_boundary_distance(self, x: float, y: float) -> float:
        """Return distance to the nearest continuous occupied/free boundary."""
        best_distance = float("inf")
        for segment in self.occupied_boundary_segments:
            distance = self._point_to_segment_distance(x, y, *segment)
            best_distance = min(best_distance, distance)
        return best_distance

    def _endpoint_error(
        self,
        x: float,
        y: float,
        yaw: float,
        measurements: Iterable[tuple[float, float | None]],
        *,
        range_max: float,
        score_upper_bound: float | None = None,
    ) -> float:
        """Score measured endpoints against occupied map-cell boundaries.

        This is an alternative to the baseline ray-range residual.  Valid
        returns are transformed into map-frame endpoints and compared with
        the nearest occupied cell rectangle.  No-return rays retain the
        baseline negative-observation penalty, so the alternative remains
        compatible with the existing sensor validity handling.
        """
        measurement_list = list(measurements)
        total_measurements = len(measurement_list)
        if total_measurements == 0:
            return float("inf")
        total = 0.0
        for scan_angle, measured_range in measurement_list:
            if measured_range is None:
                predicted_range = self._raycast_range(
                    x,
                    y,
                    float(yaw) + scan_angle,
                    range_max=range_max,
                )
                residual = (
                    self.max_match_distance_m
                    if predicted_range is not None
                    else 0.0
                )
            else:
                world_angle = float(yaw) + scan_angle
                endpoint_x = x + measured_range * math.cos(world_angle)
                endpoint_y = y + measured_range * math.sin(world_angle)
                residual = min(
                    self._point_to_occupied_distance(endpoint_x, endpoint_y),
                    self.max_match_distance_m,
                )
            total += residual
            # As with the range score, all remaining residuals are
            # non-negative, so this is a safe lower-bound early exit.
            if (
                score_upper_bound is not None
                and math.isfinite(score_upper_bound)
                and total > score_upper_bound * total_measurements
            ):
                return float("inf")
        return total / total_measurements

    def _boundary_error(
        self,
        x: float,
        y: float,
        yaw: float,
        measurements: Iterable[tuple[float, float | None]],
        *,
        range_max: float,
        score_upper_bound: float | None = None,
    ) -> float:
        """Score valid scan endpoints against continuous map boundaries.

        This model is intentionally diagnostic.  It tests whether the
        endpoint geometry is informative once raster-cell interiors are
        removed from the objective.  Invalid returns keep the baseline
        negative-observation penalty; valid returns use distance to a
        free/occupied boundary segment.
        """
        measurement_list = list(measurements)
        total_measurements = len(measurement_list)
        if total_measurements == 0:
            return float("inf")
        total = 0.0
        for scan_angle, measured_range in measurement_list:
            if measured_range is None:
                predicted_range = self._raycast_range(
                    x,
                    y,
                    float(yaw) + scan_angle,
                    range_max=range_max,
                )
                residual = (
                    self.max_match_distance_m
                    if predicted_range is not None
                    else 0.0
                )
            else:
                world_angle = float(yaw) + scan_angle
                endpoint_x = x + measured_range * math.cos(world_angle)
                endpoint_y = y + measured_range * math.sin(world_angle)
                residual = min(
                    self._point_to_boundary_distance(endpoint_x, endpoint_y),
                    self.max_match_distance_m,
                )
            total += residual
            if (
                score_upper_bound is not None
                and math.isfinite(score_upper_bound)
                and total > score_upper_bound * total_measurements
            ):
                return float("inf")
        return total / total_measurements

    def _point_to_line_error(
        self,
        x: float,
        y: float,
        yaw: float,
        measurements: Iterable[tuple[float, float | None]],
        *,
        range_max: float,
        score_upper_bound: float | None = None,
    ) -> float:
        """Score scan endpoints with nearest-map-feature point-to-line residuals.

        This is an ICP-style correspondence model specialized to a known
        occupancy map.  Each valid endpoint is associated with the closest
        occupied/free boundary segment.  The residual is the absolute normal
        distance to that segment's supporting line, plus a small penalty when
        the perpendicular projection falls outside the finite segment.  The
        latter prevents a point from matching an unrelated infinite wall.

        The outer pose search is still the existing bounded deterministic
        grid, so this mode is a local registration experiment rather than a
        full iterative ICP or SLAM implementation.
        """
        measurement_list = list(measurements)
        total_measurements = len(measurement_list)
        if total_measurements == 0 or not self.occupied_boundary_features:
            return float("inf")

        total = 0.0
        for scan_angle, measured_range in measurement_list:
            if measured_range is None:
                predicted_range = self._raycast_range(
                    x,
                    y,
                    float(yaw) + scan_angle,
                    range_max=range_max,
                )
                residual = (
                    self.max_match_distance_m
                    if predicted_range is not None
                    else 0.0
                )
            else:
                world_angle = float(yaw) + scan_angle
                endpoint_x = x + measured_range * math.cos(world_angle)
                endpoint_y = y + measured_range * math.sin(world_angle)
                best_segment_distance = float("inf")
                best_feature: tuple[float, float, float, float, float, float] | None = None
                for feature in self.occupied_boundary_features:
                    segment_distance = self._point_to_segment_distance(
                        endpoint_x,
                        endpoint_y,
                        *feature[:4],
                    )
                    if segment_distance < best_segment_distance:
                        best_segment_distance = segment_distance
                        best_feature = feature

                if best_feature is None or not math.isfinite(best_segment_distance):
                    residual = self.max_match_distance_m
                else:
                    start_x, start_y, end_x, end_y, normal_x, normal_y = (
                        best_feature
                    )
                    segment_x = end_x - start_x
                    segment_y = end_y - start_y
                    length_squared = segment_x * segment_x + segment_y * segment_y
                    if length_squared <= 0.0:
                        projection = 0.0
                    else:
                        projection = (
                            (endpoint_x - start_x) * segment_x
                            + (endpoint_y - start_y) * segment_y
                        ) / length_squared
                    projection = min(1.0, max(0.0, projection))
                    projected_x = start_x + projection * segment_x
                    projected_y = start_y + projection * segment_y
                    normal_residual = abs(
                        (endpoint_x - projected_x) * normal_x
                        + (endpoint_y - projected_y) * normal_y
                    )
                    tangent_residual = math.hypot(
                        endpoint_x - projected_x,
                        endpoint_y - projected_y,
                    )
                    residual = min(
                        normal_residual + 0.5 * tangent_residual,
                        self.max_match_distance_m,
                    )

            total += residual
            if (
                score_upper_bound is not None
                and math.isfinite(score_upper_bound)
                and total > score_upper_bound * total_measurements
            ):
                return float("inf")
        return total / total_measurements

    def _measurement_error(
        self,
        x: float,
        y: float,
        yaw: float,
        measurements: Iterable[tuple[float, float | None]],
        *,
        range_max: float,
        score_upper_bound: float | None = None,
    ) -> float:
        """Dispatch to the configured score model."""
        if self.score_mode == "endpoint":
            return self._endpoint_error(
                x,
                y,
                yaw,
                measurements,
                range_max=range_max,
                score_upper_bound=score_upper_bound,
            )
        if self.score_mode == "boundary":
            return self._boundary_error(
                x,
                y,
                yaw,
                measurements,
                range_max=range_max,
                score_upper_bound=score_upper_bound,
            )
        if self.score_mode == "point_to_line":
            return self._point_to_line_error(
                x,
                y,
                yaw,
                measurements,
                range_max=range_max,
                score_upper_bound=score_upper_bound,
            )
        return self._range_error(
            x,
            y,
            yaw,
            measurements,
            range_max=range_max,
            score_upper_bound=score_upper_bound,
        )

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
            self.last_match_diagnostics = {
                "optimizer_mode": self.optimizer_mode,
                "valid_points": valid_points,
                "evaluated_candidate_count": 0,
                "finite_candidate_count": 0,
                "second_best_score_m": None,
                "score_margin_m": None,
                "best_on_search_boundary": False,
            }
            return (
                float(prior_x),
                float(prior_y),
                float(prior_yaw),
                float("inf"),
                valid_points,
            )

        yaw_radius = self.yaw_search_radius_rad
        if yaw_search_radius_rad is not None:
            yaw_radius = max(0.0, float(yaw_search_radius_rad))
        yaw_steps = int(math.ceil(yaw_radius / self.yaw_search_step_rad))
        best_x = float(prior_x)
        best_y = float(prior_y)
        best_yaw = float(prior_yaw)
        best_score = float("inf")
        best_residual = float("inf")
        evaluated_candidate_count = 0
        finite_candidate_count = 0
        coarse_candidate_count = 0
        refinement_candidate_count = 0
        top_candidates: list[tuple[float, float, float, float, float, float, float, float]] = []
        ranked_candidates: dict[
            tuple[float, float, float],
            tuple[float, float, float, float, float, float, float, float],
        ] = {}

        def candidate_offsets(
            centre_x: float,
            centre_y: float,
            step: float,
            radius: float,
        ) -> list[tuple[float, float]]:
            """Enumerate a circular x/y grid around one search centre."""
            steps = int(math.ceil(radius / step))
            offsets: list[tuple[float, float]] = []
            for x_step in range(-steps, steps + 1):
                for y_step in range(-steps, steps + 1):
                    candidate_x = centre_x + x_step * step
                    candidate_y = centre_y + y_step * step
                    # The configured radius is Euclidean, not the half-width
                    # of the enumerating square.
                    if math.hypot(
                        candidate_x - float(prior_x),
                        candidate_y - float(prior_y),
                    ) > self.search_radius_m + 1.0e-12:
                        continue
                    offsets.append((candidate_x, candidate_y))
            return offsets

        def evaluate_candidates(
            candidates: list[tuple[float, float]],
            *,
            is_refinement: bool,
        ) -> None:
            """Evaluate candidates and retain a small ranked diagnostic set."""
            nonlocal best_x, best_y, best_yaw, best_score, best_residual
            nonlocal evaluated_candidate_count, finite_candidate_count
            nonlocal refinement_candidate_count, coarse_candidate_count
            for candidate_x, candidate_y in candidates:
                if is_refinement:
                    refinement_candidate_count += 1
                else:
                    coarse_candidate_count += 1
                for yaw_step in range(-yaw_steps, yaw_steps + 1):
                    candidate_yaw = (
                        float(prior_yaw) + yaw_step * self.yaw_search_step_rad
                    )
                    evaluated_candidate_count += 1
                    dx = candidate_x - float(prior_x)
                    dy = candidate_y - float(prior_y)
                    residual = self._measurement_error(
                        candidate_x,
                        candidate_y,
                        candidate_yaw,
                        measurements,
                        range_max=range_max,
                        score_upper_bound=best_score,
                    )
                    if not math.isfinite(residual):
                        continue
                    finite_candidate_count += 1
                    dyaw = wrap_angle(candidate_yaw - float(prior_yaw))
                    # ``residual`` measures scan/map agreement.  The prior
                    # terms regularize weakly observable scenes and keep the
                    # chosen candidate near odometry.
                    score = (
                        residual
                        + self.prior_weight * (dx * dx + dy * dy)
                        + self.yaw_prior_weight * (dyaw * dyaw)
                    )
                    entry = (
                        score,
                        residual,
                        candidate_x,
                        candidate_y,
                        wrap_angle(candidate_yaw),
                        dx,
                        dy,
                        dyaw,
                    )
                    pose_key = (
                        round(candidate_x, 9),
                        round(candidate_y, 9),
                        round(wrap_angle(candidate_yaw), 9),
                    )
                    previous_entry = ranked_candidates.get(pose_key)
                    if previous_entry is None or score < previous_entry[0]:
                        ranked_candidates[pose_key] = entry
                    top_candidates[:] = sorted(
                        ranked_candidates.values(),
                        key=lambda ranked_entry: ranked_entry[0],
                    )[:10]
                    if score < best_score:
                        best_score = score
                        best_residual = residual
                        best_x = candidate_x
                        best_y = candidate_y
                        best_yaw = wrap_angle(candidate_yaw)

        if self.optimizer_mode == "grid":
            evaluate_candidates(
                candidate_offsets(
                    float(prior_x),
                    float(prior_y),
                    self.search_step_m,
                    self.search_radius_m,
                ),
                is_refinement=False,
            )
        else:
            # Coarse-to-fine remains deterministic and global at the coarse
            # level.  Only the best coarse basins are refined, which reduces
            # repeated ray casts without pretending that the objective is
            # differentiable.  The default grid mode is unchanged.
            coarse_step = max(self.search_step_m * 2.0, self.search_step_m)
            evaluate_candidates(
                candidate_offsets(
                    float(prior_x),
                    float(prior_y),
                    coarse_step,
                    self.search_radius_m,
                ),
                is_refinement=False,
            )
            coarse_winners = [
                (entry[2], entry[3])
                for entry in top_candidates[: self.refine_top_k]
            ]
            refinement_centres = list(dict.fromkeys(coarse_winners))
            for centre_x, centre_y in refinement_centres:
                evaluate_candidates(
                    candidate_offsets(
                        centre_x,
                        centre_y,
                        self.search_step_m,
                        coarse_step,
                    ),
                    is_refinement=True,
                )

        second_best_score = (
            top_candidates[1][0] if len(top_candidates) >= 2 else None
        )
        score_margin = (
            second_best_score - best_score
            if second_best_score is not None and math.isfinite(best_score)
            else None
        )
        best_distance = math.hypot(
            best_x - float(prior_x),
            best_y - float(prior_y),
        )
        boundary_tolerance = max(self.search_step_m, 0.5 * self.search_step_m)
        best_on_search_boundary = (
            self.search_radius_m > 0.0
            and best_distance >= self.search_radius_m - boundary_tolerance
        )
        self.last_match_diagnostics = {
            "optimizer_mode": self.optimizer_mode,
            "valid_points": valid_points,
            "evaluated_candidate_count": evaluated_candidate_count,
            "finite_candidate_count": finite_candidate_count,
            "coarse_candidate_count": coarse_candidate_count,
            "refinement_candidate_count": refinement_candidate_count,
            "second_best_score_m": second_best_score,
            "score_margin_m": score_margin,
            "best_regularized_score_m": (
                best_score if math.isfinite(best_score) else None
            ),
            "best_residual_m": (
                best_residual if math.isfinite(best_residual) else None
            ),
            "best_prior_penalty_m": (
                best_score - best_residual
                if math.isfinite(best_score) and math.isfinite(best_residual)
                else None
            ),
            "best_distance_from_prior_m": best_distance,
            "best_on_search_boundary": best_on_search_boundary,
            "top_candidates": [
                {
                    "score_m": entry[0],
                    "residual_m": entry[1],
                    "x_m": entry[2],
                    "y_m": entry[3],
                    "yaw_rad": entry[4],
                    "dx_m": entry[5],
                    "dy_m": entry[6],
                    "dyaw_rad": entry[7],
                }
                for entry in top_candidates[:5]
            ],
        }

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
            self._measurement_error(
                float(x),
                float(y),
                float(yaw),
                measurements,
                range_max=range_max,
            ),
            valid_points,
        )
