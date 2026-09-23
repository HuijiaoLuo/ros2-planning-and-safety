"""Dependency-free Monte Carlo localization primitives.

The deterministic localizer selects one nearby pose with a grid search.  This
model keeps several pose hypotheses instead.  Wheel/IMU motion is used as a prior,
and a likelihood-field LiDAR model assigns each particle a weight from the
known occupancy map.  This module intentionally has no ROS imports so that the
motion model, likelihood model, resampling, and covariance calculation can be
tested independently of callback timing.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from functools import cached_property
from typing import Iterable, Sequence


Pose2D = tuple[float, float, float]
Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the half-open interval ``[-pi, pi)``."""

    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class MCLConfig:
    """Physical/statistical configuration for the particle filter.

    These values describe one reproducible baseline model.  They are not
    intended to be map-specific optimizer knobs.  Cross-map experiments must
    keep this configuration fixed and report where the model is inadequate.
    """

    particle_count: int = 500
    motion_distance_noise_std_m: float = 0.01
    motion_yaw_noise_std_rad: float = 0.01
    lidar_range_sigma_m: float = 0.08
    lidar_likelihood_floor: float = 1.0e-6
    resample_ess_ratio: float = 0.5
    random_seed: int = 0

    def __post_init__(self) -> None:
        if self.particle_count < 2:
            raise ValueError("particle_count must be at least 2")
        if self.motion_distance_noise_std_m < 0.0:
            raise ValueError("motion distance noise must be non-negative")
        if self.motion_yaw_noise_std_rad < 0.0:
            raise ValueError("motion yaw noise must be non-negative")
        if self.lidar_range_sigma_m <= 0.0:
            raise ValueError("lidar_range_sigma_m must be positive")
        if not 0.0 < self.lidar_likelihood_floor < 1.0:
            raise ValueError("likelihood floor must be in (0, 1)")
        if not 0.0 < self.resample_ess_ratio <= 1.0:
            raise ValueError("resample_ess_ratio must be in (0, 1]")


@dataclass(frozen=True)
class OccupancyGridMap:
    """Continuous-coordinate view of the occupied cells in a static map."""

    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    occupied_cells: frozenset[tuple[int, int]]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map dimensions must be positive")
        if self.resolution_m <= 0.0:
            raise ValueError("map resolution must be positive")

    @classmethod
    def from_ascii(
        cls,
        rows: Sequence[str],
        *,
        resolution_m: float = 1.0,
        origin_x_m: float = 0.0,
        origin_y_m: float = 0.0,
        occupied_symbols: str = "#X1",
    ) -> "OccupancyGridMap":
        """Build a small map fixture from rows of ``.`` and occupied cells."""

        if not rows:
            raise ValueError("rows must not be empty")
        width = len(rows[0])
        if width == 0 or any(len(row) != width for row in rows):
            raise ValueError("ASCII map rows must have equal non-zero width")
        occupied = frozenset(
            (column, row)
            for row, line in enumerate(rows)
            for column, symbol in enumerate(line)
            if symbol in occupied_symbols
        )
        return cls(
            width=width,
            height=len(rows),
            resolution_m=resolution_m,
            origin_x_m=origin_x_m,
            origin_y_m=origin_y_m,
            occupied_cells=occupied,
        )

    @cached_property
    def occupied_centres(self) -> tuple[tuple[float, float], ...]:
        """Return cached occupied-cell centres in world coordinates.

        LiDAR likelihood evaluation queries this set once per particle and
        beam.  Caching the static map geometry keeps the model unchanged while
        avoiding repeated coordinate allocation in the hot loop.
        """

        return tuple(
            (
                self.origin_x_m + (column + 0.5) * self.resolution_m,
                self.origin_y_m + (row + 0.5) * self.resolution_m,
            )
            for column, row in self.occupied_cells
        )

    @cached_property
    def occupied_distance_field_m(self) -> tuple[float, ...]:
        """Precompute the static cell-to-obstacle distance field.

        The likelihood-field model only needs the distance at an endpoint's
        map cell.  Computing that distance once per map cell avoids scanning
        every occupied cell for every particle and LiDAR beam.  The result is
        still the same nearest-occupied-centre model evaluated on the map
        raster, with the rasterization error made explicit and repeatable.
        """

        centres = self.occupied_centres
        if not centres:
            return tuple(math.inf for _ in range(self.width * self.height))
        distances: list[float] = []
        for row in range(self.height):
            y = self.origin_y_m + (row + 0.5) * self.resolution_m
            for column in range(self.width):
                x = self.origin_x_m + (column + 0.5) * self.resolution_m
                distances.append(
                    min(math.hypot(x - cx, y - cy) for cx, cy in centres)
                )
        return tuple(distances)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        """Convert a world point to a map cell, returning ``None`` outside."""

        column = math.floor((x - self.origin_x_m) / self.resolution_m)
        row = math.floor((y - self.origin_y_m) / self.resolution_m)
        if not (0 <= column < self.width and 0 <= row < self.height):
            return None
        return column, row

    def is_free(self, x: float, y: float) -> bool:
        """Return whether a continuous pose centre lies in a free map cell."""

        cell = self.world_to_cell(x, y)
        return cell is not None and cell not in self.occupied_cells

    def nearest_occupied_distance_m(self, x: float, y: float) -> float:
        """Return the cached likelihood-field distance for a world point."""

        cell = self.world_to_cell(x, y)
        if cell is None:
            return math.inf
        column, row = cell
        return self.occupied_distance_field_m[row * self.width + column]


@dataclass(frozen=True)
class Particle:
    """One weighted pose hypothesis in the map frame."""

    x_m: float
    y_m: float
    yaw_rad: float
    weight: float


@dataclass(frozen=True)
class MCLMeasurementResult:
    """Output of one LiDAR update, including uncertainty diagnostics."""

    pose: Pose2D
    covariance: Matrix3
    effective_sample_size: float
    maximum_weight: float
    normalized_entropy: float
    valid_beam_count: int
    resampled: bool


@dataclass(frozen=True)
class MCLScanObservation:
    """One scan expressed relative to the current particle time.

    ``relative_*`` is the odometry-frame transform from the scan pose to the
    current pose.  The filter inverts that transform for each current particle
    before evaluating the historical scan, so a short scan window contributes
    time-consistent evidence instead of treating old returns as if they were
    measured at the current pose.
    """

    ranges_m: tuple[float, ...]
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    relative_x_m: float = 0.0
    relative_y_m: float = 0.0
    relative_yaw_rad: float = 0.0


class ParticleFilter2D:
    """A small known-map particle filter for planar robot localization."""

    def __init__(
        self,
        occupancy_map: OccupancyGridMap,
        config: MCLConfig | None = None,
    ) -> None:
        self.map = occupancy_map
        self.config = config or MCLConfig()
        self._rng = random.Random(self.config.random_seed)
        uniform_weight = 1.0 / self.config.particle_count
        self.particles: list[Particle] = [
            Particle(0.0, 0.0, 0.0, uniform_weight)
            for _ in range(self.config.particle_count)
        ]

    def set_particles(self, particles: Iterable[Particle]) -> None:
        """Replace the particle set and normalize its weights."""

        values = list(particles)
        if len(values) != self.config.particle_count:
            raise ValueError("particle count does not match MCLConfig")
        total = sum(max(0.0, particle.weight) for particle in values)
        if total <= 0.0 or not math.isfinite(total):
            weight = 1.0 / len(values)
            self.particles = [
                Particle(p.x_m, p.y_m, wrap_angle(p.yaw_rad), weight)
                for p in values
            ]
            return
        self.particles = [
            Particle(
                particle.x_m,
                particle.y_m,
                wrap_angle(particle.yaw_rad),
                max(0.0, particle.weight) / total,
            )
            for particle in values
        ]

    def initialize_gaussian(
        self,
        mean_pose: Pose2D,
        std_x_m: float,
        std_y_m: float,
        std_yaw_rad: float,
    ) -> None:
        """Initialize hypotheses around a pose, rejecting occupied cells."""

        if min(std_x_m, std_y_m, std_yaw_rad) < 0.0:
            raise ValueError("initial standard deviations must be non-negative")
        particles: list[Particle] = []
        attempts = 0
        maximum_attempts = max(100, self.config.particle_count * 20)
        while len(particles) < self.config.particle_count and attempts < maximum_attempts:
            attempts += 1
            x = self._rng.gauss(mean_pose[0], std_x_m)
            y = self._rng.gauss(mean_pose[1], std_y_m)
            if self.map.is_free(x, y):
                particles.append(
                    Particle(
                        x,
                        y,
                        wrap_angle(self._rng.gauss(mean_pose[2], std_yaw_rad)),
                        1.0 / self.config.particle_count,
                    )
                )
        if len(particles) != self.config.particle_count:
            raise ValueError("could not initialize all particles in free space")
        self.particles = particles

    def initialize_uniform_free(self) -> None:
        """Initialize particles uniformly over free map-cell centres."""

        free_cells = [
            (column, row)
            for row in range(self.map.height)
            for column in range(self.map.width)
            if (column, row) not in self.map.occupied_cells
        ]
        if not free_cells:
            raise ValueError("map has no free cells")
        weight = 1.0 / self.config.particle_count
        self.particles = []
        for _ in range(self.config.particle_count):
            column, row = self._rng.choice(free_cells)
            self.particles.append(
                Particle(
                    self.map.origin_x_m + (column + 0.5) * self.map.resolution_m,
                    self.map.origin_y_m + (row + 0.5) * self.map.resolution_m,
                    self._rng.uniform(-math.pi, math.pi),
                    weight,
                )
            )

    def predict(self, delta_distance_m: float, delta_yaw_rad: float) -> None:
        """Propagate each hypothesis with a midpoint differential-drive model."""

        updated: list[Particle] = []
        for particle in self.particles:
            distance = float(delta_distance_m)
            yaw_delta = float(delta_yaw_rad)
            if self.config.motion_distance_noise_std_m > 0.0:
                distance += self._rng.gauss(
                    0.0, self.config.motion_distance_noise_std_m
                )
            if self.config.motion_yaw_noise_std_rad > 0.0:
                yaw_delta += self._rng.gauss(
                    0.0, self.config.motion_yaw_noise_std_rad
                )
            midpoint_yaw = particle.yaw_rad + 0.5 * yaw_delta
            updated.append(
                Particle(
                    particle.x_m + distance * math.cos(midpoint_yaw),
                    particle.y_m + distance * math.sin(midpoint_yaw),
                    wrap_angle(particle.yaw_rad + yaw_delta),
                    particle.weight,
                )
            )
        self.particles = updated

    def pose_log_likelihood(
        self,
        pose: Pose2D,
        ranges_m: Sequence[float],
        *,
        angle_min_rad: float,
        angle_increment_rad: float,
        range_min_m: float,
        range_max_m: float,
    ) -> tuple[float, int]:
        """Score scan endpoints with a likelihood-field map model.

        A valid measured endpoint near an occupied-cell centre receives high
        likelihood.  Invalid, out-of-range, or map-inconsistent beams are
        ignored or assigned the configured floor.  The score is averaged over
        valid beams so scans with different valid counts remain comparable.
        """

        x, y, yaw = pose
        if not self.map.is_free(x, y):
            return math.log(self.config.lidar_likelihood_floor), 0
        log_likelihood = 0.0
        valid_count = 0
        sigma = self.config.lidar_range_sigma_m
        floor = self.config.lidar_likelihood_floor
        for index, range_m in enumerate(ranges_m):
            if not math.isfinite(range_m) or not (range_min_m < range_m < range_max_m):
                continue
            beam_angle = yaw + angle_min_rad + index * angle_increment_rad
            endpoint_x = x + range_m * math.cos(beam_angle)
            endpoint_y = y + range_m * math.sin(beam_angle)
            distance = self.map.nearest_occupied_distance_m(endpoint_x, endpoint_y)
            if not math.isfinite(distance):
                likelihood = floor
            else:
                likelihood = math.exp(-0.5 * (distance / sigma) ** 2) + floor
            log_likelihood += math.log(max(floor, likelihood))
            valid_count += 1
        if valid_count == 0:
            return math.log(floor), 0
        return log_likelihood / valid_count, valid_count

    @staticmethod
    def scan_pose_from_current_particle(
        particle_pose: Pose2D,
        observation: MCLScanObservation,
    ) -> Pose2D:
        """Recover a historical scan pose from a current particle pose.

        The observation transform maps the historical scan frame into the
        current odometry frame.  Applying its inverse keeps the LiDAR endpoint
        geometry aligned when several scans are scored together.
        """

        dx = observation.relative_x_m
        dy = observation.relative_y_m
        dyaw = observation.relative_yaw_rad
        cos_inv = math.cos(dyaw)
        sin_inv = math.sin(dyaw)
        inverse_x = -(cos_inv * dx + sin_inv * dy)
        inverse_y = sin_inv * dx - cos_inv * dy
        x, y, yaw = particle_pose
        scan_x = x + math.cos(yaw) * inverse_x - math.sin(yaw) * inverse_y
        scan_y = y + math.sin(yaw) * inverse_x + math.cos(yaw) * inverse_y
        return scan_x, scan_y, wrap_angle(yaw - dyaw)

    def pose_log_likelihood_sequence(
        self,
        pose: Pose2D,
        observations: Sequence[MCLScanObservation],
    ) -> tuple[float, int]:
        """Accumulate independent scan evidence in the log domain.

        ``pose_log_likelihood`` returns a per-scan mean log likelihood so a
        single scan is not rewarded merely for having more valid beams. A
        temporal window contains separate observations, however, so their
        normalized scan likelihoods are multiplied. The product is computed
        as a sum of log likelihoods; a window of one therefore preserves the
        original single-scan update exactly.
        """

        if not observations:
            return math.log(self.config.lidar_likelihood_floor), 0
        accumulated_log_likelihood = 0.0
        total_valid_count = 0
        for observation in observations:
            scan_pose = self.scan_pose_from_current_particle(pose, observation)
            log_likelihood, valid_count = self.pose_log_likelihood(
                scan_pose,
                observation.ranges_m,
                angle_min_rad=observation.angle_min_rad,
                angle_increment_rad=observation.angle_increment_rad,
                range_min_m=observation.range_min_m,
                range_max_m=observation.range_max_m,
            )
            accumulated_log_likelihood += log_likelihood
            total_valid_count += valid_count
        return accumulated_log_likelihood, total_valid_count

    def update(
        self,
        ranges_m: Sequence[float],
        *,
        angle_min_rad: float,
        angle_increment_rad: float,
        range_min_m: float,
        range_max_m: float,
    ) -> MCLMeasurementResult:
        """Apply one LiDAR update through the temporal-window implementation."""

        return self.update_sequence(
            [
                MCLScanObservation(
                    ranges_m=tuple(float(value) for value in ranges_m),
                    angle_min_rad=angle_min_rad,
                    angle_increment_rad=angle_increment_rad,
                    range_min_m=range_min_m,
                    range_max_m=range_max_m,
                )
            ]
        )

    def update_sequence(
        self,
        observations: Sequence[MCLScanObservation],
    ) -> MCLMeasurementResult:
        """Apply a joint short-window LiDAR update.

        The prior weight is multiplied by the product of the per-scan
        likelihood blocks. In log space this is the sum of the independent,
        time-consistent scan log likelihoods. A single-scan call is unchanged,
        while a window accumulates independent geometric evidence without
        changing the configured uncertainty gate.
        """

        if not observations:
            raise ValueError("observations must not be empty")

        # Beam validity depends on the scan limits, not on the particle pose.
        # Keep this count separate from the likelihood loop so diagnostics
        # report the amount of temporal evidence supplied to the update.
        valid_beam_count = sum(
            sum(
                1
                for range_m in observation.ranges_m
                if math.isfinite(range_m)
                and observation.range_min_m < range_m < observation.range_max_m
            )
            for observation in observations
        )
        log_weights: list[float] = []
        for particle in self.particles:
            log_likelihood, _valid_count = self.pose_log_likelihood_sequence(
                (particle.x_m, particle.y_m, particle.yaw_rad),
                observations,
            )
            log_weights.append(math.log(max(particle.weight, 1.0e-300)) + log_likelihood)

        maximum = max(log_weights)
        raw_weights = [math.exp(value - maximum) for value in log_weights]
        total = sum(raw_weights)
        if total <= 0.0 or not math.isfinite(total):
            normalized = [1.0 / len(raw_weights)] * len(raw_weights)
        else:
            normalized = [value / total for value in raw_weights]
        self.particles = [
            Particle(p.x_m, p.y_m, p.yaw_rad, weight)
            for p, weight in zip(self.particles, normalized)
        ]

        ess = self.effective_sample_size()
        resampled = ess < self.config.resample_ess_ratio * len(self.particles)
        estimate = self.estimate(ess=ess, valid_beam_count=valid_beam_count, resampled=resampled)
        if resampled:
            self.resample_systematic()
        return estimate

    def effective_sample_size(self) -> float:
        """Return ``1 / sum(w_i^2)`` for the normalized particle weights."""

        denominator = sum(particle.weight * particle.weight for particle in self.particles)
        return 1.0 / denominator if denominator > 0.0 else 0.0

    def resample_systematic(self) -> None:
        """Replace particles using low-variance systematic resampling."""

        count = len(self.particles)
        cumulative: list[float] = []
        running = 0.0
        for particle in self.particles:
            running += particle.weight
            cumulative.append(running)
        cumulative[-1] = 1.0
        step = 1.0 / count
        position = self._rng.random() * step
        index = 0
        uniform_weight = step
        resampled: list[Particle] = []
        for sample_index in range(count):
            threshold = position + sample_index * step
            while index < count - 1 and threshold > cumulative[index]:
                index += 1
            particle = self.particles[index]
            resampled.append(
                Particle(particle.x_m, particle.y_m, particle.yaw_rad, uniform_weight)
            )
        self.particles = resampled

    def estimate(
        self,
        *,
        ess: float | None = None,
        valid_beam_count: int = 0,
        resampled: bool = False,
    ) -> MCLMeasurementResult:
        """Return weighted pose mean, covariance, and ambiguity diagnostics."""

        weights = [particle.weight for particle in self.particles]
        x = sum(weight * particle.x_m for weight, particle in zip(weights, self.particles))
        y = sum(weight * particle.y_m for weight, particle in zip(weights, self.particles))
        sin_yaw = sum(weight * math.sin(p.yaw_rad) for weight, p in zip(weights, self.particles))
        cos_yaw = sum(weight * math.cos(p.yaw_rad) for weight, p in zip(weights, self.particles))
        yaw = math.atan2(sin_yaw, cos_yaw)
        pxx = pxy = px_yaw = pyy = py_yaw = p_yaw_yaw = 0.0
        for weight, particle in zip(weights, self.particles):
            dx = particle.x_m - x
            dy = particle.y_m - y
            dyaw = wrap_angle(particle.yaw_rad - yaw)
            pxx += weight * dx * dx
            pxy += weight * dx * dy
            px_yaw += weight * dx * dyaw
            pyy += weight * dy * dy
            py_yaw += weight * dy * dyaw
            p_yaw_yaw += weight * dyaw * dyaw
        covariance: Matrix3 = (
            (pxx, pxy, px_yaw),
            (pxy, pyy, py_yaw),
            (px_yaw, py_yaw, p_yaw_yaw),
        )
        if ess is None:
            ess = self.effective_sample_size()
        maximum_weight = max(weights, default=0.0)
        entropy = -sum(
            weight * math.log(weight)
            for weight in weights
            if weight > 0.0
        )
        normalized_entropy = entropy / math.log(len(weights)) if len(weights) > 1 else 0.0
        return MCLMeasurementResult(
            pose=(x, y, yaw),
            covariance=covariance,
            effective_sample_size=ess,
            maximum_weight=maximum_weight,
            normalized_entropy=normalized_entropy,
            valid_beam_count=valid_beam_count,
            resampled=resampled,
        )
