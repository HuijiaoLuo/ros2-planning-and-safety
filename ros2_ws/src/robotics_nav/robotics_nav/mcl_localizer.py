"""ROS 2 adapter for the known-map Monte Carlo localizer."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path
from typing import Optional, Sequence

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String

from robotics_nav.mcl_localization import (
    MCLConfig,
    MCLScanObservation,
    OccupancyGridMap,
    Particle,
    ParticleFilter2D,
    Pose2D,
    wrap_angle,
)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def stamp_seconds(message: object) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def interpolate_pose_history(
    history: Sequence[tuple[float, Pose2D]],
    stamp_s: float,
) -> Optional[Pose2D]:
    """Interpolate the state estimate at a LiDAR measurement timestamp.

    ROS callbacks do not arrive in timestamp order: a scan can be received
    before the state-estimate message carrying the same simulator time.  The
    old adapter paired that scan with whichever pose happened to be latest at
    callback time, which created physically impossible pairs such as a scan at
    ``t=0.20`` associated with a pose at ``t=0.17``.  MCL then interpreted the
    timestamp mismatch as motion and scored the wrong geometry.

    Position is linearly interpolated.  Heading uses the wrapped shortest
    angular difference, so interpolation remains valid across ``+pi/-pi``.
    The function returns ``None`` until a pose at or after the requested scan
    time is available; the caller must keep that scan pending rather than
    inventing a future state.
    """
    if not history or not math.isfinite(stamp_s):
        return None
    if stamp_s < history[0][0] or stamp_s > history[-1][0]:
        return None
    for index in range(len(history) - 1):
        first_stamp, first_pose = history[index]
        second_stamp, second_pose = history[index + 1]
        if stamp_s > second_stamp:
            continue
        interval = second_stamp - first_stamp
        if interval <= 1.0e-12:
            return second_pose
        fraction = (stamp_s - first_stamp) / interval
        yaw_delta = wrap_angle(second_pose[2] - first_pose[2])
        return (
            first_pose[0] + fraction * (second_pose[0] - first_pose[0]),
            first_pose[1] + fraction * (second_pose[1] - first_pose[1]),
            wrap_angle(first_pose[2] + fraction * yaw_delta),
        )
    return history[-1][1]


class MCLLocalizer(Node):
    """Publish a probabilistic map pose under the existing topic contract.

    The first experiment assumes the simulator's ``map`` and ``odom``
    coordinates are aligned. The node intentionally publishes the input frame
    ID unchanged. A later integration can add a timestamped map-to-odom
    transform without changing the particle-filter core.
    """

    def __init__(self) -> None:
        super().__init__("mcl_localizer")
        self.declare_parameter("input_pose_topic", "/state_estimate")
        self.declare_parameter("output_pose_topic", "/localized_estimate")
        # This event stream is separate from the high-rate navigation output.
        # It publishes only accepted map-pose candidates for an optional
        # coupled estimator, so the same candidate is not fused repeatedly.
        self.declare_parameter(
            "external_position_topic", "/localization_candidate"
        )
        # Per-update posterior evidence is deliberately separate from the
        # accepted-correction event above.  A scan can be informative enough
        # to diagnose the belief while still being rejected as a navigation
        # correction, so terminal logic must not infer belief quality from
        # correction application alone.
        self.declare_parameter("belief_topic", "/localization_belief")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("match_valid_topic", "/localization_match_valid")
        self.declare_parameter("match_status_topic", "/localization_match_status")
        self.declare_parameter("match_score_topic", "/localization_match_score_m")
        self.declare_parameter("candidate_correction_topic", "/localization_candidate_correction_m")
        self.declare_parameter("candidate_dx_topic", "/localization_candidate_dx_m")
        self.declare_parameter("candidate_dy_topic", "/localization_candidate_dy_m")
        self.declare_parameter("applied_dx_topic", "/localization_applied_dx_m")
        self.declare_parameter("applied_dy_topic", "/localization_applied_dy_m")
        self.declare_parameter("score_improvement_topic", "/localization_score_improvement_m")
        self.declare_parameter("diagnostic_output", "")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("update_rate_hz", 2.0)
        self.declare_parameter("scan_stride", 6)
        self.declare_parameter("particle_count", 500)
        self.declare_parameter("motion_distance_noise_std_m", 0.01)
        self.declare_parameter("motion_yaw_noise_std_rad", 0.01)
        self.declare_parameter("lidar_range_sigma_m", 0.08)
        self.declare_parameter("lidar_likelihood_floor", 1.0e-6)
        self.declare_parameter("resample_ess_ratio", 0.5)
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("initial_position_std_m", 0.05)
        self.declare_parameter("initial_heading_std_rad", 0.10)
        self.declare_parameter("initialization_mode", "local")
        self.declare_parameter("minimum_valid_beams", 6)
        self.declare_parameter("observation_window_size", 3)
        self.declare_parameter("max_scan_pose_age_s", 0.25)
        self.declare_parameter("max_position_std_m", 0.15)
        self.declare_parameter("max_normalized_entropy", 0.98)

        input_topic = str(self.get_parameter("input_pose_topic").value)
        output_topic = str(self.get_parameter("output_pose_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        self.diagnostic_output = str(self.get_parameter("diagnostic_output").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.update_rate_hz = max(0.1, float(self.get_parameter("update_rate_hz").value))
        self.scan_stride = max(1, int(self.get_parameter("scan_stride").value))
        self.initial_position_std_m = max(0.0, float(self.get_parameter("initial_position_std_m").value))
        self.initial_heading_std_rad = max(0.0, float(self.get_parameter("initial_heading_std_rad").value))
        self.initialization_mode = str(
            self.get_parameter("initialization_mode").value
        ).strip().lower()
        if self.initialization_mode not in {"local", "global"}:
            raise ValueError(
                "initialization_mode must be 'local' or 'global'"
            )
        self.minimum_valid_beams = max(1, int(self.get_parameter("minimum_valid_beams").value))
        self.observation_window_size = max(
            1, int(self.get_parameter("observation_window_size").value)
        )
        self.max_scan_pose_age_s = max(
            0.0, float(self.get_parameter("max_scan_pose_age_s").value)
        )
        self.max_position_std_m = max(
            0.0, float(self.get_parameter("max_position_std_m").value)
        )
        self.max_normalized_entropy = min(
            1.0, max(0.0, float(self.get_parameter("max_normalized_entropy").value))
        )
        self.config = MCLConfig(
            particle_count=int(self.get_parameter("particle_count").value),
            motion_distance_noise_std_m=float(
                self.get_parameter("motion_distance_noise_std_m").value
            ),
            motion_yaw_noise_std_rad=float(
                self.get_parameter("motion_yaw_noise_std_rad").value
            ),
            lidar_range_sigma_m=float(self.get_parameter("lidar_range_sigma_m").value),
            lidar_likelihood_floor=float(
                self.get_parameter("lidar_likelihood_floor").value
            ),
            resample_ess_ratio=float(self.get_parameter("resample_ess_ratio").value),
            random_seed=int(float(self.get_parameter("random_seed").value)),
        )

        self.pose_subscription = self.create_subscription(
            Odometry, input_topic, self.pose_callback, qos_profile_sensor_data
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, map_qos
        )
        self.pose_publisher = self.create_publisher(Odometry, output_topic, 10)
        self.external_position_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("external_position_topic").value),
            10,
        )
        self.belief_publisher = self.create_publisher(
            String,
            str(self.get_parameter("belief_topic").value),
            10,
        )
        self.valid_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("match_valid_topic").value),
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("match_status_topic").value),
            10,
        )
        self.score_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("match_score_topic").value),
            10,
        )
        self.candidate_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("candidate_correction_topic").value),
            10,
        )
        self.candidate_dx_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("candidate_dx_topic").value),
            10,
        )
        self.candidate_dy_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("candidate_dy_topic").value),
            10,
        )
        self.applied_dx_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("applied_dx_topic").value),
            10,
        )
        self.applied_dy_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("applied_dy_topic").value),
            10,
        )
        self.score_improvement_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("score_improvement_topic").value),
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self.publish_timer_callback
        )
        self.update_timer = self.create_timer(
            1.0 / self.update_rate_hz, self.update_timer_callback
        )

        self.latest_pose: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        # Keep a short timestamped state history for scan/pose association.
        # The history length is derived from the temporal observation window,
        # not exposed as another estimator tuning parameter.
        self.pose_history: deque[tuple[float, Pose2D]] = deque(
            maxlen=max(20, self.observation_window_size * 10)
        )
        self.pending_scans: deque[tuple[float, LaserScan]] = deque(
            maxlen=max(10, self.observation_window_size * 3)
        )
        self.scan_history: deque[tuple[float, LaserScan, float, Pose2D]] = deque(
            maxlen=self.observation_window_size
        )
        self.latest_map: Optional[OccupancyGridMap] = None
        self.filter: Optional[ParticleFilter2D] = None
        self.last_motion_pose: Optional[Pose2D] = None
        self.last_motion_stamp_s: Optional[float] = None
        self.last_scan_stamp_s: Optional[float] = None
        self.latest_estimate = None
        self.latest_status = "waiting_for_map"
        # A particle prediction is not an accepted absolute localization
        # update.  Downstream planning must not treat an ambiguous or stale
        # MCL posterior as a trustworthy map-frame pose, so the published
        # output falls back to the current state estimate until a valid scan
        # update is available.
        self.latest_measurement_valid = False
        self.latest_score = float("nan")
        self.match_id = 0
        self.diagnostic_handle = None
        if self.diagnostic_output:
            path = Path(self.diagnostic_output)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.diagnostic_handle = path.open("w", encoding="utf-8")
            self.write_diagnostic(
                {
                    "record_type": "config",
                    "backend": "mcl",
                    "config": {
                        "input_pose_topic": input_topic,
                        "motion_prior_contract": (
                            "wheel_imu_only_when_external_fusion_is_enabled"
                        ),
                        "particle_count": self.config.particle_count,
                        "motion_distance_noise_std_m": self.config.motion_distance_noise_std_m,
                        "motion_yaw_noise_std_rad": self.config.motion_yaw_noise_std_rad,
                        "lidar_range_sigma_m": self.config.lidar_range_sigma_m,
                        "lidar_likelihood_floor": self.config.lidar_likelihood_floor,
                        "resample_ess_ratio": self.config.resample_ess_ratio,
                        "random_seed": self.config.random_seed,
                        "initial_position_std_m": self.initial_position_std_m,
                        "initial_heading_std_rad": self.initial_heading_std_rad,
                        "initialization_mode": self.initialization_mode,
                        "minimum_valid_beams": self.minimum_valid_beams,
                        "observation_window_size": self.observation_window_size,
                        "max_scan_pose_age_s": self.max_scan_pose_age_s,
                        "max_position_std_m": self.max_position_std_m,
                        "max_normalized_entropy": self.max_normalized_entropy,
                        "scan_stride": self.scan_stride,
                    },
                    "assumption": (
                        "map and odom coordinates are aligned for the first "
                        "experiment; the input pose is a motion prior, not "
                        "an externally corrected feedback pose"
                    ),
                }
            )

        self.get_logger().info(
            f"MCL: {input_topic} + {scan_topic} + {map_topic} -> "
            f"{output_topic}; particles={self.config.particle_count}, "
            f"update_rate={self.update_rate_hz:.1f} Hz, "
            f"range_sigma={self.config.lidar_range_sigma_m:.3f} m, "
            f"observation_window={self.observation_window_size}, "
            f"initialization={self.initialization_mode}."
        )

    def write_diagnostic(self, record: dict[str, object]) -> None:
        if self.diagnostic_handle is None:
            return
        json.dump(record, self.diagnostic_handle, separators=(",", ":"))
        self.diagnostic_handle.write("\n")
        self.diagnostic_handle.flush()

    def pose_from_message(self, message: Odometry) -> Pose2D:
        pose = message.pose.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def pose_callback(self, message: Odometry) -> None:
        current_pose = self.pose_from_message(message)
        current_stamp_s = stamp_seconds(message)
        self.latest_pose = message
        if (
            not self.pose_history
            or current_stamp_s > self.pose_history[-1][0]
        ):
            self.pose_history.append((current_stamp_s, current_pose))
        self.associate_pending_scans()
        if self.filter is None or self.last_motion_pose is None:
            self.last_motion_pose = current_pose
            self.last_motion_stamp_s = current_stamp_s
            self.try_initialize_filter(current_pose)
            return
        if self.last_motion_stamp_s is not None and current_stamp_s <= self.last_motion_stamp_s:
            return
        previous_pose = self.last_motion_pose
        delta_x = current_pose[0] - previous_pose[0]
        delta_y = current_pose[1] - previous_pose[1]
        delta_distance = delta_x * math.cos(previous_pose[2]) + delta_y * math.sin(previous_pose[2])
        delta_yaw = wrap_angle(current_pose[2] - previous_pose[2])
        self.filter.predict(delta_distance, delta_yaw)
        self.last_motion_pose = current_pose
        self.last_motion_stamp_s = current_stamp_s

    def try_initialize_filter(self, initial_pose: Pose2D) -> None:
        if self.filter is not None or self.latest_map is None:
            return
        self.filter = ParticleFilter2D(self.latest_map, self.config)
        try:
            if self.initialization_mode == "global":
                # A global map prior deliberately does not assume that the
                # odometry pose identifies the correct map branch.  The
                # first scan updates all free-space hypotheses and later
                # motion increments preserve the surviving branches.
                self.filter.initialize_uniform_free()
            else:
                self.filter.initialize_gaussian(
                    initial_pose,
                    self.initial_position_std_m,
                    self.initial_position_std_m,
                    self.initial_heading_std_rad,
                )
        except ValueError:
            # Keep startup deterministic even if the first pose lies on a
            # raster boundary. The measurement update can still recover from
            # this point once valid scan evidence arrives.
            weight = 1.0 / self.config.particle_count
            self.filter.set_particles(
                [
                    self.filter_particle(initial_pose, weight)
                    for _ in range(self.config.particle_count)
                ]
            )
        self.latest_status = "waiting_for_scan"

    @staticmethod
    def filter_particle(pose: Pose2D, weight: float):
        return Particle(pose[0], pose[1], pose[2], weight)

    def map_callback(self, message: OccupancyGrid) -> None:
        self.latest_map = OccupancyGridMap(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution_m=float(message.info.resolution),
            origin_x_m=float(message.info.origin.position.x),
            origin_y_m=float(message.info.origin.position.y),
            occupied_cells=frozenset(
                (column, row)
                for row in range(int(message.info.height))
                for column in range(int(message.info.width))
                if int(message.data[row * int(message.info.width) + column]) >= 50
            ),
        )
        if self.latest_pose is not None:
            self.try_initialize_filter(self.pose_from_message(self.latest_pose))
        self.write_diagnostic(
            {
                "record_type": "map",
                "width": int(message.info.width),
                "height": int(message.info.height),
                "resolution_m": float(message.info.resolution),
                "origin_x_m": float(message.info.origin.position.x),
                "origin_y_m": float(message.info.origin.position.y),
                "data": [int(value) for value in message.data],
            }
        )

    def scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message
        scan_stamp_s = stamp_seconds(message)
        if not math.isfinite(scan_stamp_s):
            return
        newest_known_scan = None
        if self.pending_scans:
            newest_known_scan = self.pending_scans[-1][0]
        if self.scan_history:
            newest_known_scan = max(
                scan_stamp_s if newest_known_scan is None else newest_known_scan,
                self.scan_history[-1][0],
            )
        if newest_known_scan is not None and scan_stamp_s <= newest_known_scan:
            return
        # Do not associate with latest_pose here. The latest pose may still be
        # older than the scan; pose_callback() will associate this measurement
        # once a state sample at or after scan_stamp_s is available.
        self.pending_scans.append((scan_stamp_s, message))
        self.associate_pending_scans()

    def associate_pending_scans(self) -> None:
        """Move scans with valid temporal associations into the MCL window."""
        if not self.pose_history or not self.pending_scans:
            return
        remaining: deque[tuple[float, LaserScan]] = deque(
            maxlen=self.pending_scans.maxlen
        )
        for scan_stamp_s, scan in self.pending_scans:
            scan_pose = interpolate_pose_history(self.pose_history, scan_stamp_s)
            if scan_pose is None:
                remaining.append((scan_stamp_s, scan))
                continue
            self.scan_history.append(
                (scan_stamp_s, scan, scan_stamp_s, scan_pose)
            )
        self.pending_scans = remaining

    @staticmethod
    def make_scan_observation(
        scan: LaserScan,
        scan_pose: Pose2D,
        current_pose: Pose2D,
        *,
        stride: int = 1,
    ) -> MCLScanObservation:
        """Express one historical scan relative to the current odometry pose.

        The particle filter estimates the current pose.  To reuse a previous
        scan, this method records the odometry motion from that scan's pose to
        the current pose in the scan frame.  The filter then applies the
        inverse transform to each current particle before ray scoring.
        """

        delta_x_world = current_pose[0] - scan_pose[0]
        delta_y_world = current_pose[1] - scan_pose[1]
        cos_scan = math.cos(scan_pose[2])
        sin_scan = math.sin(scan_pose[2])
        relative_x = cos_scan * delta_x_world + sin_scan * delta_y_world
        relative_y = -sin_scan * delta_x_world + cos_scan * delta_y_world
        relative_yaw = wrap_angle(current_pose[2] - scan_pose[2])
        beam_stride = max(1, int(stride))
        return MCLScanObservation(
            ranges_m=tuple(float(value) for value in scan.ranges[::beam_stride]),
            angle_min_rad=float(scan.angle_min),
            angle_increment_rad=float(scan.angle_increment) * beam_stride,
            range_min_m=float(scan.range_min),
            range_max_m=float(scan.range_max),
            relative_x_m=relative_x,
            relative_y_m=relative_y,
            relative_yaw_rad=relative_yaw,
        )

    def publish_timer_callback(self) -> None:
        if self.latest_pose is None:
            return
        if (
            self.filter is None
            or self.latest_estimate is None
            or not self.latest_measurement_valid
        ):
            # Preserve the high-rate odometry/state-estimate stream when MCL
            # has no currently accepted scan evidence.  Publishing the last
            # particle mean here would silently let an invalid posterior
            # steer A* and the controller.
            self.publish_pose(self.latest_pose, self.pose_from_message(self.latest_pose), None)
            return

        # The particle set is predicted whenever a new state-estimate pose
        # arrives.  Recompute its mean here so the published header timestamp
        # and the published pose refer to the same current state, rather than
        # relabelling the last LiDAR-update result as a newer pose.
        current_estimate = self.filter.estimate(
            valid_beam_count=self.latest_estimate.valid_beam_count
        )
        self.publish_pose(
            self.latest_pose,
            current_estimate.pose,
            current_estimate.covariance,
        )

    def publish_pose(
        self,
        source: Odometry,
        pose: Pose2D,
        covariance: object,
        *,
        publisher=None,
    ) -> None:
        output = Odometry()
        output.header = source.header
        output.child_frame_id = source.child_frame_id
        output.pose.pose.position.x = pose[0]
        output.pose.pose.position.y = pose[1]
        (
            output.pose.pose.orientation.x,
            output.pose.pose.orientation.y,
            output.pose.pose.orientation.z,
            output.pose.pose.orientation.w,
        ) = quaternion_from_yaw(pose[2])
        output.twist = source.twist
        output.pose.covariance = list(source.pose.covariance)
        if covariance is not None:
            output.pose.covariance[0] = float(covariance[0][0])
            output.pose.covariance[1] = float(covariance[0][1])
            output.pose.covariance[6] = float(covariance[1][0])
            output.pose.covariance[7] = float(covariance[1][1])
            output.pose.covariance[5] = float(covariance[0][2])
            output.pose.covariance[30] = float(covariance[2][0])
            output.pose.covariance[11] = float(covariance[1][2])
            output.pose.covariance[35] = float(covariance[2][2])
        (self.pose_publisher if publisher is None else publisher).publish(output)

    def update_timer_callback(self) -> None:
        if self.filter is None or self.latest_pose is None or not self.scan_history:
            return
        update_start = time.monotonic()
        latest_record = self.scan_history[-1]
        scan_stamp_s, latest_scan, latest_scan_pose_stamp_s, _ = latest_record
        if self.last_scan_stamp_s is not None and scan_stamp_s <= self.last_scan_stamp_s:
            return

        pose_stamp_s = stamp_seconds(self.latest_pose)
        scan_pose_age_s = pose_stamp_s - scan_stamp_s
        valid_range_count = self.valid_scan_beam_count(latest_scan, stride=self.scan_stride)
        if scan_pose_age_s > self.max_scan_pose_age_s:
            self.last_scan_stamp_s = scan_stamp_s
            self.reject_scan_update(
                status="stale_scan",
                scan_stamp_s=scan_stamp_s,
                pose_stamp_s=pose_stamp_s,
                scan_pose_age_s=scan_pose_age_s,
                valid_range_count=valid_range_count,
                update_start=update_start,
            )
            return
        if scan_pose_age_s < -0.05 or latest_scan_pose_stamp_s - scan_stamp_s < -0.05:
            self.last_scan_stamp_s = scan_stamp_s
            self.reject_scan_update(
                status="scan_ahead_of_pose",
                scan_stamp_s=scan_stamp_s,
                pose_stamp_s=pose_stamp_s,
                scan_pose_age_s=scan_pose_age_s,
                valid_range_count=valid_range_count,
                update_start=update_start,
            )
            return
        if valid_range_count < self.minimum_valid_beams:
            self.last_scan_stamp_s = scan_stamp_s
            self.reject_scan_update(
                status="insufficient_points",
                scan_stamp_s=scan_stamp_s,
                pose_stamp_s=pose_stamp_s,
                scan_pose_age_s=scan_pose_age_s,
                valid_range_count=valid_range_count,
                update_start=update_start,
            )
            return

        current_pose = self.pose_from_message(self.latest_pose)
        observations: list[MCLScanObservation] = []
        observation_scans: list[LaserScan] = []
        for (
            historical_scan_stamp_s,
            historical_scan,
            historical_pose_stamp_s,
            historical_pose,
        ) in list(self.scan_history)[-self.observation_window_size :]:
            historical_scan_pose_age_s = historical_pose_stamp_s - historical_scan_stamp_s
            if (
                historical_scan_pose_age_s > self.max_scan_pose_age_s
                or historical_scan_pose_age_s < -0.05
            ):
                continue
            if (
                self.valid_scan_beam_count(historical_scan, stride=self.scan_stride)
                < self.minimum_valid_beams
            ):
                continue
            observations.append(
                self.make_scan_observation(
                    historical_scan,
                    historical_pose,
                    current_pose,
                    stride=self.scan_stride,
                )
            )
            observation_scans.append(historical_scan)
        if not observations:
            self.last_scan_stamp_s = scan_stamp_s
            self.reject_scan_update(
                status="insufficient_temporal_evidence",
                scan_stamp_s=scan_stamp_s,
                pose_stamp_s=pose_stamp_s,
                scan_pose_age_s=scan_pose_age_s,
                valid_range_count=valid_range_count,
                update_start=update_start,
                observation_count=0,
            )
            return

        prior_particles = list(self.filter.particles)
        source_pose = current_pose
        # Keep the odometry/state-estimate hypothesis as an explicit reference.
        # The particle filter can produce a compact posterior around a wrong
        # map alias, so posterior spread alone is not evidence that the LiDAR
        # candidate is better than the continuously propagated state.  Record
        # both likelihoods first; this is diagnostic only and does not alter
        # the acceptance decision in this step.
        prior_log_likelihood, _ = self.filter.pose_log_likelihood_sequence(
            source_pose, observations
        )
        result = self.filter.update_sequence(observations)
        self.last_scan_stamp_s = scan_stamp_s
        dx = result.pose[0] - source_pose[0]
        dy = result.pose[1] - source_pose[1]
        correction = math.hypot(dx, dy)
        candidate_log_likelihood, _ = self.filter.pose_log_likelihood_sequence(
            result.pose, observations
        )
        log_likelihood_gain = candidate_log_likelihood - prior_log_likelihood
        source_covariance_xy = self.planar_covariance_from_odometry(self.latest_pose)
        correction_mahalanobis_sq = self.correction_mahalanobis_sq(
            (dx, dy), source_covariance_xy, result.covariance
        )
        # This is a diagnostic posterior comparison only.  It combines the
        # LiDAR evidence with an approximate odometry prior without changing
        # the current acceptance policy or introducing a new tuning gate.
        bayesian_log_gain = (
            None
            if correction_mahalanobis_sq is None
            else log_likelihood_gain - 0.5 * correction_mahalanobis_sq
        )
        position_std_m = max(
            math.sqrt(max(0.0, result.covariance[0][0])),
            math.sqrt(max(0.0, result.covariance[1][1])),
        )
        valid = all(
            self.valid_scan_beam_count(scan, stride=self.scan_stride)
            >= self.minimum_valid_beams
            for scan in observation_scans
        )
        high_entropy_large_correction = (
            result.normalized_entropy > self.max_normalized_entropy
            and correction > self.max_position_std_m
        )
        if valid and (
            position_std_m > self.max_position_std_m
            or high_entropy_large_correction
        ):
            # Keep the candidate for diagnostics, but do not let a broad
            # posterior steer the planner. Restore the prior posterior so
            # this rejected scan cannot contaminate later predictions.
            self.filter.set_particles(prior_particles)
            valid = False
            self.latest_status = "candidate_uncertainty_too_large"
        elif valid and log_likelihood_gain <= 1.0e-12:
            # A compact posterior is not enough to justify an absolute
            # correction.  If the candidate is no better than the continuously
            # propagated state under the same scan window, the observation is
            # not discriminative at this pose.  Restore the prior so a flat or
            # worse likelihood cannot move the navigation pose.
            self.filter.set_particles(prior_particles)
            valid = False
            self.latest_status = "candidate_not_better_than_prior"
        elif (
            self.initialization_mode == "local"
            and valid
            and bayesian_log_gain is not None
            and bayesian_log_gain <= 0.0
        ):
            # A local map alias can improve the LiDAR likelihood while moving
            # beyond the uncertainty carried by the propagated state.  The
            # zero boundary here is the posterior model comparison itself,
            # not a manually tuned correction threshold.  Roll back the
            # particle update so an unsupported absolute correction cannot
            # steer downstream navigation.
            self.filter.set_particles(prior_particles)
            valid = False
            self.latest_status = "candidate_inconsistent_with_prior"
        else:
            self.latest_status = "accepted" if valid else "insufficient_points"
            if valid:
                self.latest_estimate = result
        self.latest_measurement_valid = valid
        self.latest_score = -candidate_log_likelihood
        self.publish_match_diagnostics(valid, self.latest_status, correction, dx, dy)
        self.publish_belief_event(
            match_id=self.match_id,
            scan_stamp_s=scan_stamp_s,
            pose_stamp_s=pose_stamp_s,
            scan_pose_age_s=scan_pose_age_s,
            observation_count=len(observations),
            status=self.latest_status,
            measurement_applied=valid,
            pose=result.pose,
            covariance=result.covariance,
            correction_m=correction,
            dx_m=dx,
            dy_m=dy,
            valid_beam_count=result.valid_beam_count,
            effective_sample_size=result.effective_sample_size,
            maximum_weight=result.maximum_weight,
            normalized_entropy=result.normalized_entropy,
            log_likelihood_gain=log_likelihood_gain,
            correction_mahalanobis_sq=correction_mahalanobis_sq,
            bayesian_log_gain=bayesian_log_gain,
            source_covariance_xy=source_covariance_xy,
            resampled=result.resampled,
            update_latency_s=time.monotonic() - update_start,
        )
        if valid:
            # Publish exactly one accepted candidate per scan update.  The
            # regular /localized_estimate stream may repeat the latest
            # posterior at 30 Hz; this event stream prevents repeated
            # Bayesian updates from the same LiDAR evidence.
            self.publish_pose(
                self.latest_pose,
                result.pose,
                result.covariance,
                publisher=self.external_position_publisher,
            )
        self.write_diagnostic(
            {
                "record_type": "mcl_update",
                "match_id": self.match_id,
                "scan_stamp_s": scan_stamp_s,
                "pose_stamp_s": pose_stamp_s,
                "scan_pose_age_s": scan_pose_age_s,
                "observation_count": len(observations),
                "update_latency_s": time.monotonic() - update_start,
                "measurement_applied": valid,
                "candidate_pose": list(result.pose),
                "source_pose": list(source_pose),
                "pose": list(result.pose if valid else self.filter.estimate().pose),
                "candidate_covariance": [list(row) for row in result.covariance],
                "candidate_position_std_m": position_std_m,
                "correction_m": correction,
                "dx_m": dx,
                "dy_m": dy,
                "prior_log_likelihood": prior_log_likelihood,
                "candidate_log_likelihood": candidate_log_likelihood,
                "log_likelihood_gain": log_likelihood_gain,
                "source_covariance_xy": source_covariance_xy,
                "correction_mahalanobis_sq": correction_mahalanobis_sq,
                "bayesian_log_gain": bayesian_log_gain,
                "valid_beam_count": result.valid_beam_count,
                "effective_sample_size": result.effective_sample_size,
                "maximum_weight": result.maximum_weight,
                "normalized_entropy": result.normalized_entropy,
                "resampled": result.resampled,
                "status": self.latest_status,
            }
        )
        self.match_id += 1

    @staticmethod
    def valid_scan_beam_count(scan: LaserScan, *, stride: int = 1) -> int:
        """Count finite in-range returns before touching particle weights."""

        return sum(
            1
            for range_m in scan.ranges[:: max(1, int(stride))]
            if math.isfinite(range_m)
            and float(scan.range_min) < float(range_m) < float(scan.range_max)
        )

    @staticmethod
    def planar_covariance_from_odometry(
        message: Odometry,
    ) -> Optional[list[list[float]]]:
        """Extract a finite, positive-definite x/y covariance from odometry.

        This is used only by the Bayesian consistency diagnostic.  A missing,
        non-finite, or singular covariance must not silently become a very
        confident prior, so those cases return ``None``.  The ROS covariance
        indices are ``xx=0``, ``xy=1``, ``yx=6``, and ``yy=7``.
        """

        raw = list(message.pose.covariance)
        if len(raw) < 8:
            return None
        xx, xy, yx, yy = (float(raw[index]) for index in (0, 1, 6, 7))
        if not all(math.isfinite(value) for value in (xx, xy, yx, yy)):
            return None
        # Message conversion can introduce tiny xy/yx asymmetry.  A physical
        # covariance is symmetric, so use the symmetric cross term.
        cross = 0.5 * (xy + yx)
        determinant = xx * yy - cross * cross
        if xx <= 0.0 or yy <= 0.0 or determinant <= 1.0e-12:
            return None
        return [[xx, cross], [cross, yy]]

    @staticmethod
    def correction_mahalanobis_sq(
        correction_xy: tuple[float, float],
        source_covariance: Optional[list[list[float]]],
        candidate_covariance: object,
    ) -> Optional[float]:
        """Compute a covariance-weighted distance between two pose hypotheses.

        The source and candidate uncertainties are combined as an approximate
        independent sum.  This is intentionally diagnostic-only for now: the
        MCL posterior is initialized from the source estimate, so strict
        independence is not true.  It is still useful for exposing a compact
        map alias that lies far outside the uncertainty of the propagated
        state.
        """

        if source_covariance is None:
            return None
        try:
            candidate_xx = float(candidate_covariance[0][0])
            candidate_xy = float(candidate_covariance[0][1])
            candidate_yx = float(candidate_covariance[1][0])
            candidate_yy = float(candidate_covariance[1][1])
        except (IndexError, TypeError, ValueError):
            return None
        values = (candidate_xx, candidate_xy, candidate_yx, candidate_yy)
        if not all(math.isfinite(value) for value in values):
            return None
        cross = 0.5 * (candidate_xy + candidate_yx)
        xx = source_covariance[0][0] + candidate_xx
        xy = source_covariance[0][1] + cross
        yy = source_covariance[1][1] + candidate_yy
        determinant = xx * yy - xy * xy
        if xx <= 0.0 or yy <= 0.0 or determinant <= 1.0e-12:
            return None
        dx, dy = correction_xy
        numerator = yy * dx * dx - 2.0 * xy * dx * dy + xx * dy * dy
        result = numerator / determinant
        return result if math.isfinite(result) and result >= 0.0 else None

    def reject_scan_update(
        self,
        *,
        status: str,
        scan_stamp_s: float,
        pose_stamp_s: float,
        scan_pose_age_s: float,
        valid_range_count: int,
        update_start: float,
        observation_count: int = 1,
    ) -> None:
        """Record a rejected scan without modifying the particle posterior."""

        self.latest_status = status
        self.latest_measurement_valid = False
        self.latest_score = float("nan")
        current_result = self.filter.estimate() if self.filter is not None else None
        pose = current_result.pose if current_result is not None else self.pose_from_message(self.latest_pose)
        covariance = current_result.covariance if current_result is not None else None
        self.publish_match_diagnostics(False, status, 0.0, 0.0, 0.0)
        self.publish_belief_event(
            match_id=self.match_id,
            scan_stamp_s=scan_stamp_s,
            pose_stamp_s=pose_stamp_s,
            scan_pose_age_s=scan_pose_age_s,
            observation_count=observation_count,
            status=status,
            measurement_applied=False,
            pose=pose,
            covariance=covariance,
            correction_m=0.0,
            dx_m=0.0,
            dy_m=0.0,
            valid_beam_count=valid_range_count,
            effective_sample_size=(
                current_result.effective_sample_size
                if current_result is not None
                else 0.0
            ),
            maximum_weight=(
                current_result.maximum_weight if current_result is not None else 0.0
            ),
            normalized_entropy=(
                current_result.normalized_entropy if current_result is not None else 0.0
            ),
            log_likelihood_gain=None,
            correction_mahalanobis_sq=None,
            bayesian_log_gain=None,
            source_covariance_xy=(
                self.planar_covariance_from_odometry(self.latest_pose)
                if self.latest_pose is not None
                else None
            ),
            resampled=False,
            update_latency_s=time.monotonic() - update_start,
        )
        self.write_diagnostic(
            {
                "record_type": "mcl_update",
                "match_id": self.match_id,
                "scan_stamp_s": scan_stamp_s,
                "pose_stamp_s": pose_stamp_s,
                "scan_pose_age_s": scan_pose_age_s,
                "observation_count": observation_count,
                "update_latency_s": time.monotonic() - update_start,
                "measurement_applied": False,
                "pose": list(pose),
                "covariance": covariance,
                "correction_m": 0.0,
                "dx_m": 0.0,
                "dy_m": 0.0,
                "valid_beam_count": valid_range_count,
                "effective_sample_size": (
                    current_result.effective_sample_size
                    if current_result is not None
                    else 0.0
                ),
                "maximum_weight": (
                    current_result.maximum_weight if current_result is not None else 0.0
                ),
                "normalized_entropy": (
                    current_result.normalized_entropy if current_result is not None else 0.0
                ),
                "source_covariance_xy": (
                    self.planar_covariance_from_odometry(self.latest_pose)
                    if self.latest_pose is not None
                    else None
                ),
                "correction_mahalanobis_sq": None,
                "bayesian_log_gain": None,
                "resampled": False,
                "status": status,
            }
        )
        self.match_id += 1

    def publish_belief_event(
        self,
        *,
        match_id: int,
        scan_stamp_s: float,
        pose_stamp_s: float,
        scan_pose_age_s: float,
        observation_count: int,
        status: str,
        measurement_applied: bool,
        pose: Pose2D,
        covariance: object,
        correction_m: float,
        dx_m: float,
        dy_m: float,
        valid_beam_count: int,
        effective_sample_size: float,
        maximum_weight: float,
        normalized_entropy: float,
        log_likelihood_gain: Optional[float],
        correction_mahalanobis_sq: Optional[float],
        bayesian_log_gain: Optional[float],
        source_covariance_xy: Optional[list[list[float]]],
        resampled: bool,
        update_latency_s: float,
    ) -> None:
        """Publish one timestamped MCL posterior-quality event.

        ``/localization_candidate`` means that a correction was accepted for
        downstream fusion.  ``/localization_belief`` is broader: it reports
        every attempted scan update, including rejected candidates, together
        with the posterior quality indicators needed to distinguish weak,
        ambiguous, stale, and informative observations.  Keeping these
        streams separate prevents a controller from treating correction
        application as the only definition of localization evidence.
        """
        covariance_xy = None
        if covariance is not None:
            covariance_xy = [
                [float(covariance[0][0]), float(covariance[0][1])],
                [float(covariance[1][0]), float(covariance[1][1])],
            ]
        event = {
            "record_type": "mcl_belief",
            "match_id": int(match_id),
            "scan_stamp_s": float(scan_stamp_s),
            "pose_stamp_s": float(pose_stamp_s),
            "scan_pose_age_s": float(scan_pose_age_s),
            "observation_count": int(observation_count),
            "status": str(status),
            "measurement_applied": bool(measurement_applied),
            "pose": [float(pose[0]), float(pose[1]), float(pose[2])],
            "covariance_xy": covariance_xy,
            "correction_m": float(correction_m),
            "dx_m": float(dx_m),
            "dy_m": float(dy_m),
            "valid_beam_count": int(valid_beam_count),
            "effective_sample_size": float(effective_sample_size),
            "maximum_weight": float(maximum_weight),
            "normalized_entropy": float(normalized_entropy),
            "log_likelihood_gain": (
                None
                if log_likelihood_gain is None
                else float(log_likelihood_gain)
            ),
            "source_covariance_xy": source_covariance_xy,
            "correction_mahalanobis_sq": (
                None
                if correction_mahalanobis_sq is None
                else float(correction_mahalanobis_sq)
            ),
            "bayesian_log_gain": (
                None
                if bayesian_log_gain is None
                else float(bayesian_log_gain)
            ),
            "resampled": bool(resampled),
            "update_latency_s": float(update_latency_s),
        }
        message = String()
        message.data = json.dumps(event, separators=(",", ":"))
        self.belief_publisher.publish(message)
        # Keep the ROS event stream and the offline audit stream identical.
        # ``mcl_update`` remains the compact compatibility record; this
        # separate record preserves posterior evidence even when a candidate
        # is rejected as a navigation correction.
        self.write_diagnostic(event)

    def publish_match_diagnostics(
        self,
        valid: bool,
        status: str,
        correction: float,
        dx: float,
        dy: float,
    ) -> None:
        valid_message = Bool()
        valid_message.data = valid
        self.valid_publisher.publish(valid_message)
        status_message = String()
        status_message.data = status
        self.status_publisher.publish(status_message)
        for publisher, value in (
            (self.score_publisher, self.latest_score),
            (self.candidate_publisher, correction),
            (self.candidate_dx_publisher, dx),
            (self.candidate_dy_publisher, dy),
            (self.applied_dx_publisher, dx if valid else 0.0),
            (self.applied_dy_publisher, dy if valid else 0.0),
            (self.score_improvement_publisher, 0.0),
        ):
            message = Float64()
            message.data = float(value)
            publisher.publish(message)

    def destroy_node(self) -> bool:
        if self.diagnostic_handle is not None:
            self.diagnostic_handle.close()
            self.diagnostic_handle = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MCLLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # SIGINT is the normal way ros2 launch stops a long-running
        # localization experiment; it is not an estimator failure.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
