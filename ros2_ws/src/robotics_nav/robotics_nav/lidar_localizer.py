#!/usr/bin/env python3
"""Publish an experimental local LiDAR-to-map position correction."""

from __future__ import annotations

import copy
import concurrent.futures
import json
import math
import multiprocessing
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
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
from tf2_ros import TransformBroadcaster

from robotics_nav.lidar_localization import (
    LidarMapMatcher,
    Pose2D,
    interpolate_pose,
    map_odom_from_poses,
    transform_pose,
)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def rotate_planar_covariance(
    covariance_xy: tuple[float, float, float],
    yaw: float,
) -> tuple[float, float, float]:
    """Rotate ``(Pxx, Pxy, Pyy)`` into a frame rotated by ``yaw``.

    ``nav_msgs/Odometry`` reports the position covariance in the pose frame.
    The local scan matcher evaluates candidate displacements in the map
    frame, so the covariance must be rotated before it is used for an offline
    Mahalanobis diagnostic.
    """
    pxx, pxy, pyy = covariance_xy
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotated_xx = cosine * cosine * pxx - 2.0 * cosine * sine * pxy + sine * sine * pyy
    rotated_xy = (
        cosine * sine * pxx
        + (cosine * cosine - sine * sine) * pxy
        - sine * cosine * pyy
    )
    rotated_yy = sine * sine * pxx + 2.0 * sine * cosine * pxy + cosine * cosine * pyy
    return rotated_xx, rotated_xy, rotated_yy


def mahalanobis_squared_2d(
    displacement: tuple[float, float],
    covariance_xy: tuple[float, float, float] | None,
) -> float | None:
    """Return the 2-D covariance-normalized displacement, if invertible.

    This is diagnostic only at present.  The LiDAR matcher does not use this
    value to rank candidates until the published odometry covariance has been
    calibrated against representative motion errors.
    """
    if covariance_xy is None or not all(math.isfinite(value) for value in covariance_xy):
        return None
    pxx, pxy, pyy = covariance_xy
    determinant = pxx * pyy - pxy * pxy
    if determinant <= 1.0e-12:
        return None
    dx, dy = displacement
    return (pyy * dx * dx - 2.0 * pxy * dx * dy + pxx * dy * dy) / determinant


def run_match_job(
    job: tuple[LidarMapMatcher, dict[str, object]],
) -> dict[str, object]:
    """Evaluate one immutable scan snapshot in a separate process.

    The matcher is intentionally a transparent Python implementation and is
    CPU-bound during ray casting.  A thread would still compete for the
    interpreter lock with ROS callbacks, so the worker process is used to keep
    high-rate pose relay independent of matching cost.
    """
    matcher, snapshot = job
    worker_start = time.monotonic()
    map_x, map_y, map_yaw = snapshot["map_pose"]  # type: ignore[misc]
    ranges = snapshot["ranges"]
    x, y, matched_yaw, score, point_count = matcher.match_pose(
        map_x,
        map_y,
        map_yaw,
        ranges,
        angle_min=float(snapshot["angle_min"]),
        angle_increment=float(snapshot["angle_increment"]),
        range_min=float(snapshot["range_min"]),
        range_max=float(snapshot["range_max"]),
    )
    prior_score, _ = matcher.score_pose(
        map_x,
        map_y,
        map_yaw,
        ranges,
        angle_min=float(snapshot["angle_min"]),
        angle_increment=float(snapshot["angle_increment"]),
        range_min=float(snapshot["range_min"]),
        range_max=float(snapshot["range_max"]),
    )
    search_diagnostics = dict(matcher.last_match_diagnostics)
    best_regularized_score = search_diagnostics.get(
        "best_regularized_score_m"
    )
    if (
        isinstance(best_regularized_score, (int, float))
        and math.isfinite(float(best_regularized_score))
        and math.isfinite(prior_score)
    ):
        search_diagnostics["regularized_score_improvement_m"] = (
            prior_score - float(best_regularized_score)
        )
    else:
        search_diagnostics["regularized_score_improvement_m"] = None
    search_diagnostics["prior_residual_m"] = (
        prior_score if math.isfinite(prior_score) else None
    )
    search_diagnostics["best_residual_m"] = (
        score if math.isfinite(score) else None
    )
    return {
        "diagnostic_match_id": snapshot.get("diagnostic_match_id", -1),
        "odom_pose": snapshot["odom_pose"],
        "map_pose": snapshot["map_pose"],
        "position_covariance_xy": snapshot.get("position_covariance_xy"),
        "map_odom_yaw": snapshot.get("map_odom_yaw", 0.0),
        "scan_stamp_s": snapshot.get("scan_stamp_s"),
        "pose_stamp_s": snapshot.get("pose_stamp_s"),
        "x": x,
        "y": y,
        "matched_yaw": matched_yaw,
        "score": score,
        "prior_score": prior_score,
        "point_count": point_count,
        "search_diagnostics": search_diagnostics,
        "submitted_monotonic_s": snapshot.get("submitted_monotonic_s"),
        "worker_compute_time_s": time.monotonic() - worker_start,
    }


class LidarLocalizer(Node):
    """Correct local wheel/IMU position drift using the known static map."""

    def __init__(self) -> None:
        super().__init__("lidar_localizer")

        self.declare_parameter("input_pose_topic", "/state_estimate")
        self.declare_parameter("output_pose_topic", "/localized_estimate")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("correction_topic", "/localization_correction_m")
        self.declare_parameter("match_valid_topic", "/localization_match_valid")
        self.declare_parameter("match_status_topic", "/localization_match_status")
        self.declare_parameter("match_score_topic", "/localization_match_score_m")
        self.declare_parameter(
            "candidate_correction_topic",
            "/localization_candidate_correction_m",
        )
        self.declare_parameter(
            "candidate_dx_topic",
            "/localization_candidate_dx_m",
        )
        self.declare_parameter(
            "candidate_dy_topic",
            "/localization_candidate_dy_m",
        )
        self.declare_parameter(
            "applied_dx_topic",
            "/localization_applied_dx_m",
        )
        self.declare_parameter(
            "applied_dy_topic",
            "/localization_applied_dy_m",
        )
        self.declare_parameter(
            "heading_correction_topic",
            "/localization_heading_correction_rad",
        )
        self.declare_parameter(
            "score_improvement_topic",
            "/localization_score_improvement_m",
        )
        self.declare_parameter("search_radius_m", 0.35)
        self.declare_parameter("search_step_m", 0.025)
        self.declare_parameter("scan_stride", 6)
        self.declare_parameter("max_match_distance_m", 0.25)
        self.declare_parameter("prior_weight", 0.08)
        self.declare_parameter("score_mode", "range")
        self.declare_parameter("optimizer_mode", "grid")
        self.declare_parameter("refine_top_k", 5)
        self.declare_parameter("minimum_points", 6)
        self.declare_parameter("max_correction_m", 0.15)
        self.declare_parameter("max_total_correction_m", 0.20)
        self.declare_parameter("robot_radius_m", 0.35)
        self.declare_parameter("max_match_score_m", 0.12)
        self.declare_parameter("correction_smoothing", 0.25)
        # A non-zero value is diagnostic only: matched yaw is logged but is
        # deliberately not applied to the persistent map->odom correction.
        self.declare_parameter("yaw_search_radius_rad", 0.0)
        self.declare_parameter("yaw_search_step_rad", 0.05)
        self.declare_parameter("yaw_prior_weight", 0.02)
        self.declare_parameter("max_heading_correction_rad", 0.25)
        self.declare_parameter("minimum_score_improvement_m", 0.005)
        self.declare_parameter("minimum_consecutive_matches", 3)
        self.declare_parameter("candidate_consistency_m", 0.05)
        self.declare_parameter("minimum_reapplication_change_m", 0.05)
        # A worker result is only useful if it still describes approximately
        # the pose at which its scan was acquired.  These limits expose the
        # asynchronous timing assumptions as explicit safety gates instead of
        # silently applying a result after the robot has moved on.
        self.declare_parameter("max_match_age_s", 1.0)
        self.declare_parameter("max_odom_motion_during_match_m", 0.10)
        self.declare_parameter("max_odom_yaw_change_during_match_rad", 0.35)
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("broadcast_map_odom_tf", True)
        # Pose publication and scan matching have different timing needs.  The
        # controller needs a fresh pose at sensor/control rate, while the
        # deliberately simple Python matcher is much slower.  Keep them as
        # separate rates so a slow search cannot stall navigation.
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("match_rate_hz", 2.0)
        self.declare_parameter("diagnostic_output", "")

        input_topic = str(self.get_parameter("input_pose_topic").value)
        output_topic = str(self.get_parameter("output_pose_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        correction_topic = str(self.get_parameter("correction_topic").value)
        match_valid_topic = str(self.get_parameter("match_valid_topic").value)
        match_status_topic = str(self.get_parameter("match_status_topic").value)
        match_score_topic = str(self.get_parameter("match_score_topic").value)
        candidate_correction_topic = str(
            self.get_parameter("candidate_correction_topic").value
        )
        candidate_dx_topic = str(
            self.get_parameter("candidate_dx_topic").value
        )
        candidate_dy_topic = str(
            self.get_parameter("candidate_dy_topic").value
        )
        applied_dx_topic = str(
            self.get_parameter("applied_dx_topic").value
        )
        applied_dy_topic = str(
            self.get_parameter("applied_dy_topic").value
        )
        heading_correction_topic = str(
            self.get_parameter("heading_correction_topic").value
        )
        score_improvement_topic = str(
            self.get_parameter("score_improvement_topic").value
        )
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.match_rate = max(
            0.1, float(self.get_parameter("match_rate_hz").value)
        )
        self.match_interval_s = 1.0 / self.match_rate
        self.diagnostic_output = str(
            self.get_parameter("diagnostic_output").value
        )
        self.max_correction_m = max(
            0.0, float(self.get_parameter("max_correction_m").value)
        )
        self.max_total_correction_m = max(
            0.0,
            float(self.get_parameter("max_total_correction_m").value),
        )
        self.robot_radius_m = max(
            0.0,
            float(self.get_parameter("robot_radius_m").value),
        )
        self.max_match_score_m = max(
            0.0, float(self.get_parameter("max_match_score_m").value)
        )
        self.correction_smoothing = min(
            1.0, max(0.0, float(self.get_parameter("correction_smoothing").value))
        )
        self.minimum_consecutive_matches = max(
            1, int(self.get_parameter("minimum_consecutive_matches").value)
        )
        self.candidate_consistency_m = max(
            0.0, float(self.get_parameter("candidate_consistency_m").value)
        )
        self.minimum_reapplication_change_m = max(
            0.0,
            float(
                self.get_parameter("minimum_reapplication_change_m").value
            ),
        )
        self.max_match_age_s = max(
            0.0, float(self.get_parameter("max_match_age_s").value)
        )
        self.max_odom_motion_during_match_m = max(
            0.0,
            float(
                self.get_parameter("max_odom_motion_during_match_m").value
            ),
        )
        self.max_odom_yaw_change_during_match_rad = max(
            0.0,
            float(
                self.get_parameter(
                    "max_odom_yaw_change_during_match_rad"
                ).value
            ),
        )
        self.output_frame_id = str(
            self.get_parameter("output_frame_id").value
        )
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.broadcast_map_odom_tf = str(
            self.get_parameter("broadcast_map_odom_tf").value
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.max_heading_correction_rad = max(
            0.0, float(self.get_parameter("max_heading_correction_rad").value)
        )
        self.minimum_score_improvement_m = max(
            0.0, float(self.get_parameter("minimum_score_improvement_m").value)
        )
        self.matcher = LidarMapMatcher(
            search_radius_m=float(self.get_parameter("search_radius_m").value),
            search_step_m=float(self.get_parameter("search_step_m").value),
            scan_stride=int(self.get_parameter("scan_stride").value),
            max_match_distance_m=float(
                self.get_parameter("max_match_distance_m").value
            ),
            prior_weight=float(self.get_parameter("prior_weight").value),
            score_mode=str(self.get_parameter("score_mode").value),
            optimizer_mode=str(self.get_parameter("optimizer_mode").value),
            refine_top_k=int(self.get_parameter("refine_top_k").value),
            yaw_search_radius_rad=float(
                self.get_parameter("yaw_search_radius_rad").value
            ),
            yaw_search_step_rad=float(
                self.get_parameter("yaw_search_step_rad").value
            ),
            yaw_prior_weight=float(self.get_parameter("yaw_prior_weight").value),
            minimum_points=int(self.get_parameter("minimum_points").value),
        )
        # The matcher is read-only during a search, but the map callback can
        # arrive while a worker is evaluating a snapshot.  This lock prevents
        # ray-casting from observing a partially replaced map.
        self.matcher_lock = threading.Lock()

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            map_qos,
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.broadcast_map_odom_tf else None
        )
        self.correction_publisher = self.create_publisher(
            Float64, correction_topic, 10
        )
        self.match_valid_publisher = self.create_publisher(
            Bool, match_valid_topic, 10
        )
        self.match_status_publisher = self.create_publisher(
            String, match_status_topic, 10
        )
        self.match_score_publisher = self.create_publisher(
            Float64, match_score_topic, 10
        )
        self.candidate_correction_publisher = self.create_publisher(
            Float64, candidate_correction_topic, 10
        )
        # Publish signed components as well as the scalar magnitude.  The
        # magnitude answers "how large?", while dx/dy reveal whether the
        # matcher is applying a coherent correction or oscillating between
        # competing local map matches.
        self.candidate_dx_publisher = self.create_publisher(
            Float64, candidate_dx_topic, 10
        )
        self.candidate_dy_publisher = self.create_publisher(
            Float64, candidate_dy_topic, 10
        )
        self.applied_dx_publisher = self.create_publisher(
            Float64, applied_dx_topic, 10
        )
        self.applied_dy_publisher = self.create_publisher(
            Float64, applied_dy_topic, 10
        )
        self.heading_correction_publisher = self.create_publisher(
            Float64, heading_correction_topic, 10
        )
        self.score_improvement_publisher = self.create_publisher(
            Float64, score_improvement_topic, 10
        )
        self.timer = self.create_timer(
            1.0 / max(0.1, self.publish_rate), self.publish_estimate
        )
        self.match_executor = concurrent.futures.ProcessPoolExecutor(
            max_workers=1,
            mp_context=multiprocessing.get_context("spawn"),
        )
        self.match_future: Optional[concurrent.futures.Future] = None
        self.last_match_submit_time = 0.0

        self.latest_pose: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        self.last_status: Optional[str] = None
        self.valid_match_streak = 0
        self.previous_candidate_correction: Optional[tuple[float, float, float]] = None
        # A candidate is a local measurement, not a velocity command.  Keep
        # the last applied innovation so an unchanged local minimum is not
        # integrated repeatedly into the persistent map->odom transform.
        self.last_applied_candidate_correction: Optional[
            tuple[float, float, float]
        ] = None
        # The localizer estimates a persistent map->odom transform.  A scan
        # match updates this transform; rejected matches keep the last valid
        # transform and therefore do not snap the published pose back to raw
        # odometry.
        self.map_odom_transform: Pose2D = (0.0, 0.0, 0.0)
        # Measure accumulated translation relative to the startup transform.
        # This prevents many small accepted updates from moving the map frame
        # without a bounded external correction.
        self.initial_map_odom_transform: Pose2D = self.map_odom_transform
        self.last_match_result: Optional[dict[str, object]] = None
        self.diagnostic_match_id = 0
        self.diagnostic_map_written = False
        self.diagnostic_handle = None
        if self.diagnostic_output:
            diagnostic_path = Path(self.diagnostic_output)
            diagnostic_path.parent.mkdir(parents=True, exist_ok=True)
            self.diagnostic_handle = diagnostic_path.open(
                "w", encoding="utf-8"
            )
            self._write_diagnostic(
                {
                    "record_type": "config",
                    "matcher": {
                        "search_radius_m": self.matcher.search_radius_m,
                        "search_step_m": self.matcher.search_step_m,
                        "scan_stride": self.matcher.scan_stride,
                        "max_match_distance_m": self.matcher.max_match_distance_m,
                        "prior_weight": self.matcher.prior_weight,
                        "score_mode": self.matcher.score_mode,
                        "optimizer_mode": self.matcher.optimizer_mode,
                        "refine_top_k": self.matcher.refine_top_k,
                        "yaw_search_radius_rad": self.matcher.yaw_search_radius_rad,
                        "yaw_search_step_rad": self.matcher.yaw_search_step_rad,
                        "yaw_prior_weight": self.matcher.yaw_prior_weight,
                        "minimum_points": self.matcher.minimum_points,
                        "ignore_outer_boundary": self.matcher.ignore_outer_boundary,
                    },
                    "gates": {
                        "max_correction_m": self.max_correction_m,
                        "max_total_correction_m": self.max_total_correction_m,
                        "robot_radius_m": self.robot_radius_m,
                        "max_match_score_m": self.max_match_score_m,
                        "max_heading_correction_rad": self.max_heading_correction_rad,
                        "minimum_score_improvement_m": self.minimum_score_improvement_m,
                        "minimum_consecutive_matches": self.minimum_consecutive_matches,
                        "candidate_consistency_m": self.candidate_consistency_m,
                        "minimum_reapplication_change_m": self.minimum_reapplication_change_m,
                        "correction_smoothing": self.correction_smoothing,
                    },
                }
            )

        self.get_logger().info(
            f"LiDAR localizer: {input_topic} + {scan_topic} + {map_topic} "
            f"-> {output_topic}; search_radius="
            f"{self.matcher.search_radius_m:.3f} m, max_correction="
            f"{self.max_correction_m:.3f} m, max_score="
            f"{self.max_match_score_m:.3f} m, smoothing="
            f"{self.correction_smoothing:.2f}, consecutive_matches="
            f"{self.minimum_consecutive_matches}, yaw_search="
            f"{self.matcher.yaw_search_radius_rad:.3f} rad, publish_rate="
            f"{self.publish_rate:.1f} Hz, match_rate={self.match_rate:.1f} Hz, "
            f"score_mode={self.matcher.score_mode}."
        )

    def report_status(self, status: str) -> None:
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def _write_diagnostic(self, record: dict[str, object]) -> None:
        """Append one JSONL audit record when offline replay is enabled."""
        if self.diagnostic_handle is None:
            return
        json.dump(record, self.diagnostic_handle, separators=(",", ":"))
        self.diagnostic_handle.write("\n")
        self.diagnostic_handle.flush()

    def pose_callback(self, message: Odometry) -> None:
        self.latest_pose = message

    def scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message

    def map_callback(self, message: OccupancyGrid) -> None:
        with self.matcher_lock:
            self.matcher.update_map(
                width=message.info.width,
                height=message.info.height,
                resolution=message.info.resolution,
                origin_x=message.info.origin.position.x,
                origin_y=message.info.origin.position.y,
                data=message.data,
            )
            if not self.diagnostic_map_written:
                self._write_diagnostic(
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
                self.diagnostic_map_written = True

    def publish_map_odom_transform(self, stamp: object) -> None:
        if self.tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame_id
        transform.child_frame_id = self.odom_frame_id
        transform.transform.translation.x = self.map_odom_transform[0]
        transform.transform.translation.y = self.map_odom_transform[1]
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = quaternion_from_yaw(self.map_odom_transform[2])
        self.tf_broadcaster.sendTransform(transform)

    def _make_match_job(
        self,
    ) -> Optional[tuple[LidarMapMatcher, dict[str, object]]]:
        """Copy the latest messages and matcher into an isolated job."""
        if self.latest_pose is None or self.latest_scan is None:
            return None
        with self.matcher_lock:
            if not self.matcher.ready:
                return None
            matcher_snapshot = copy.deepcopy(self.matcher)
            pose = self.latest_pose.pose.pose
            odom_pose: Pose2D = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            )
            map_pose = transform_pose(self.map_odom_transform, odom_pose)
            pose_covariance = self.latest_pose.pose.covariance
            position_covariance_xy = (
                float(pose_covariance[0]),
                0.5 * float(pose_covariance[1] + pose_covariance[6]),
                float(pose_covariance[7]),
            )
            map_odom_yaw = float(self.map_odom_transform[2])
        scan = self.latest_scan
        submitted_monotonic_s = time.monotonic()

        def stamp_to_seconds(message: object) -> float | None:
            stamp = getattr(message, "stamp", None)
            if stamp is None:
                return None
            return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)

        snapshot = {
            "diagnostic_match_id": self.diagnostic_match_id,
            "odom_pose": odom_pose,
            "map_pose": map_pose,
            "position_covariance_xy": position_covariance_xy,
            "map_odom_yaw": map_odom_yaw,
            "submitted_monotonic_s": submitted_monotonic_s,
            "scan_stamp_s": stamp_to_seconds(scan.header),
            "pose_stamp_s": stamp_to_seconds(self.latest_pose.header),
            "ranges": tuple(scan.ranges),
            "angle_min": scan.angle_min,
            "angle_increment": scan.angle_increment,
            "range_min": scan.range_min,
            "range_max": scan.range_max,
        }
        if self.diagnostic_handle is not None:
            self._write_diagnostic(
                {
                    "record_type": "match_input",
                    "match_id": self.diagnostic_match_id,
                    "submitted_monotonic_s": submitted_monotonic_s,
                    "scan_stamp_s": snapshot["scan_stamp_s"],
                    "pose_stamp_s": snapshot["pose_stamp_s"],
                    "odom_pose": list(odom_pose),
                    "map_pose": list(map_pose),
                    "angle_min": float(scan.angle_min),
                    "angle_increment": float(scan.angle_increment),
                    "range_min": float(scan.range_min),
                    "range_max": float(scan.range_max),
                    "ranges": [
                        float(value) if math.isfinite(float(value)) else None
                        for value in scan.ranges
                    ],
                }
            )
        self.diagnostic_match_id += 1
        return matcher_snapshot, snapshot

    def _apply_match_result(self, result: dict[str, object]) -> None:
        """Apply gates and, if safe, update the persistent map-to-odom pose.

        Matching runs in a worker thread, but transform updates and ROS
        publication stay in the executor thread.  This keeps the persistent
        correction single-writer and makes the safety logic easy to audit.
        """
        odom_pose = result["odom_pose"]  # type: ignore[assignment]
        diagnostic_match_id = int(result.get("diagnostic_match_id", -1))
        map_x, map_y, map_yaw = result["map_pose"]  # type: ignore[misc]
        x = float(result["x"])
        y = float(result["y"])
        matched_yaw = float(result["matched_yaw"])
        score = float(result["score"])
        prior_score = float(result["prior_score"])
        point_count = int(result["point_count"])
        score_improvement = (
            prior_score - score
            if math.isfinite(prior_score) and math.isfinite(score)
            else float("-inf")
        )
        applied_monotonic_s = time.monotonic()
        submitted_monotonic_s = result.get("submitted_monotonic_s")
        match_age_s = (
            applied_monotonic_s - float(submitted_monotonic_s)
            if submitted_monotonic_s is not None
            else None
        )
        # The transform update is computed from the pose paired with the
        # scan.  Record how far live odometry moved before the worker result
        # returned.  This is diagnostic only: the existing acceptance policy
        # is intentionally unchanged by this measurement.
        current_odom_pose: Pose2D | None = None
        odom_motion_during_match_m: float | None = None
        odom_yaw_change_during_match_rad: float | None = None
        if self.latest_pose is not None:
            current_pose = self.latest_pose.pose.pose
            current_odom_pose = (
                current_pose.position.x,
                current_pose.position.y,
                yaw_from_quaternion(
                    current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w,
                ),
            )
            odom_motion_during_match_m = math.hypot(
                current_odom_pose[0] - odom_pose[0],
                current_odom_pose[1] - odom_pose[1],
            )
            odom_yaw_change_during_match_rad = wrap_angle(
                current_odom_pose[2] - odom_pose[2]
            )
        match_age_valid = (
            match_age_s is None or match_age_s <= self.max_match_age_s
        )
        odom_motion_valid = (
            odom_motion_during_match_m is None
            or odom_motion_during_match_m
            <= self.max_odom_motion_during_match_m
        )
        odom_yaw_change_valid = (
            odom_yaw_change_during_match_rad is None
            or abs(odom_yaw_change_during_match_rad)
            <= self.max_odom_yaw_change_during_match_rad
        )
        stale_match = not (
            match_age_valid and odom_motion_valid and odom_yaw_change_valid
        )
        candidate_dx = x - map_x
        candidate_dy = y - map_y
        candidate_dyaw = wrap_angle(matched_yaw - map_yaw)
        candidate_correction = math.hypot(candidate_dx, candidate_dy)
        raw_position_covariance = result.get("position_covariance_xy")
        map_odom_yaw = float(result.get("map_odom_yaw", 0.0))
        position_covariance_xy = (
            tuple(float(value) for value in raw_position_covariance)
            if isinstance(raw_position_covariance, (list, tuple))
            and len(raw_position_covariance) == 3
            else None
        )
        map_position_covariance_xy = (
            rotate_planar_covariance(position_covariance_xy, map_odom_yaw)
            if position_covariance_xy is not None
            else None
        )
        candidate_mahalanobis_sq = mahalanobis_squared_2d(
            (candidate_dx, candidate_dy),
            map_position_covariance_xy,
        )
        # LiDAR matching may search yaw to improve the scan score, but this
        # node intentionally corrects position only.  Preserve the current
        # fused wheel/IMU heading and publish the matched yaw only as a
        # diagnostic innovation.
        candidate_map_pose: Pose2D = (x, y, map_yaw)
        desired_map_odom = map_odom_from_poses(
            candidate_map_pose,
            odom_pose,
        )
        proposed_map_odom = interpolate_pose(
            self.map_odom_transform,
            desired_map_odom,
            self.correction_smoothing,
        )
        total_correction = math.hypot(
            proposed_map_odom[0] - self.initial_map_odom_transform[0],
            proposed_map_odom[1] - self.initial_map_odom_transform[1],
        )
        candidate_is_free = self.matcher.is_free(
            x,
            y,
            clearance_radius_m=self.robot_radius_m,
        )
        candidate_is_new = (
            self.last_applied_candidate_correction is None
            or math.hypot(
                candidate_dx - self.last_applied_candidate_correction[0],
                candidate_dy - self.last_applied_candidate_correction[1],
            ) >= self.minimum_reapplication_change_m
        )
        # These are safety gates around the optimizer, not extra tuning terms.
        # A low residual alone is insufficient: the candidate must have enough
        # returns, stay close to the prior, have a bounded yaw change, and
        # improve the prior-pose score.
        quality_valid = (
            math.isfinite(score)
            and point_count >= self.matcher.minimum_points
            and score <= self.max_match_score_m
            and candidate_correction <= self.max_correction_m
            and total_correction <= self.max_total_correction_m
            and abs(candidate_dyaw) <= self.max_heading_correction_rad
            and candidate_is_free
            and candidate_is_new
            and score_improvement >= self.minimum_score_improvement_m
            and not stale_match
        )
        # The localizer applies position corrections only.  The matched yaw
        # is retained as a diagnostic innovation, so yaw must not invalidate
        # an otherwise coherent x/y candidate or make the same position look
        # new on every scan.
        consistent = (
            self.previous_candidate_correction is None
            or math.hypot(
                candidate_dx - self.previous_candidate_correction[0],
                candidate_dy - self.previous_candidate_correction[1],
            ) <= self.candidate_consistency_m
        )
        if quality_valid and consistent:
            self.valid_match_streak += 1
        elif quality_valid:
            self.valid_match_streak = 1
        else:
            self.valid_match_streak = 0
        self.previous_candidate_correction = (
            (candidate_dx, candidate_dy, candidate_dyaw)
            if quality_valid
            else None
        )
        match_valid = self.valid_match_streak >= self.minimum_consecutive_matches
        gate_checks = {
            "finite_score": math.isfinite(score),
            "minimum_points": point_count >= self.matcher.minimum_points,
            "score_bound": score <= self.max_match_score_m,
            "candidate_correction_bound": candidate_correction <= self.max_correction_m,
            "total_correction_bound": total_correction <= self.max_total_correction_m,
            "heading_correction_bound": abs(candidate_dyaw) <= self.max_heading_correction_rad,
            "candidate_is_free": candidate_is_free,
            "candidate_is_new": candidate_is_new,
            "score_improvement": score_improvement >= self.minimum_score_improvement_m,
            "match_age_bound": match_age_valid,
            "odom_motion_bound": odom_motion_valid,
            "odom_yaw_change_bound": odom_yaw_change_valid,
            "candidate_consistent": consistent,
            "minimum_consecutive_matches": match_valid,
        }
        rejection_reasons = [
            name for name, passed in gate_checks.items() if not passed
        ]
        if not math.isfinite(score):
            match_status = "invalid_score"
        elif point_count < self.matcher.minimum_points:
            match_status = "insufficient_points"
        elif score > self.max_match_score_m:
            match_status = "score_too_large"
        elif candidate_correction > self.max_correction_m:
            match_status = "correction_too_large"
        elif total_correction > self.max_total_correction_m:
            match_status = "total_correction_too_large"
        elif abs(candidate_dyaw) > self.max_heading_correction_rad:
            match_status = "heading_correction_too_large"
        elif stale_match:
            match_status = "stale_match"
        elif not candidate_is_free:
            match_status = "candidate_in_unsafe_cell"
        elif not candidate_is_new:
            match_status = "repeated_correction"
        elif score_improvement < self.minimum_score_improvement_m:
            match_status = "insufficient_score_improvement"
        elif not consistent:
            match_status = "inconsistent_candidate"
        elif not match_valid:
            match_status = "waiting_for_consecutive_matches"
        else:
            match_status = "accepted"
        if match_valid:
            # Convert the accepted map-frame candidate back into a map->odom
            # transform.  Interpolation avoids a discontinuous jump in the pose
            # that a future controller could consume.
            previous_map_odom = self.map_odom_transform
            self.map_odom_transform = proposed_map_odom
            previous_map_pose = transform_pose(previous_map_odom, odom_pose)
            updated_map_pose = transform_pose(
                self.map_odom_transform,
                odom_pose,
            )
            correction_x = updated_map_pose[0] - previous_map_pose[0]
            correction_y = updated_map_pose[1] - previous_map_pose[1]
            correction_yaw = wrap_angle(
                updated_map_pose[2] - previous_map_pose[2]
            )
            self.last_applied_candidate_correction = (
                candidate_dx,
                candidate_dy,
                candidate_dyaw,
            )
        else:
            correction_x = 0.0
            correction_y = 0.0
            correction_yaw = 0.0

        self._write_diagnostic(
            {
                "record_type": "match_result",
                "match_id": diagnostic_match_id,
                "applied_monotonic_s": applied_monotonic_s,
                "match_age_s": match_age_s,
                "worker_compute_time_s": result.get("worker_compute_time_s"),
                "scan_stamp_s": result.get("scan_stamp_s"),
                "pose_stamp_s": result.get("pose_stamp_s"),
                "odom_pose": list(odom_pose),
                "current_odom_pose": (
                    list(current_odom_pose)
                    if current_odom_pose is not None
                    else None
                ),
                "odom_motion_during_match_m": odom_motion_during_match_m,
                "odom_yaw_change_during_match_rad": (
                    odom_yaw_change_during_match_rad
                ),
                "map_pose": [map_x, map_y, map_yaw],
                "matched_pose": [x, y, matched_yaw],
                "candidate_dx_m": candidate_dx,
                "candidate_dy_m": candidate_dy,
                "position_covariance_xy_m2": (
                    list(position_covariance_xy)
                    if position_covariance_xy is not None
                    else None
                ),
                "map_position_covariance_xy_m2": (
                    list(map_position_covariance_xy)
                    if map_position_covariance_xy is not None
                    else None
                ),
                "candidate_position_mahalanobis_sq": candidate_mahalanobis_sq,
                "candidate_dyaw_rad": candidate_dyaw,
                "candidate_correction_m": candidate_correction,
                "score_m": score,
                "prior_score_m": prior_score,
                "score_improvement_m": score_improvement,
                "search_diagnostics": result.get("search_diagnostics", {}),
                "point_count": point_count,
                "candidate_is_free": candidate_is_free,
                "candidate_is_new": candidate_is_new,
                "quality_valid": quality_valid,
                "consistent": consistent,
                "valid_match_streak": self.valid_match_streak,
                "match_valid": match_valid,
                "status": match_status,
                "gate_checks": gate_checks,
                "rejection_reasons": rejection_reasons,
                "correction_x_m": correction_x,
                "correction_y_m": correction_y,
                "correction_yaw_rad": correction_yaw,
                "total_correction_m": total_correction,
                "map_odom_after": list(self.map_odom_transform),
            }
        )

        # Publish the pose produced by the previous persistent transform even
        # when this scan is rejected.  A rejected measurement must not snap the
        # output back to raw odometry or inject a one-frame correction.
        estimate_pose = transform_pose(self.map_odom_transform, odom_pose)
        estimate = copy.deepcopy(self.latest_pose)
        estimate.header.frame_id = self.output_frame_id or self.map_frame_id
        estimate.pose.pose.position.x = estimate_pose[0]
        estimate.pose.pose.position.y = estimate_pose[1]
        estimate_yaw = estimate_pose[2]
        (
            estimate.pose.pose.orientation.x,
            estimate.pose.pose.orientation.y,
            estimate.pose.pose.orientation.z,
            estimate.pose.pose.orientation.w,
        ) = quaternion_from_yaw(estimate_yaw)
        self.publish_map_odom_transform(estimate.header.stamp)
        self.publisher.publish(estimate)
        correction = Float64()
        correction.data = math.hypot(correction_x, correction_y)
        self.correction_publisher.publish(correction)
        valid = Bool()
        valid.data = match_valid
        self.match_valid_publisher.publish(valid)
        status = String()
        status.data = match_status
        self.match_status_publisher.publish(status)
        score_message = Float64()
        score_message.data = score if math.isfinite(score) else float("inf")
        self.match_score_publisher.publish(score_message)
        candidate_message = Float64()
        candidate_message.data = candidate_correction
        self.candidate_correction_publisher.publish(candidate_message)
        # Candidate components are computed before the quality gates.  Applied
        # components are zero when the candidate is rejected; otherwise they
        # are the actual change in the persistent map->odom transform after
        # smoothing.  Keeping these streams separate makes rejected matches
        # visible instead of hiding them behind the published pose.
        candidate_dx_message = Float64()
        candidate_dx_message.data = candidate_dx
        self.candidate_dx_publisher.publish(candidate_dx_message)
        candidate_dy_message = Float64()
        candidate_dy_message.data = candidate_dy
        self.candidate_dy_publisher.publish(candidate_dy_message)
        applied_dx_message = Float64()
        applied_dx_message.data = correction_x
        self.applied_dx_publisher.publish(applied_dx_message)
        applied_dy_message = Float64()
        applied_dy_message.data = correction_y
        self.applied_dy_publisher.publish(applied_dy_message)
        heading_message = Float64()
        heading_message.data = correction_yaw
        self.heading_correction_publisher.publish(heading_message)
        improvement_message = Float64()
        improvement_message.data = score_improvement
        self.score_improvement_publisher.publish(improvement_message)
        self.report_status(
            f"Publishing localized pose; scan_points={point_count}, "
            f"quality={quality_valid}, accepted={match_valid}, "
            f"status={match_status}, streak={self.valid_match_streak}, "
            f"correction={correction.data:.3f} m "
            f"(dx={correction_x:.3f}, dy={correction_y:.3f}), candidate="
            f"{candidate_correction:.3f} m "
            f"(dx={candidate_dx:.3f}, dy={candidate_dy:.3f}, "
            f"dyaw={candidate_dyaw:.3f}), match_score={score:.3f}, "
            f"score_improvement={score_improvement:.3f}, "
            f"total_correction={total_correction:.3f} m."
        )

    def _publish_current_pose(self) -> None:
        """Relay the newest pose at control rate using the last safe transform."""
        if self.latest_pose is None:
            return
        pose = self.latest_pose.pose.pose
        odom_pose: Pose2D = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        estimate_pose = transform_pose(self.map_odom_transform, odom_pose)
        estimate = copy.deepcopy(self.latest_pose)
        estimate.header.frame_id = self.output_frame_id or self.map_frame_id
        estimate.pose.pose.position.x = estimate_pose[0]
        estimate.pose.pose.position.y = estimate_pose[1]
        (
            estimate.pose.pose.orientation.x,
            estimate.pose.pose.orientation.y,
            estimate.pose.pose.orientation.z,
            estimate.pose.pose.orientation.w,
        ) = quaternion_from_yaw(estimate_pose[2])
        self.publish_map_odom_transform(estimate.header.stamp)
        self.publisher.publish(estimate)

    def publish_estimate(self) -> None:
        """Keep pose output fast while scheduling matching at a slower rate."""
        if self.latest_pose is None:
            self.report_status("Waiting for the input pose.")
            return

        now = time.monotonic()
        if self.match_future is not None and self.match_future.done():
            completed = self.match_future
            self.match_future = None
            try:
                self._apply_match_result(completed.result())
            except Exception as exc:
                self.report_status(f"LiDAR matching failed: {exc}")

        if self.latest_scan is None:
            self.report_status("Waiting for a LiDAR scan.")
        elif not self.matcher.ready:
            self.report_status("Waiting for the static map.")
        elif (
            self.match_future is None
            and now - self.last_match_submit_time >= self.match_interval_s
        ):
            job = self._make_match_job()
            if job is not None:
                self.match_future = self.match_executor.submit(
                    run_match_job,
                    job,
                )
                self.last_match_submit_time = now
                self.report_status("LiDAR match running in the background.")

        # This call is intentionally independent of the matcher.  Even if a
        # search takes several seconds, the follower continues receiving fresh
        # wheel/IMU state rather than repeatedly steering on an old message.
        self._publish_current_pose()

    def destroy_node(self):
        """Stop the worker before releasing ROS resources during shutdown."""
        if self.match_future is not None and not self.match_future.done():
            self.match_future.cancel()
        try:
            self.match_executor.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            self.match_executor.shutdown(wait=False)
        if self.diagnostic_handle is not None:
            self.diagnostic_handle.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
