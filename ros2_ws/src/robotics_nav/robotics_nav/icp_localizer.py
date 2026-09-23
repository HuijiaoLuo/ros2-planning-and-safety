#!/usr/bin/env python3
"""ROS 2 adapter for the dependency-free point-to-point ICP baseline.

The node deliberately keeps registration separate from the high-rate pose
relay.  Wheel/IMU state is published continuously; a LiDAR scan occasionally
starts one ICP job against the known static map.  A converged result updates a
map-to-odom transform and is published once as a timestamped localization
candidate.  Rejected or unfinished registration never blocks the controller
and never gets silently converted into a correction.
"""

from __future__ import annotations

import concurrent.futures
import copy
import json
import math
import multiprocessing
import time
from pathlib import Path
from typing import Optional

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

from robotics_nav.icp_localization import (
    ICPConfig,
    ICPResult,
    point_to_point_icp,
    scan_points_from_ranges,
)
from robotics_nav.mcl_localization import OccupancyGridMap, Pose2D, wrap_angle


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


def pose_from_message(message: Odometry) -> Pose2D:
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


def transform_pose(transform: Pose2D, pose: Pose2D) -> Pose2D:
    """Compose a planar map-to-odom transform with an odom pose."""

    tx, ty, tyaw = transform
    x, y, yaw = pose
    cosine = math.cos(tyaw)
    sine = math.sin(tyaw)
    return (
        tx + cosine * x - sine * y,
        ty + sine * x + cosine * y,
        wrap_angle(tyaw + yaw),
    )


def inverse_pose(pose: Pose2D) -> Pose2D:
    """Return the inverse of a planar rigid transform."""

    x, y, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        -cosine * x - sine * y,
        sine * x - cosine * y,
        wrap_angle(-yaw),
    )


def map_odom_from_candidate(candidate: Pose2D, source: Pose2D) -> Pose2D:
    """Compute the map-to-odom transform implied by one ICP candidate."""

    return transform_pose(candidate, inverse_pose(source))


def planar_distance(first: Pose2D, second: Pose2D) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def run_icp_job(
    job: tuple[
        list[tuple[float, float]],
        OccupancyGridMap,
        Pose2D,
        ICPConfig,
        float,
        float,
    ]
) -> dict[str, object]:
    """Run one immutable ICP snapshot outside the ROS callback thread."""

    scan_points, occupancy_map, initial_pose, config, scan_stamp_s, pose_stamp_s = job
    started = time.monotonic()
    result = point_to_point_icp(
        scan_points,
        occupancy_map,
        initial_pose,
        config=config,
    )
    return {
        "result": result,
        "worker_compute_time_s": time.monotonic() - started,
        "scan_stamp_s": scan_stamp_s,
        "pose_stamp_s": pose_stamp_s,
    }


class ICPLocalizer(Node):
    """Publish known-map point-to-point ICP corrections under the V4 topic API."""

    def __init__(self) -> None:
        super().__init__("icp_localizer")
        self.declare_parameter("input_pose_topic", "/state_estimate")
        self.declare_parameter("output_pose_topic", "/localized_estimate")
        self.declare_parameter("external_position_topic", "/localization_candidate")
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
        self.declare_parameter("match_rate_hz", 2.0)
        self.declare_parameter("scan_stride", 6)
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("minimum_correspondences", 6)
        self.declare_parameter("maximum_correspondence_distance_m", 0.35)
        self.declare_parameter("max_iterations", 20)

        input_topic = str(self.get_parameter("input_pose_topic").value)
        output_topic = str(self.get_parameter("output_pose_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        self.external_position_topic = str(
            self.get_parameter("external_position_topic").value
        )
        self.diagnostic_output = str(self.get_parameter("diagnostic_output").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.match_rate_hz = max(0.1, float(self.get_parameter("match_rate_hz").value))
        self.scan_stride = max(1, int(self.get_parameter("scan_stride").value))
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.config = ICPConfig(
            max_iterations=max(1, int(self.get_parameter("max_iterations").value)),
            maximum_correspondence_distance_m=max(
                1.0e-6,
                float(self.get_parameter("maximum_correspondence_distance_m").value),
            ),
            minimum_correspondences=max(
                3, int(self.get_parameter("minimum_correspondences").value)
            ),
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_subscription = self.create_subscription(
            Odometry, input_topic, self.pose_callback, qos_profile_sensor_data
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, map_qos
        )
        self.pose_publisher = self.create_publisher(Odometry, output_topic, 10)
        self.candidate_publisher = self.create_publisher(
            Odometry, self.external_position_topic, 10
        )
        self.valid_publisher = self.create_publisher(
            Bool, str(self.get_parameter("match_valid_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("match_status_topic").value), 10
        )
        self.score_publisher = self.create_publisher(
            Float64, str(self.get_parameter("match_score_topic").value), 10
        )
        self.candidate_correction_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("candidate_correction_topic").value),
            10,
        )
        self.candidate_dx_publisher = self.create_publisher(
            Float64, str(self.get_parameter("candidate_dx_topic").value), 10
        )
        self.candidate_dy_publisher = self.create_publisher(
            Float64, str(self.get_parameter("candidate_dy_topic").value), 10
        )
        self.applied_dx_publisher = self.create_publisher(
            Float64, str(self.get_parameter("applied_dx_topic").value), 10
        )
        self.applied_dy_publisher = self.create_publisher(
            Float64, str(self.get_parameter("applied_dy_topic").value), 10
        )
        self.score_improvement_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("score_improvement_topic").value),
            10,
        )

        self.latest_pose: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_map: Optional[OccupancyGridMap] = None
        self.map_odom_transform: Pose2D = (0.0, 0.0, 0.0)
        self.latest_status = "waiting_for_input"
        self.latest_score = float("nan")
        self.latest_result: Optional[dict[str, object]] = None
        self.last_match_submit_time = 0.0
        self.match_id = 0
        self.match_future: Optional[concurrent.futures.Future] = None
        # Keep the worker-pool handle separate from rclpy.node.Node.executor.
        # ``Node`` exposes ``executor`` as a managed property; assigning a
        # ProcessPoolExecutor to that name breaks node construction before the
        # ICP worker can process its first scan.
        self.worker_executor = concurrent.futures.ProcessPoolExecutor(
            max_workers=1,
            mp_context=multiprocessing.get_context("spawn"),
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self.timer_callback
        )
        self.diagnostic_handle = None
        if self.diagnostic_output:
            path = Path(self.diagnostic_output)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.diagnostic_handle = path.open("w", encoding="utf-8")
            self.write_diagnostic(
                {
                    "record_type": "config",
                    "backend": "icp_point_to_point",
                    "config": {
                        "max_iterations": self.config.max_iterations,
                        "maximum_correspondence_distance_m": self.config.maximum_correspondence_distance_m,
                        "minimum_correspondences": self.config.minimum_correspondences,
                        "translation_tolerance_m": self.config.translation_tolerance_m,
                        "rotation_tolerance_rad": self.config.rotation_tolerance_rad,
                        "scan_stride": self.scan_stride,
                        "publish_rate_hz": self.publish_rate_hz,
                        "match_rate_hz": self.match_rate_hz,
                    },
                    "assumption": (
                        "the occupancy map contains point targets at occupied-cell "
                        "centres and the state estimate is a local initial pose"
                    ),
                }
            )
        self.get_logger().info(
            f"ICP localizer: {input_topic} + {scan_topic} + {map_topic} -> "
            f"{output_topic}; point_to_point, stride={self.scan_stride}, "
            f"match_rate={self.match_rate_hz:.1f} Hz, "
            f"max_correspondence={self.config.maximum_correspondence_distance_m:.2f} m."
        )

    def write_diagnostic(self, record: dict[str, object]) -> None:
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
        width = int(message.info.width)
        height = int(message.info.height)
        self.latest_map = OccupancyGridMap(
            width=width,
            height=height,
            resolution_m=float(message.info.resolution),
            origin_x_m=float(message.info.origin.position.x),
            origin_y_m=float(message.info.origin.position.y),
            occupied_cells=frozenset(
                (column, row)
                for row in range(height)
                for column in range(width)
                if int(message.data[row * width + column]) >= 50
            ),
        )

    def _make_job(
        self,
    ) -> Optional[
        tuple[
            list[tuple[float, float]],
            OccupancyGridMap,
            Pose2D,
            ICPConfig,
            float,
            float,
        ]
    ]:
        if self.latest_pose is None or self.latest_scan is None or self.latest_map is None:
            return None
        pose = pose_from_message(self.latest_pose)
        scan = self.latest_scan
        points = scan_points_from_ranges(
            scan.ranges,
            angle_min_rad=float(scan.angle_min),
            angle_increment_rad=float(scan.angle_increment),
            range_min_m=float(scan.range_min),
            range_max_m=float(scan.range_max),
            stride=self.scan_stride,
        )
        return (
            points,
            self.latest_map,
            pose,
            self.config,
            stamp_seconds(scan),
            stamp_seconds(self.latest_pose),
        )

    def _make_pose_message(
        self,
        source: Odometry,
        pose: Pose2D,
        *,
        stamp: tuple[int, int] | None = None,
    ) -> Odometry:
        message = copy.deepcopy(source)
        message.header.frame_id = self.output_frame_id or source.header.frame_id
        if stamp is not None:
            message.header.stamp.sec = int(stamp[0])
            message.header.stamp.nanosec = int(stamp[1])
        message.pose.pose.position.x = pose[0]
        message.pose.pose.position.y = pose[1]
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = quaternion_from_yaw(pose[2])
        return message

    def publish_diagnostics(
        self,
        *,
        valid: bool,
        status: str,
        score: float,
        candidate_dx: float,
        candidate_dy: float,
        applied_dx: float,
        applied_dy: float,
        candidate_message: Optional[Odometry],
    ) -> None:
        valid_message = Bool()
        valid_message.data = bool(valid)
        self.valid_publisher.publish(valid_message)
        status_message = String()
        status_message.data = status
        self.status_publisher.publish(status_message)
        score_message = Float64()
        score_message.data = score if math.isfinite(score) else float("inf")
        self.score_publisher.publish(score_message)
        for publisher, value in (
            (self.candidate_correction_publisher, math.hypot(candidate_dx, candidate_dy)),
            (self.candidate_dx_publisher, candidate_dx),
            (self.candidate_dy_publisher, candidate_dy),
            (self.applied_dx_publisher, applied_dx),
            (self.applied_dy_publisher, applied_dy),
        ):
            message = Float64()
            message.data = float(value)
            publisher.publish(message)
        improvement_message = Float64()
        improvement_message.data = 0.0
        self.score_improvement_publisher.publish(improvement_message)
        if valid and candidate_message is not None:
            self.candidate_publisher.publish(candidate_message)

    def _apply_result(self, payload: dict[str, object]) -> None:
        result = payload["result"]
        if not isinstance(result, ICPResult) or self.latest_pose is None:
            return
        initial_pose = result.initial_pose
        candidate_pose = result.pose
        candidate_dx = result.correction_x_m
        candidate_dy = result.correction_y_m
        accepted = bool(
            result.converged
            and result.correspondence_count >= self.config.minimum_correspondences
            and math.isfinite(result.mean_residual_m)
        )
        status = "accepted" if accepted else result.status
        applied_dx = 0.0
        applied_dy = 0.0
        if accepted:
            previous_transform = self.map_odom_transform
            self.map_odom_transform = map_odom_from_candidate(
                candidate_pose, initial_pose
            )
            current_pose = pose_from_message(self.latest_pose)
            previous_map_pose = transform_pose(previous_transform, current_pose)
            updated_map_pose = transform_pose(self.map_odom_transform, current_pose)
            applied_dx = updated_map_pose[0] - previous_map_pose[0]
            applied_dy = updated_map_pose[1] - previous_map_pose[1]
        self.latest_status = status
        self.latest_score = result.mean_residual_m
        self.latest_result = payload
        scan_stamp_s = payload.get("scan_stamp_s")
        pose_stamp_s = payload.get("pose_stamp_s")
        self.write_diagnostic(
            {
                "record_type": "icp_result",
                "match_id": self.match_id,
                "scan_stamp_s": scan_stamp_s,
                "pose_stamp_s": pose_stamp_s,
                "match_age_s": (
                    None
                    if scan_stamp_s is None
                    else pose_stamp_s - scan_stamp_s
                ),
                "worker_compute_time_s": payload.get("worker_compute_time_s"),
                "initial_pose": list(initial_pose),
                "candidate_pose": list(candidate_pose),
                "candidate_dx_m": candidate_dx,
                "candidate_dy_m": candidate_dy,
                "candidate_correction_m": result.correction_m,
                "candidate_dyaw_rad": result.correction_yaw_rad,
                "mean_residual_m": result.mean_residual_m,
                "rms_residual_m": result.rms_residual_m,
                "correspondence_count": result.correspondence_count,
                "iterations": result.iterations,
                "converged": result.converged,
                "status": status,
                "measurement_applied": accepted,
                "applied_dx_m": applied_dx,
                "applied_dy_m": applied_dy,
                "map_odom_after": list(self.map_odom_transform),
            }
        )
        candidate_message = None
        if accepted:
            candidate_message = self._make_pose_message(
                self.latest_pose,
                candidate_pose,
                stamp=(
                    int(pose_stamp_s),
                    int(round((float(pose_stamp_s) - int(pose_stamp_s)) * 1.0e9)),
                )
                if isinstance(pose_stamp_s, (int, float))
                else None,
            )
        self.publish_diagnostics(
            valid=accepted,
            status=status,
            score=result.mean_residual_m,
            candidate_dx=candidate_dx,
            candidate_dy=candidate_dy,
            applied_dx=applied_dx,
            applied_dy=applied_dy,
            candidate_message=candidate_message,
        )
        self.get_logger().info(
            f"ICP result: status={status}, points={result.correspondence_count}, "
            f"iterations={result.iterations}, residual={result.mean_residual_m:.3f} m, "
            f"candidate={result.correction_m:.3f} m, applied="
            f"{math.hypot(applied_dx, applied_dy):.3f} m."
        )

    def _publish_current_pose(self) -> None:
        if self.latest_pose is None:
            return
        pose = transform_pose(self.map_odom_transform, pose_from_message(self.latest_pose))
        self.pose_publisher.publish(self._make_pose_message(self.latest_pose, pose))

    def timer_callback(self) -> None:
        now = time.monotonic()
        if self.match_future is not None and self.match_future.done():
            completed = self.match_future
            self.match_future = None
            try:
                self._apply_result(completed.result())
            except Exception as exc:  # pragma: no cover - ROS runtime guard
                self.latest_status = "icp_failed"
                self.get_logger().error(f"ICP registration failed: {exc}")
        if self.latest_pose is None:
            self.latest_status = "waiting_for_input"
        elif self.latest_scan is None:
            self.latest_status = "waiting_for_scan"
        elif self.latest_map is None:
            self.latest_status = "waiting_for_map"
        elif (
            self.match_future is None
            and now - self.last_match_submit_time >= 1.0 / self.match_rate_hz
        ):
            job = self._make_job()
            if job is not None:
                self.match_id += 1
                self.match_future = self.worker_executor.submit(run_icp_job, job)
                self.last_match_submit_time = now
        self._publish_current_pose()

    def destroy_node(self):
        if self.match_future is not None and not self.match_future.done():
            self.match_future.cancel()
        try:
            self.worker_executor.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            self.worker_executor.shutdown(wait=False)
        if self.diagnostic_handle is not None:
            self.diagnostic_handle.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ICPLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ros2 launch may already have shut down the shared context during
        # SIGINT.  Avoid turning a clean experiment stop into exit code 1 by
        # shutting it down only while it is still active.
        if rclpy.ok():
            rclpy.shutdown()
