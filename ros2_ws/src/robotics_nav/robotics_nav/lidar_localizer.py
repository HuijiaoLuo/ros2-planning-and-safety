#!/usr/bin/env python3
"""Publish a local LiDAR-to-map position correction for V3.6."""

from __future__ import annotations

import copy
import math
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
        self.declare_parameter("minimum_points", 6)
        self.declare_parameter("max_correction_m", 0.15)
        self.declare_parameter("max_match_score_m", 0.12)
        self.declare_parameter("correction_smoothing", 0.25)
        self.declare_parameter("yaw_search_radius_rad", 0.15)
        self.declare_parameter("yaw_search_step_rad", 0.05)
        self.declare_parameter("yaw_prior_weight", 0.02)
        self.declare_parameter("max_heading_correction_rad", 0.25)
        self.declare_parameter("minimum_score_improvement_m", 0.005)
        self.declare_parameter("minimum_consecutive_matches", 3)
        self.declare_parameter("candidate_consistency_m", 0.05)
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("broadcast_map_odom_tf", True)
        self.declare_parameter("publish_rate_hz", 5.0)

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
        heading_correction_topic = str(
            self.get_parameter("heading_correction_topic").value
        )
        score_improvement_topic = str(
            self.get_parameter("score_improvement_topic").value
        )
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.max_correction_m = max(
            0.0, float(self.get_parameter("max_correction_m").value)
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
            yaw_search_radius_rad=float(
                self.get_parameter("yaw_search_radius_rad").value
            ),
            yaw_search_step_rad=float(
                self.get_parameter("yaw_search_step_rad").value
            ),
            yaw_prior_weight=float(self.get_parameter("yaw_prior_weight").value),
            minimum_points=int(self.get_parameter("minimum_points").value),
        )

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
        self.heading_correction_publisher = self.create_publisher(
            Float64, heading_correction_topic, 10
        )
        self.score_improvement_publisher = self.create_publisher(
            Float64, score_improvement_topic, 10
        )
        self.timer = self.create_timer(
            1.0 / max(0.1, self.publish_rate), self.publish_estimate
        )

        self.latest_pose: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        self.last_status: Optional[str] = None
        self.valid_match_streak = 0
        self.previous_candidate_correction: Optional[tuple[float, float, float]] = None
        # The localizer estimates a persistent map->odom transform.  A scan
        # match updates this transform; rejected matches keep the last valid
        # transform and therefore do not snap the published pose back to raw
        # odometry.
        self.map_odom_transform: Pose2D = (0.0, 0.0, 0.0)

        self.get_logger().info(
            f"LiDAR localizer: {input_topic} + {scan_topic} + {map_topic} "
            f"-> {output_topic}; search_radius="
            f"{self.matcher.search_radius_m:.3f} m, max_correction="
            f"{self.max_correction_m:.3f} m, max_score="
            f"{self.max_match_score_m:.3f} m, smoothing="
            f"{self.correction_smoothing:.2f}, consecutive_matches="
            f"{self.minimum_consecutive_matches}, yaw_search="
            f"{self.matcher.yaw_search_radius_rad:.3f} rad."
        )

    def report_status(self, status: str) -> None:
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def pose_callback(self, message: Odometry) -> None:
        self.latest_pose = message

    def scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message

    def map_callback(self, message: OccupancyGrid) -> None:
        self.matcher.update_map(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            data=message.data,
        )

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

    def publish_estimate(self) -> None:
        if self.latest_pose is None:
            self.report_status("Waiting for the input pose.")
            return
        if self.latest_scan is None:
            self.report_status("Waiting for a LiDAR scan.")
            return
        if not self.matcher.ready:
            self.report_status("Waiting for the static map.")
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
        map_pose = transform_pose(self.map_odom_transform, odom_pose)
        map_x, map_y, map_yaw = map_pose
        x, y, matched_yaw, score, point_count = self.matcher.match_pose(
            map_x,
            map_y,
            map_yaw,
            self.latest_scan.ranges,
            angle_min=self.latest_scan.angle_min,
            angle_increment=self.latest_scan.angle_increment,
            range_min=self.latest_scan.range_min,
            range_max=self.latest_scan.range_max,
        )
        prior_score, _ = self.matcher.score_pose(
            map_x,
            map_y,
            map_yaw,
            self.latest_scan.ranges,
            angle_min=self.latest_scan.angle_min,
            angle_increment=self.latest_scan.angle_increment,
            range_min=self.latest_scan.range_min,
            range_max=self.latest_scan.range_max,
        )
        score_improvement = (
            prior_score - score
            if math.isfinite(prior_score) and math.isfinite(score)
            else float("-inf")
        )
        candidate_dx = x - map_x
        candidate_dy = y - map_y
        candidate_dyaw = wrap_angle(matched_yaw - map_yaw)
        candidate_correction = math.hypot(candidate_dx, candidate_dy)
        quality_valid = (
            math.isfinite(score)
            and point_count >= self.matcher.minimum_points
            and score <= self.max_match_score_m
            and candidate_correction <= self.max_correction_m
            and abs(candidate_dyaw) <= self.max_heading_correction_rad
            and score_improvement >= self.minimum_score_improvement_m
        )
        consistent = (
            self.previous_candidate_correction is None
            or math.hypot(
                candidate_dx - self.previous_candidate_correction[0],
                candidate_dy - self.previous_candidate_correction[1],
            )
            <= self.candidate_consistency_m
            and abs(
                wrap_angle(
                    candidate_dyaw - self.previous_candidate_correction[2]
                )
            )
            <= self.matcher.yaw_search_step_rad
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
        if not math.isfinite(score):
            match_status = "invalid_score"
        elif point_count < self.matcher.minimum_points:
            match_status = "insufficient_points"
        elif score > self.max_match_score_m:
            match_status = "score_too_large"
        elif candidate_correction > self.max_correction_m:
            match_status = "correction_too_large"
        elif abs(candidate_dyaw) > self.max_heading_correction_rad:
            match_status = "heading_correction_too_large"
        elif score_improvement < self.minimum_score_improvement_m:
            match_status = "insufficient_score_improvement"
        elif not consistent:
            match_status = "inconsistent_candidate"
        elif not match_valid:
            match_status = "waiting_for_consecutive_matches"
        else:
            match_status = "accepted"
        if match_valid:
            candidate_map_pose: Pose2D = (x, y, matched_yaw)
            desired_map_odom = map_odom_from_poses(
                candidate_map_pose,
                odom_pose,
            )
            previous_map_odom = self.map_odom_transform
            self.map_odom_transform = interpolate_pose(
                previous_map_odom,
                desired_map_odom,
                self.correction_smoothing,
            )
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
        else:
            correction_x = 0.0
            correction_y = 0.0
            correction_yaw = 0.0

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
            f"correction={correction.data:.3f} m, candidate="
            f"{candidate_correction:.3f} m, candidate_dyaw="
            f"{candidate_dyaw:.3f} rad, match_score={score:.3f}, "
            f"score_improvement={score_improvement:.3f}."
        )


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
