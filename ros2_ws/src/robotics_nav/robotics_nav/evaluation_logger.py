#!/usr/bin/env python3
"""Collect closed-loop navigation metrics without affecting robot control."""

from __future__ import annotations

import csv
import math
from pathlib import Path
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Float64, String


def distance_2d(first_x: float, first_y: float, second_x: float, second_y: float) -> float:
    return math.hypot(second_x - first_x, second_y - first_y)


class EvaluationLogger(Node):
    """Measure the outcome of one closed-loop navigation run."""

    def __init__(self) -> None:
        super().__init__("evaluation_logger")

        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("front_angle_deg", 60.0)
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("output_path", "")
        self.declare_parameter("trace_output", "")
        self.declare_parameter("plan_output", "")
        self.declare_parameter("map_output", "")
        self.declare_parameter("collision_topic", "/collision/contacts")
        self.declare_parameter("safety_override_topic", "/safety_override")
        self.declare_parameter("minimum_clearance", 0.50)
        self.declare_parameter("sensor_latency", 0.10)
        self.declare_parameter("scan_delay_s", 0.0)
        self.declare_parameter("scan_noise_std_m", 0.0)
        self.declare_parameter("scan_noise_seed", 0)
        self.declare_parameter("safety_margin", 0.15)
        self.declare_parameter("planning_radius_m", 0.35)
        self.declare_parameter("experiment_timeout_s", 0.0)
        self.declare_parameter("navigation_pose_topic", "/odom")
        # These topics are diagnostics only.  None of them is used to affect
        # the planner, follower, or safety supervisor.
        self.declare_parameter("state_estimate_topic", "/state_estimate")
        self.declare_parameter(
            "localization_candidate_correction_topic",
            "/localization_candidate_correction_m",
        )
        self.declare_parameter(
            "localization_candidate_dx_topic", "/localization_candidate_dx_m"
        )
        self.declare_parameter(
            "localization_candidate_dy_topic", "/localization_candidate_dy_m"
        )
        self.declare_parameter(
            "localization_applied_dx_topic", "/localization_applied_dx_m"
        )
        self.declare_parameter(
            "localization_applied_dy_topic", "/localization_applied_dy_m"
        )
        self.declare_parameter(
            "localization_match_valid_topic", "/localization_match_valid"
        )
        self.declare_parameter(
            "localization_match_status_topic", "/localization_match_status"
        )
        self.declare_parameter(
            "localization_match_score_topic", "/localization_match_score_m"
        )
        self.declare_parameter(
            "localization_score_improvement_topic",
            "/localization_score_improvement_m",
        )

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.front_angle = math.radians(
            float(self.get_parameter("front_angle_deg").value)
        )
        sample_rate = float(self.get_parameter("sample_rate_hz").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self.trace_output = str(self.get_parameter("trace_output").value)
        self.plan_output = str(self.get_parameter("plan_output").value)
        self.map_output = str(self.get_parameter("map_output").value)
        self.collision_topic = str(self.get_parameter("collision_topic").value)
        self.configured_minimum_clearance = float(
            self.get_parameter("minimum_clearance").value
        )
        self.configured_sensor_latency = float(
            self.get_parameter("sensor_latency").value
        )
        self.configured_scan_delay = float(
            self.get_parameter("scan_delay_s").value
        )
        self.configured_scan_noise_std = float(
            self.get_parameter("scan_noise_std_m").value
        )
        self.configured_scan_noise_seed = int(
            float(self.get_parameter("scan_noise_seed").value)
        )
        self.configured_safety_margin = float(
            self.get_parameter("safety_margin").value
        )
        self.configured_planning_radius = float(
            self.get_parameter("planning_radius_m").value
        )
        self.configured_effective_planning_radius = max(
            self.configured_planning_radius + self.configured_safety_margin,
            self.configured_minimum_clearance,
        )
        self.configured_experiment_timeout = float(
            self.get_parameter("experiment_timeout_s").value
        )
        self.navigation_pose_topic = str(
            self.get_parameter("navigation_pose_topic").value
        )
        self.state_estimate_topic = str(
            self.get_parameter("state_estimate_topic").value
        )
        self.localization_candidate_correction_topic = str(
            self.get_parameter("localization_candidate_correction_topic").value
        )
        self.localization_candidate_dx_topic = str(
            self.get_parameter("localization_candidate_dx_topic").value
        )
        self.localization_candidate_dy_topic = str(
            self.get_parameter("localization_candidate_dy_topic").value
        )
        self.localization_applied_dx_topic = str(
            self.get_parameter("localization_applied_dx_topic").value
        )
        self.localization_applied_dy_topic = str(
            self.get_parameter("localization_applied_dy_topic").value
        )
        self.localization_match_valid_topic = str(
            self.get_parameter("localization_match_valid_topic").value
        )
        self.localization_match_status_topic = str(
            self.get_parameter("localization_match_status_topic").value
        )
        self.localization_match_score_topic = str(
            self.get_parameter("localization_match_score_topic").value
        )
        self.localization_score_improvement_topic = str(
            self.get_parameter("localization_score_improvement_topic").value
        )
        safety_override_topic = str(
            self.get_parameter("safety_override_topic").value
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_navigation_pose: Optional[Odometry] = None
        self.latest_state_estimate: Optional[Odometry] = None
        self.latest_plan: Optional[NavPath] = None
        self.initial_plan: Optional[NavPath] = None
        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_raw_command: Optional[Twist] = None
        self.latest_safe_command: Optional[Twist] = None
        self.latest_override_state: Optional[bool] = None
        self.collision_state: Optional[bool] = None
        self.latest_localization_candidate_correction_m: Optional[float] = None
        self.latest_localization_candidate_dx_m: Optional[float] = None
        self.latest_localization_candidate_dy_m: Optional[float] = None
        self.latest_localization_applied_dx_m: Optional[float] = None
        self.latest_localization_applied_dy_m: Optional[float] = None
        self.latest_localization_match_valid: Optional[bool] = None
        self.latest_localization_match_status: Optional[str] = None
        self.latest_localization_match_score_m: Optional[float] = None
        self.latest_localization_score_improvement_m: Optional[float] = None

        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.plan_update_count = 0
        self.initial_planned_path_length: Optional[float] = None
        self.latest_planned_path_length: Optional[float] = None
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.last_sample_time: Optional[float] = None
        self.started_at: Optional[float] = None
        self.goal_reached_at: Optional[float] = None
        self.navigation_goal_reached_at: Optional[float] = None
        self.state_estimate_goal_reached_at: Optional[float] = None
        self.termination_reason: Optional[str] = None

        self.travelled_distance = 0.0
        self.motion_time = 0.0
        self.minimum_clearance = float("inf")
        self.safety_override_count = 0
        self.safety_override_time = 0.0
        self.override_active = False
        self.sample_count = 0
        self.trace_rows: list[dict[str, object]] = []

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )
        if self.navigation_pose_topic != "/odom":
            self.create_subscription(
                Odometry,
                self.navigation_pose_topic,
                self.navigation_pose_callback,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Odometry,
            self.state_estimate_topic,
            self.state_estimate_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(NavPath, "/plan", self.plan_callback, path_qos)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, path_qos)
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self.raw_command_callback,
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.safe_command_callback,
            10,
        )
        self.create_subscription(
            Bool,
            safety_override_topic,
            self.override_state_callback,
            10,
        )
        self.create_subscription(
            Contacts,
            self.collision_topic,
            self.collision_callback,
            10,
        )
        self.create_subscription(
            Float64,
            self.localization_candidate_correction_topic,
            self.localization_candidate_correction_callback,
            10,
        )
        self.create_subscription(
            Float64,
            self.localization_candidate_dx_topic,
            self.localization_candidate_dx_callback,
            10,
        )
        self.create_subscription(
            Float64,
            self.localization_candidate_dy_topic,
            self.localization_candidate_dy_callback,
            10,
        )
        self.create_subscription(
            Float64,
            self.localization_applied_dx_topic,
            self.localization_applied_dx_callback,
            10,
        )
        self.create_subscription(
            Float64,
            self.localization_applied_dy_topic,
            self.localization_applied_dy_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.localization_match_valid_topic,
            self.localization_match_valid_callback,
            10,
        )
        self.create_subscription(
            String,
            self.localization_match_status_topic,
            self.localization_match_status_callback,
            10,
        )
        self.create_subscription(
            Float64,
            self.localization_match_score_topic,
            self.localization_match_score_callback,
            10,
        )
        self.create_subscription(
            Float64,
            self.localization_score_improvement_topic,
            self.localization_score_improvement_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / sample_rate, self.sample)
        self.timeout_timer = self.create_timer(0.1, self.check_experiment_timeout)

    def finish_run(self, reason: str) -> None:
        if self.termination_reason is not None:
            return
        self.termination_reason = reason
        self.get_logger().info(f"Finishing evaluation: {reason}.")
        if rclpy.ok():
            rclpy.shutdown()

    def check_experiment_timeout(self) -> None:
        """Terminate bounded experiments without sending a motion command."""
        if (
            self.configured_experiment_timeout <= 0.0
            or self.started_at is None
            or self.termination_reason is not None
        ):
            return
        if time.monotonic() - self.started_at >= self.configured_experiment_timeout:
            self.finish_run("experiment_timeout")

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def navigation_pose_callback(self, message: Odometry) -> None:
        self.latest_navigation_pose = message

    def state_estimate_callback(self, message: Odometry) -> None:
        """Record the pre-localization state estimate for diagnosis only."""
        self.latest_state_estimate = message

    def plan_callback(self, message: NavPath) -> None:
        """Track the initial/latest rasterized plan and its endpoint."""
        self.latest_plan = message
        self.plan_update_count += 1
        if not message.poses:
            return

        if self.initial_plan is None:
            self.initial_plan = message

        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        self.goal_x, self.goal_y = points[-1]
        path_length = sum(
            distance_2d(first[0], first[1], second[0], second[1])
            for first, second in zip(points, points[1:])
        )
        if self.initial_planned_path_length is None:
            self.initial_planned_path_length = path_length
        self.latest_planned_path_length = path_length

    def map_callback(self, message: OccupancyGrid) -> None:
        self.latest_map = message

    def scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message

    def raw_command_callback(self, message: Twist) -> None:
        self.latest_raw_command = message

    def safe_command_callback(self, message: Twist) -> None:
        """Record the command after the LiDAR safety layer has gated it.

        ``/cmd_vel_raw`` is the path follower's request.  ``/cmd_vel`` is the
        command that reaches the simulator after the safety supervisor may
        stop forward motion or inject a recovery turn.  Keeping both makes a
        recovery spin distinguishable from a path-follower heading command.
        """
        self.latest_safe_command = message

    def override_state_callback(self, message: Bool) -> None:
        self.latest_override_state = bool(message.data)

    def collision_callback(self, message: Contacts) -> None:
        self.collision_state = bool(message.contacts)

    def localization_candidate_correction_callback(self, message: Float64) -> None:
        self.latest_localization_candidate_correction_m = float(message.data)

    def localization_candidate_dx_callback(self, message: Float64) -> None:
        self.latest_localization_candidate_dx_m = float(message.data)

    def localization_candidate_dy_callback(self, message: Float64) -> None:
        self.latest_localization_candidate_dy_m = float(message.data)

    def localization_applied_dx_callback(self, message: Float64) -> None:
        self.latest_localization_applied_dx_m = float(message.data)

    def localization_applied_dy_callback(self, message: Float64) -> None:
        self.latest_localization_applied_dy_m = float(message.data)

    def localization_match_valid_callback(self, message: Bool) -> None:
        self.latest_localization_match_valid = bool(message.data)

    def localization_match_status_callback(self, message: String) -> None:
        self.latest_localization_match_status = str(message.data)

    def localization_match_score_callback(self, message: Float64) -> None:
        self.latest_localization_match_score_m = float(message.data)

    def localization_score_improvement_callback(self, message: Float64) -> None:
        self.latest_localization_score_improvement_m = float(message.data)

    @staticmethod
    def yaw_from_quaternion(orientation) -> float:
        sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def stamp_seconds(message: Odometry) -> float:
        """Return an Odometry header stamp in seconds for synchronization checks.

        The evaluation logger receives the truth, navigation, and estimator
        topics independently.  Recording their message timestamps makes it
        possible to distinguish a real pose error from a final-cache timing
        mismatch when a run is stopped by a timeout or SIGINT.
        """
        stamp = message.header.stamp
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)

    def front_clearance(self, scan: LaserScan) -> Optional[float]:
        """Return the closest valid LiDAR range inside the front sector.

        The value is measured from the LiDAR origin, not from the robot body.
        Infinite rays are retained as evidence that the sector was observed but
        contained no finite obstacle return.
        """
        if scan.angle_increment == 0.0:
            return None

        closest: Optional[float] = None
        saw_clear_ray = False
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > self.front_angle or math.isnan(value):
                continue
            if math.isinf(value):
                saw_clear_ray = True
                continue
            if scan.range_min <= value <= scan.range_max:
                closest = value if closest is None else min(closest, value)

        if closest is not None:
            return closest
        return scan.range_max if saw_clear_ray else None

    def sample(self) -> None:
        """Accumulate one evaluation sample without influencing control."""
        if self.latest_odom is None:
            return

        now = time.monotonic()
        delta_time = (
            0.0
            if self.last_sample_time is None
            else max(0.0, now - self.last_sample_time)
        )
        position = self.latest_odom.pose.pose.position
        current_x, current_y = position.x, position.y

        # Travelled distance is the measured odometry polyline length.  It is
        # intentionally separate from the initial grid-plan length because the
        # executed controller follows a continuous trajectory.
        if self.start_x is None:
            self.start_x, self.start_y = current_x, current_y
            self.started_at = now
        if self.last_x is not None and self.last_y is not None:
            self.travelled_distance += distance_2d(
                self.last_x,
                self.last_y,
                current_x,
                current_y,
            )
        self.last_x, self.last_y = current_x, current_y

        clearance: Optional[float] = None
        if self.latest_scan is not None:
            clearance = self.front_clearance(self.latest_scan)
            if clearance is not None:
                self.minimum_clearance = min(self.minimum_clearance, clearance)

        # The Gazebo contact sensor publishes only when a contact event is
        # present. If the bridge publisher exists but no event has arrived,
        # interpret that as a verified collision-free state. Keep `unknown`
        # only for runs where the collision topic is genuinely unavailable.
        if (
            self.collision_state is None
            and self.count_publishers(self.collision_topic) > 0
        ):
            self.collision_state = False

        if self.latest_raw_command is not None:
            raw_is_active = (
                abs(self.latest_raw_command.linear.x) > 1.0e-4
                or abs(self.latest_raw_command.angular.z) > 1.0e-4
            )
            if raw_is_active:
                self.motion_time += delta_time

        # Twist messages do not carry timestamps, so comparing the latest
        # /cmd_vel_raw and /cmd_vel messages can falsely report an override
        # when callbacks arrive at slightly different times. The supervisor
        # publishes the authoritative Bool state instead.
        if self.latest_override_state is not None:
            if self.latest_override_state and not self.override_active:
                self.safety_override_count += 1
            if self.latest_override_state:
                self.safety_override_time += delta_time
            self.override_active = self.latest_override_state

        # Always compute physical success from /odom.  A configurable
        # navigation pose is logged separately so false estimated-goal
        # completion remains visible instead of being counted as success.
        navigation_pose = (
            self.latest_odom
            if self.navigation_pose_topic == "/odom"
            else self.latest_navigation_pose
        )
        ground_truth_goal_distance: Optional[float] = None
        state_estimate_goal_distance: Optional[float] = None
        navigation_goal_distance: Optional[float] = None
        if self.goal_x is not None and self.goal_y is not None:
            ground_truth_goal_distance = distance_2d(
                current_x,
                current_y,
                self.goal_x,
                self.goal_y,
            )
            if (
                ground_truth_goal_distance <= self.goal_tolerance
                and self.goal_reached_at is None
            ):
                self.goal_reached_at = now
            if navigation_pose is not None:
                navigation_position = navigation_pose.pose.pose.position
                navigation_goal_distance = distance_2d(
                    navigation_position.x,
                    navigation_position.y,
                    self.goal_x,
                    self.goal_y,
                )
                if (
                    navigation_goal_distance <= self.goal_tolerance
                    and self.navigation_goal_reached_at is None
                ):
                    self.navigation_goal_reached_at = now
            if self.latest_state_estimate is not None:
                state_position = self.latest_state_estimate.pose.pose.position
                state_estimate_goal_distance = distance_2d(
                    state_position.x,
                    state_position.y,
                    self.goal_x,
                    self.goal_y,
                )
                if (
                    state_estimate_goal_distance <= self.goal_tolerance
                    and self.state_estimate_goal_reached_at is None
                ):
                    self.state_estimate_goal_reached_at = now

        navigation_position = (
            None
            if navigation_pose is None
            else navigation_pose.pose.pose.position
        )
        state_estimate_position = (
            None
            if self.latest_state_estimate is None
            else self.latest_state_estimate.pose.pose.position
        )
        applied_correction = (
            None
            if self.latest_localization_applied_dx_m is None
            or self.latest_localization_applied_dy_m is None
            else math.hypot(
                self.latest_localization_applied_dx_m,
                self.latest_localization_applied_dy_m,
            )
        )

        raw_linear_x = (
            None if self.latest_raw_command is None else self.latest_raw_command.linear.x
        )
        raw_angular_z = (
            None if self.latest_raw_command is None else self.latest_raw_command.angular.z
        )
        safe_linear_x = (
            None
            if self.latest_safe_command is None
            else self.latest_safe_command.linear.x
        )
        safe_angular_z = (
            None
            if self.latest_safe_command is None
            else self.latest_safe_command.angular.z
        )
        self.trace_rows.append(
            {
                "time_s": now - self.started_at,
                "x_m": current_x,
                "y_m": current_y,
                "odom_timestamp_s": self.stamp_seconds(self.latest_odom),
                "navigation_timestamp_s": (
                    None
                    if navigation_pose is None
                    else self.stamp_seconds(navigation_pose)
                ),
                "state_estimate_timestamp_s": (
                    None
                    if self.latest_state_estimate is None
                    else self.stamp_seconds(self.latest_state_estimate)
                ),
                "navigation_timestamp_offset_s": (
                    None
                    if navigation_pose is None
                    else self.stamp_seconds(navigation_pose)
                    - self.stamp_seconds(self.latest_odom)
                ),
                "state_estimate_timestamp_offset_s": (
                    None
                    if self.latest_state_estimate is None
                    else self.stamp_seconds(self.latest_state_estimate)
                    - self.stamp_seconds(self.latest_odom)
                ),
                "navigation_x_m": (
                    None if navigation_position is None else navigation_position.x
                ),
                "navigation_y_m": (
                    None if navigation_position is None else navigation_position.y
                ),
                "state_estimate_x_m": (
                    None
                    if state_estimate_position is None
                    else state_estimate_position.x
                ),
                "state_estimate_y_m": (
                    None
                    if state_estimate_position is None
                    else state_estimate_position.y
                ),
                "ground_truth_goal_error_m": ground_truth_goal_distance,
                "navigation_goal_error_m": navigation_goal_distance,
                "state_estimate_goal_error_m": state_estimate_goal_distance,
                "yaw_rad": self.yaw_from_quaternion(
                    self.latest_odom.pose.pose.orientation
                ),
                "goal_x_m": self.goal_x,
                "goal_y_m": self.goal_y,
                "raw_linear_x_mps": raw_linear_x,
                "raw_angular_z_radps": raw_angular_z,
                "executed_linear_x_mps": safe_linear_x,
                "executed_angular_z_radps": safe_angular_z,
                "front_clearance_m": clearance,
                "safety_override": self.latest_override_state,
                "collision": self.collision_state,
                "localization_candidate_correction_m": (
                    self.latest_localization_candidate_correction_m
                ),
                "localization_candidate_dx_m": self.latest_localization_candidate_dx_m,
                "localization_candidate_dy_m": self.latest_localization_candidate_dy_m,
                "localization_applied_correction_m": applied_correction,
                "localization_applied_dx_m": self.latest_localization_applied_dx_m,
                "localization_applied_dy_m": self.latest_localization_applied_dy_m,
                "localization_match_valid": self.latest_localization_match_valid,
                "localization_match_status": self.latest_localization_match_status,
                "localization_match_score_m": self.latest_localization_match_score_m,
                "localization_score_improvement_m": (
                    self.latest_localization_score_improvement_m
                ),
            }
        )

        self.last_sample_time = now
        self.sample_count += 1

        if self.goal_reached_at is not None and self.termination_reason is None:
            self.finish_run("goal_reached")

    def result(self) -> dict[str, object]:
        """Return one CSV-ready row with physical and navigation-pose metrics."""
        now = time.monotonic()

        # Use the last complete evaluation sample as the final snapshot.  The
        # timeout callback can shut down the executor before a final pose
        # callback is delivered; reading the live caches here would then mix
        # messages from different logical times and can disagree with the
        # time-series logger.  The live-cache calculation below remains as a
        # fallback for runs that ended before the first sample.
        final_sample = self.trace_rows[-1] if self.trace_rows else None
        final_error: Optional[float] = None
        if final_sample is not None:
            final_error = final_sample.get("ground_truth_goal_error_m")
        elif (
            self.latest_odom is not None
            and self.goal_x is not None
            and self.goal_y is not None
        ):
            position = self.latest_odom.pose.pose.position
            final_error = distance_2d(
                position.x,
                position.y,
                self.goal_x,
                self.goal_y,
            )

        navigation_final_error: Optional[float] = None
        navigation_pose = (
            self.latest_odom
            if self.navigation_pose_topic == "/odom"
            else self.latest_navigation_pose
        )
        if final_sample is not None:
            navigation_final_error = final_sample.get("navigation_goal_error_m")
        elif (
            navigation_pose is not None
            and self.goal_x is not None
            and self.goal_y is not None
        ):
            position = navigation_pose.pose.pose.position
            navigation_final_error = distance_2d(
                position.x,
                position.y,
                self.goal_x,
                self.goal_y,
            )

        state_estimate_final_error: Optional[float] = None
        if final_sample is not None:
            state_estimate_final_error = final_sample.get(
                "state_estimate_goal_error_m"
            )
        elif (
            self.latest_state_estimate is not None
            and self.goal_x is not None
            and self.goal_y is not None
        ):
            position = self.latest_state_estimate.pose.pose.position
            state_estimate_final_error = distance_2d(
                position.x,
                position.y,
                self.goal_x,
                self.goal_y,
            )

        elapsed = None if self.started_at is None else now - self.started_at
        time_to_goal = (
            None
            if self.started_at is None or self.goal_reached_at is None
            else self.goal_reached_at - self.started_at
        )
        navigation_time_to_goal = (
            None
            if self.started_at is None
            or self.navigation_goal_reached_at is None
            else self.navigation_goal_reached_at - self.started_at
        )
        state_estimate_time_to_goal = (
            None
            if self.started_at is None
            or self.state_estimate_goal_reached_at is None
            else self.state_estimate_goal_reached_at - self.started_at
        )
        navigation_goal_error_gap = (
            None
            if final_error is None or navigation_final_error is None
            else abs(navigation_final_error - final_error)
        )
        navigation_reached_before_ground_truth = (
            self.navigation_goal_reached_at is not None
            and (
                self.goal_reached_at is None
                or self.navigation_goal_reached_at < self.goal_reached_at
            )
        )
        return {
            "success": self.goal_reached_at is not None,
            "ground_truth_goal_reached": self.goal_reached_at is not None,
            "navigation_pose_topic": self.navigation_pose_topic,
            "navigation_pose_goal_reached": (
                self.navigation_goal_reached_at is not None
            ),
            "start_x": self.start_x,
            "start_y": self.start_y,
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "final_error_m": final_error,
            "ground_truth_final_error_m": final_error,
            "navigation_pose_final_error_m": navigation_final_error,
            "state_estimate_topic": self.state_estimate_topic,
            "state_estimate_goal_reached": (
                self.state_estimate_goal_reached_at is not None
            ),
            "state_estimate_final_error_m": state_estimate_final_error,
            "final_executed_linear_x_mps": (
                None
                if final_sample is None
                else final_sample.get("executed_linear_x_mps")
            ),
            "final_executed_angular_z_radps": (
                None
                if final_sample is None
                else final_sample.get("executed_angular_z_radps")
            ),
            "evaluation_final_sample_time_s": (
                None if final_sample is None else final_sample.get("time_s")
            ),
            "evaluation_final_odom_timestamp_s": (
                None
                if final_sample is None
                else final_sample.get("odom_timestamp_s")
            ),
            "evaluation_final_navigation_timestamp_s": (
                None
                if final_sample is None
                else final_sample.get("navigation_timestamp_s")
            ),
            "evaluation_final_state_estimate_timestamp_s": (
                None
                if final_sample is None
                else final_sample.get("state_estimate_timestamp_s")
            ),
            "evaluation_final_navigation_timestamp_offset_s": (
                None
                if final_sample is None
                else final_sample.get("navigation_timestamp_offset_s")
            ),
            "evaluation_final_state_estimate_timestamp_offset_s": (
                None
                if final_sample is None
                else final_sample.get("state_estimate_timestamp_offset_s")
            ),
            "evaluation_final_errors_source": (
                "last_complete_trace_sample"
                if final_sample is not None
                else "live_cache_fallback"
            ),
            "navigation_pose_goal_error_gap_m": navigation_goal_error_gap,
            "navigation_pose_reached_before_ground_truth": (
                navigation_reached_before_ground_truth
            ),
            "elapsed_time_s": elapsed,
            "time_to_goal_s": time_to_goal,
            "navigation_pose_time_to_goal_s": navigation_time_to_goal,
            "state_estimate_time_to_goal_s": state_estimate_time_to_goal,
            "initial_planned_path_length_m": self.initial_planned_path_length,
            "latest_planned_path_length_m": self.latest_planned_path_length,
            "plan_update_count": self.plan_update_count,
            "travelled_distance_m": self.travelled_distance,
            # Efficiency is undefined for incomplete runs: partial travel is
            # not comparable with the full initial plan length.
            "path_efficiency": (
                None
                if self.goal_reached_at is None
                or self.initial_planned_path_length is None
                or self.travelled_distance <= 0.0
                else self.initial_planned_path_length / self.travelled_distance
            ),
            # This ratio is descriptive only.  A continuous trajectory can cut
            # grid corners, so a value below one does not mean A* found a
            # shorter discrete path.
            "path_length_ratio": (
                None
                if self.goal_reached_at is None
                or self.initial_planned_path_length is None
                or self.initial_planned_path_length <= 0.0
                else self.travelled_distance / self.initial_planned_path_length
            ),
            "configured_minimum_clearance_m": self.configured_minimum_clearance,
            "configured_sensor_latency_s": self.configured_sensor_latency,
            "configured_scan_delay_s": self.configured_scan_delay,
            "configured_scan_noise_std_m": self.configured_scan_noise_std,
            "configured_scan_noise_seed": self.configured_scan_noise_seed,
            "configured_safety_margin_m": self.configured_safety_margin,
            "configured_planning_radius_m": self.configured_planning_radius,
            "configured_effective_planning_clearance_m": (
                self.configured_effective_planning_radius
            ),
            "configured_experiment_timeout_s": self.configured_experiment_timeout,
            "termination_reason": self.termination_reason or "external_interrupt",
            "minimum_clearance_m": (
                None
                if math.isinf(self.minimum_clearance)
                else self.minimum_clearance
            ),
            "safety_override_count": self.safety_override_count,
            "safety_override_time_s": self.safety_override_time,
            "motion_time_s": self.motion_time,
            "safety_override_ratio": (
                None
                if self.motion_time <= 0.0
                else self.safety_override_time / self.motion_time
            ),
            "collision": (
                "unknown"
                if self.collision_state is None
                else str(self.collision_state).lower()
            ),
            "localization_match_valid": self.latest_localization_match_valid,
            "localization_match_status": self.latest_localization_match_status,
            "localization_candidate_correction_m": (
                self.latest_localization_candidate_correction_m
            ),
            "localization_candidate_dx_m": self.latest_localization_candidate_dx_m,
            "localization_candidate_dy_m": self.latest_localization_candidate_dy_m,
            "localization_applied_correction_m": (
                None
                if self.latest_localization_applied_dx_m is None
                or self.latest_localization_applied_dy_m is None
                else math.hypot(
                    self.latest_localization_applied_dx_m,
                    self.latest_localization_applied_dy_m,
                )
            ),
            "localization_applied_dx_m": self.latest_localization_applied_dx_m,
            "localization_applied_dy_m": self.latest_localization_applied_dy_m,
            "localization_match_score_m": self.latest_localization_match_score_m,
            "localization_score_improvement_m": (
                self.latest_localization_score_improvement_m
            ),
            "samples": self.sample_count,
        }

    def report(self) -> None:
        metrics = self.result()
        print("Evaluation summary")
        for key, value in metrics.items():
            print(f"{key}: {value}")

        if self.output_path:
            output = Path(self.output_path)
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(metrics))
                writer.writeheader()
                writer.writerow(metrics)
            print(f"Wrote evaluation metrics to {output}")

        if self.trace_output:
            trace_path = Path(self.trace_output)
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            trace_fields = [
                "time_s",
                "x_m",
                "y_m",
                "odom_timestamp_s",
                "navigation_timestamp_s",
                "state_estimate_timestamp_s",
                "navigation_timestamp_offset_s",
                "state_estimate_timestamp_offset_s",
                "navigation_x_m",
                "navigation_y_m",
                "state_estimate_x_m",
                "state_estimate_y_m",
                "ground_truth_goal_error_m",
                "navigation_goal_error_m",
                "state_estimate_goal_error_m",
                "yaw_rad",
                "goal_x_m",
                "goal_y_m",
                "raw_linear_x_mps",
                "raw_angular_z_radps",
                "executed_linear_x_mps",
                "executed_angular_z_radps",
                "front_clearance_m",
                "safety_override",
                "collision",
                "localization_candidate_correction_m",
                "localization_candidate_dx_m",
                "localization_candidate_dy_m",
                "localization_applied_correction_m",
                "localization_applied_dx_m",
                "localization_applied_dy_m",
                "localization_match_valid",
                "localization_match_status",
                "localization_match_score_m",
                "localization_score_improvement_m",
            ]
            with trace_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=trace_fields)
                writer.writeheader()
                writer.writerows(self.trace_rows)
            print(f"Wrote evaluation trace to {trace_path}")

        plan_message = self.initial_plan or self.latest_plan
        if self.plan_output and plan_message is not None:
            plan_path = Path(self.plan_output)
            plan_path.parent.mkdir(parents=True, exist_ok=True)
            with plan_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["index", "x_m", "y_m"])
                writer.writeheader()
                for index, pose in enumerate(plan_message.poses):
                    writer.writerow(
                        {
                            "index": index,
                            "x_m": pose.pose.position.x,
                            "y_m": pose.pose.position.y,
                        }
                    )
            print(f"Wrote evaluation plan to {plan_path}")

        if self.map_output and self.latest_map is not None:
            map_path = Path(self.map_output)
            map_path.parent.mkdir(parents=True, exist_ok=True)
            with map_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=["x_m", "y_m", "occupancy", "resolution_m"],
                )
                writer.writeheader()
                for row in range(self.latest_map.info.height):
                    for column in range(self.latest_map.info.width):
                        index = row * self.latest_map.info.width + column
                        if self.latest_map.data[index] < 50:
                            continue
                        writer.writerow(
                            {
                                "x_m": self.latest_map.info.origin.position.x
                                + (column + 0.5) * self.latest_map.info.resolution,
                                "y_m": self.latest_map.info.origin.position.y
                                + (row + 0.5) * self.latest_map.info.resolution,
                                "occupancy": self.latest_map.data[index],
                                "resolution_m": self.latest_map.info.resolution,
                            }
                        )
            print(f"Wrote evaluation map to {map_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.termination_reason is None:
            # SIGINT may come from the shell's timeout utility rather than a
            # person pressing Ctrl+C. Keep the source-neutral label explicit.
            node.termination_reason = "external_interrupt"
    except Exception:
        # rclpy raises ExternalShutdownException when another node requests a
        # coordinated shutdown. Re-raise genuine live-node failures, but let
        # the logger still write its final metrics during normal shutdown.
        if rclpy.ok():
            raise
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
