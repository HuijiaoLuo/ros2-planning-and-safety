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
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


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
        self.declare_parameter("collision_topic", "/collision")
        self.declare_parameter("safety_override_topic", "/safety_override")

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.front_angle = math.radians(
            float(self.get_parameter("front_angle_deg").value)
        )
        sample_rate = float(self.get_parameter("sample_rate_hz").value)
        self.output_path = str(self.get_parameter("output_path").value)
        collision_topic = str(self.get_parameter("collision_topic").value)
        safety_override_topic = str(
            self.get_parameter("safety_override_topic").value
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_plan: Optional[NavPath] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_raw_command: Optional[Twist] = None
        self.latest_override_state: Optional[bool] = None
        self.collision_state: Optional[bool] = None

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

        self.travelled_distance = 0.0
        self.motion_time = 0.0
        self.minimum_clearance = float("inf")
        self.safety_override_count = 0
        self.safety_override_time = 0.0
        self.override_active = False
        self.sample_count = 0

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
        self.create_subscription(NavPath, "/plan", self.plan_callback, path_qos)
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
            Bool,
            safety_override_topic,
            self.override_state_callback,
            10,
        )
        self.create_subscription(
            Bool,
            collision_topic,
            self.collision_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / sample_rate, self.sample)

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def plan_callback(self, message: NavPath) -> None:
        self.latest_plan = message
        self.plan_update_count += 1
        if not message.poses:
            return

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

    def scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message

    def raw_command_callback(self, message: Twist) -> None:
        self.latest_raw_command = message

    def override_state_callback(self, message: Bool) -> None:
        self.latest_override_state = bool(message.data)

    def collision_callback(self, message: Bool) -> None:
        self.collision_state = bool(message.data)

    def front_clearance(self, scan: LaserScan) -> Optional[float]:
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

        if self.latest_scan is not None:
            clearance = self.front_clearance(self.latest_scan)
            if clearance is not None:
                self.minimum_clearance = min(self.minimum_clearance, clearance)

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

        if self.goal_x is not None and self.goal_y is not None:
            goal_distance = distance_2d(
                current_x,
                current_y,
                self.goal_x,
                self.goal_y,
            )
            if goal_distance <= self.goal_tolerance and self.goal_reached_at is None:
                self.goal_reached_at = now

        self.last_sample_time = now
        self.sample_count += 1

    def result(self) -> dict[str, object]:
        now = time.monotonic()
        final_error: Optional[float] = None
        if (
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

        elapsed = None if self.started_at is None else now - self.started_at
        time_to_goal = (
            None
            if self.started_at is None or self.goal_reached_at is None
            else self.goal_reached_at - self.started_at
        )
        return {
            "success": self.goal_reached_at is not None,
            "start_x": self.start_x,
            "start_y": self.start_y,
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "final_error_m": final_error,
            "elapsed_time_s": elapsed,
            "time_to_goal_s": time_to_goal,
            "initial_planned_path_length_m": self.initial_planned_path_length,
            "latest_planned_path_length_m": self.latest_planned_path_length,
            "plan_update_count": self.plan_update_count,
            "travelled_distance_m": self.travelled_distance,
            "path_efficiency": (
                None
                if self.initial_planned_path_length is None
                or self.travelled_distance <= 0.0
                else self.initial_planned_path_length / self.travelled_distance
            ),
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
            "samples": self.sample_count,
        }

    def report(self) -> None:
        metrics = self.result()
        print("Evaluation summary")
        for key, value in metrics.items():
            print(f"{key}: {value}")

        if not self.output_path:
            return

        output = Path(self.output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(metrics))
            writer.writeheader()
            writer.writerow(metrics)
        print(f"Wrote evaluation metrics to {output}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
