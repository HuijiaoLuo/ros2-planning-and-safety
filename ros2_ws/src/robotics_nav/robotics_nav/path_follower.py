#!/usr/bin/env python3
"""Follow a nav_msgs/Path with a simple differential-drive controller."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class PathFollower(Node):
    """Convert a planned path and odometry into raw velocity commands."""

    def __init__(self) -> None:
        super().__init__("path_follower")

        # Keep the target close to the grid path. A long lookahead makes the
        # unicycle controller cut corners around inflated obstacles.
        self.declare_parameter("lookahead_distance", 0.10)
        # Use a tighter final-position tolerance than the grid resolution so
        # the robot does not stop visibly far from the exact goal marker.
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("distance_gain", 0.8)
        self.declare_parameter("heading_gain", 2.0)
        self.declare_parameter("rotate_in_place_threshold", math.pi / 3.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.lookahead_distance = float(
            self.get_parameter("lookahead_distance").value
        )
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.distance_gain = float(self.get_parameter("distance_gain").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.rotate_in_place_threshold = float(
            self.get_parameter("rotate_in_place_threshold").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel_raw", 10)

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_subscription = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            path_qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.control_loop)

        self.latest_path: Optional[Path] = None
        self.latest_odom: Optional[Odometry] = None
        self.goal_reached = False
        self.last_control_state: Optional[str] = None

    def report_state(self, state: str) -> None:
        """Log only control-state transitions, not every timer tick."""
        if state != self.last_control_state:
            self.get_logger().info(state)
            self.last_control_state = state

    def path_callback(self, message: Path) -> None:
        self.latest_path = message
        self.goal_reached = False
        self.last_control_state = None
        if message.poses:
            endpoint = message.poses[-1].pose.position
            self.get_logger().info(
                f"Received path with {len(message.poses)} poses; "
                f"endpoint=({endpoint.x:.2f}, {endpoint.y:.2f})."
            )
        else:
            self.get_logger().warn("Received an empty path.")

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def select_target(self, x: float, y: float) -> Optional[tuple[float, float]]:
        if self.latest_path is None or not self.latest_path.poses:
            return None

        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in self.latest_path.poses
        ]
        nearest_index = min(
            range(len(points)),
            key=lambda index: math.hypot(points[index][0] - x, points[index][1] - y),
        )

        travelled = 0.0
        target_index = nearest_index
        for index in range(nearest_index, len(points) - 1):
            travelled += math.hypot(
                points[index + 1][0] - points[index][0],
                points[index + 1][1] - points[index][1],
            )
            target_index = index + 1
            if travelled >= self.lookahead_distance:
                break
        return points[target_index]

    def control_loop(self) -> None:
        if self.latest_path is None:
            self.report_state("Waiting for /plan.")
            self.publish_stop()
            return

        if self.latest_odom is None:
            self.report_state("Waiting for /odom.")
            self.publish_stop()
            return

        if not self.latest_path.poses:
            self.report_state("Stopped because /plan is empty.")
            self.publish_stop()
            return

        position = self.latest_odom.pose.pose.position
        orientation = self.latest_odom.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        goal = self.latest_path.poses[-1].pose.position
        goal_distance = math.hypot(goal.x - position.x, goal.y - position.y)
        if goal_distance <= self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info("Planned path goal reached.")
                self.goal_reached = True
            self.report_state(
                f"Stopped at planned endpoint; distance={goal_distance:.3f} m."
            )
            self.publish_stop()
            return

        target = self.select_target(position.x, position.y)
        if target is None:
            self.report_state("Stopped because no path target is available.")
            self.publish_stop()
            return

        target_x, target_y = target
        target_heading = math.atan2(target_y - position.y, target_x - position.x)
        heading_error = wrap_angle(target_heading - yaw)
        target_distance = math.hypot(target_x - position.x, target_y - position.y)

        command = Twist()
        command.angular.z = clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if abs(heading_error) <= self.rotate_in_place_threshold:
            command.linear.x = clamp(
                self.distance_gain * target_distance,
                0.0,
                self.max_linear_speed,
            )
        else:
            command.linear.x = 0.0

        self.report_state(
            f"Tracking path; target=({target_x:.2f}, {target_y:.2f}), "
            f"v={command.linear.x:.2f}, omega={command.angular.z:.2f}."
        )
        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
