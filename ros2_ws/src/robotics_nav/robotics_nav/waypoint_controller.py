#!/usr/bin/env python3
"""A small pose-to-velocity controller for a known waypoint."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointController(Node):
    """Convert odometry and a fixed goal into a raw velocity command."""

    def __init__(self) -> None:
        super().__init__("waypoint_controller")

        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tolerance", 0.08)
        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("distance_gain", 0.8)
        self.declare_parameter("heading_gain", 2.0)
        self.declare_parameter("rotate_in_place_threshold", math.pi / 3.0)

        self.publisher = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.distance_gain = float(self.get_parameter("distance_gain").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.rotate_in_place_threshold = float(
            self.get_parameter("rotate_in_place_threshold").value
        )
        self.goal_reached = False

        self.get_logger().info(
            f"Goal set to ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        dx = self.goal_x - position.x
        dy = self.goal_y - position.y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info("Goal reached; publishing zero velocity.")
                self.goal_reached = True
            self.publish_stop()
            return

        self.goal_reached = False
        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_heading - yaw)

        command = Twist()
        command.angular.z = clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        if abs(heading_error) <= self.rotate_in_place_threshold:
            command.linear.x = clamp(
                self.distance_gain * distance,
                0.0,
                self.max_linear_speed,
            )
        else:
            # Rotate before driving when the goal is substantially off-axis.
            command.linear.x = 0.0

        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointController()
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
