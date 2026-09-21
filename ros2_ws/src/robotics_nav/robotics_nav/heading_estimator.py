#!/usr/bin/env python3
"""Fuse wheel odometry and IMU yaw rate into an estimated odometry topic."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from robotics_nav.heading_fusion import HeadingFusion, wrap_angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def stamp_seconds(message: object) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


class HeadingEstimator(Node):
    """Publish wheel-odometry position with gyro/wheel-fused orientation."""

    def __init__(self) -> None:
        super().__init__("heading_estimator")

        self.declare_parameter("wheel_odom_topic", "/wheel_odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("output_topic", "/state_estimate")
        self.declare_parameter("wheel_weight", 0.02)
        self.declare_parameter("publish_rate_hz", 30.0)

        wheel_topic = str(self.get_parameter("wheel_odom_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        wheel_weight = float(self.get_parameter("wheel_weight").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        self.fusion = HeadingFusion(wheel_weight=wheel_weight)
        self.latest_wheel_odom: Optional[Odometry] = None
        self.latest_fused_yaw: Optional[float] = None
        self.last_status: Optional[str] = None

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(
            Odometry,
            wheel_topic,
            self.wheel_odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_estimate)

        self.get_logger().info(
            f"Heading estimator: {wheel_topic} + {imu_topic} -> {output_topic}; "
            f"wheel_weight={wheel_weight:.3f}."
        )

    def report_status(self, status: str) -> None:
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def wheel_odom_callback(self, message: Odometry) -> None:
        self.latest_wheel_odom = message
        orientation = message.pose.pose.orientation
        wheel_yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.latest_fused_yaw = self.fusion.update_wheel(wheel_yaw)

    def imu_callback(self, message: Imu) -> None:
        fused_yaw = self.fusion.update_gyro(
            message.angular_velocity.z,
            stamp_seconds(message),
        )
        if fused_yaw is not None:
            self.latest_fused_yaw = fused_yaw

    def publish_estimate(self) -> None:
        if self.latest_wheel_odom is None or not self.fusion.ready:
            self.report_status("Waiting for wheel odometry and IMU samples.")
            return
        if self.latest_fused_yaw is None:
            return

        estimate = Odometry()
        estimate.header = self.latest_wheel_odom.header
        estimate.header.frame_id = self.latest_wheel_odom.header.frame_id or "odom"
        estimate.child_frame_id = self.latest_wheel_odom.child_frame_id or "base_link"
        estimate.pose.pose.position = self.latest_wheel_odom.pose.pose.position
        estimate.pose.pose.orientation.x, estimate.pose.pose.orientation.y, estimate.pose.pose.orientation.z, estimate.pose.pose.orientation.w = quaternion_from_yaw(  # noqa: E501
            wrap_angle(self.latest_fused_yaw)
        )
        estimate.twist = self.latest_wheel_odom.twist
        estimate.pose.covariance = self.latest_wheel_odom.pose.covariance
        estimate.twist.covariance = self.latest_wheel_odom.twist.covariance
        self.publisher.publish(estimate)
        self.report_status("Publishing /state_estimate.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingEstimator()
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
