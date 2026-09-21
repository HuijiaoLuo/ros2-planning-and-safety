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
from std_msgs.msg import Float64

from robotics_nav.heading_fusion import (
    AdaptiveHeadingFusion,
    DifferentialDrivePoseModel,
    GyroMeasurementModel,
    HeadingFusion,
    WheelSlipMeasurementModel,
    wrap_angle,
)


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


def parameter_bool(value: object) -> bool:
    """Parse ROS launch values whether they arrive as bools or strings."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class HeadingEstimator(Node):
    """Publish wheel-odometry position with gyro/wheel-fused orientation."""

    def __init__(self) -> None:
        super().__init__("heading_estimator")

        self.declare_parameter("wheel_odom_topic", "/wheel_odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("output_topic", "/state_estimate")
        self.declare_parameter("fusion_mode", "fixed")
        self.declare_parameter("wheel_weight", 0.02)
        self.declare_parameter("gyro_rate_noise_std_rad_s", 0.01)
        self.declare_parameter("wheel_yaw_noise_std_rad", 0.07)
        self.declare_parameter("gyro_bias_random_walk_std_rad_s2", 0.001)
        self.declare_parameter("initial_heading_variance_rad2", 0.25)
        self.declare_parameter("initial_bias_variance_rad2_s2", 0.01)
        self.declare_parameter("adaptive_wheel_noise", False)
        self.declare_parameter("wheel_yaw_noise_min_std_rad", 0.02)
        self.declare_parameter("wheel_yaw_noise_max_std_rad", 0.20)
        self.declare_parameter("wheel_noise_adaptation_rate", 0.05)
        self.declare_parameter("fusion_gain_topic", "/heading_fusion_gain")
        self.declare_parameter("gyro_bias_estimate_topic", "/gyro_bias_estimate")
        self.declare_parameter("heading_innovation_topic", "/heading_fusion_innovation")
        self.declare_parameter(
            "wheel_yaw_noise_estimate_topic", "/wheel_yaw_noise_std_estimate"
        )
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("imu_gyro_bias_rad_s", 0.0)
        self.declare_parameter("imu_gyro_noise_std_rad_s", 0.0)
        self.declare_parameter("imu_gyro_noise_seed", 0)
        self.declare_parameter("wheel_slip_ratio", 0.0)
        self.declare_parameter("position_mode", "wheel_pose")

        wheel_topic = str(self.get_parameter("wheel_odom_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        fusion_mode = str(self.get_parameter("fusion_mode").value).lower()
        wheel_weight = float(self.get_parameter("wheel_weight").value)
        gyro_rate_noise = float(
            self.get_parameter("gyro_rate_noise_std_rad_s").value
        )
        wheel_yaw_noise = float(
            self.get_parameter("wheel_yaw_noise_std_rad").value
        )
        self.configured_wheel_yaw_noise = wheel_yaw_noise
        bias_random_walk = float(
            self.get_parameter("gyro_bias_random_walk_std_rad_s2").value
        )
        initial_heading_variance = float(
            self.get_parameter("initial_heading_variance_rad2").value
        )
        initial_bias_variance = float(
            self.get_parameter("initial_bias_variance_rad2_s2").value
        )
        adaptive_wheel_noise = parameter_bool(
            self.get_parameter("adaptive_wheel_noise").value
        )
        wheel_yaw_noise_min_std = float(
            self.get_parameter("wheel_yaw_noise_min_std_rad").value
        )
        wheel_yaw_noise_max_std = float(
            self.get_parameter("wheel_yaw_noise_max_std_rad").value
        )
        wheel_noise_adaptation_rate = float(
            self.get_parameter("wheel_noise_adaptation_rate").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        gyro_bias = float(self.get_parameter("imu_gyro_bias_rad_s").value)
        gyro_noise = float(
            self.get_parameter("imu_gyro_noise_std_rad_s").value
        )
        gyro_seed = int(
            float(self.get_parameter("imu_gyro_noise_seed").value)
        )
        wheel_slip_ratio = float(self.get_parameter("wheel_slip_ratio").value)
        position_mode = str(self.get_parameter("position_mode").value).lower()
        if position_mode not in {"wheel_pose", "propagated"}:
            raise ValueError("position_mode must be 'wheel_pose' or 'propagated'")

        if fusion_mode == "adaptive":
            self.fusion = AdaptiveHeadingFusion(
                gyro_rate_noise_std_rad_s=gyro_rate_noise,
                wheel_yaw_noise_std_rad=wheel_yaw_noise,
                gyro_bias_random_walk_std_rad_s2=bias_random_walk,
                initial_heading_variance_rad2=initial_heading_variance,
                initial_bias_variance_rad2_s2=initial_bias_variance,
                adaptive_wheel_noise=adaptive_wheel_noise,
                wheel_yaw_noise_min_std_rad=wheel_yaw_noise_min_std,
                wheel_yaw_noise_max_std_rad=wheel_yaw_noise_max_std,
                wheel_noise_adaptation_rate=wheel_noise_adaptation_rate,
            )
        elif fusion_mode == "fixed":
            self.fusion = HeadingFusion(wheel_weight=wheel_weight)
        else:
            raise ValueError("fusion_mode must be 'fixed' or 'adaptive'")
        self.fusion_mode = fusion_mode
        self.gyro_model = GyroMeasurementModel(
            bias_rad_s=gyro_bias,
            noise_std_rad_s=gyro_noise,
            seed=gyro_seed,
        )
        self.wheel_model = WheelSlipMeasurementModel(wheel_slip_ratio)
        self.position_model = DifferentialDrivePoseModel(wheel_slip_ratio)
        self.position_mode = position_mode
        self.latest_wheel_odom: Optional[Odometry] = None
        self.latest_wheel_pose: Optional[tuple[float, float, float]] = None
        self.latest_fused_yaw: Optional[float] = None
        self.last_status: Optional[str] = None

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.gain_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("fusion_gain_topic").value),
            10,
        )
        self.bias_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("gyro_bias_estimate_topic").value),
            10,
        )
        self.innovation_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("heading_innovation_topic").value),
            10,
        )
        self.wheel_noise_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("wheel_yaw_noise_estimate_topic").value),
            10,
        )
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
            f"fusion_mode={fusion_mode}, "
            f"wheel_weight={wheel_weight:.3f}, "
            f"gyro_rate_noise={gyro_rate_noise:.4f} rad/s, "
            f"wheel_yaw_noise={wheel_yaw_noise:.4f} rad, "
            f"adaptive_wheel_noise={adaptive_wheel_noise}, "
            f"wheel_noise_bounds=({wheel_yaw_noise_min_std:.4f}, "
            f"{wheel_yaw_noise_max_std:.4f}) rad, "
            f"gyro_bias={gyro_bias:.4f} rad/s, "
            f"gyro_noise_std={gyro_noise:.4f} rad/s, seed={gyro_seed}, "
            f"wheel_slip_ratio={wheel_slip_ratio:.3f}."
            f" position_mode={position_mode}."
        )

    def report_status(self, status: str) -> None:
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def wheel_odom_callback(self, message: Odometry) -> None:
        self.latest_wheel_odom = message
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        wheel_yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if self.position_mode == "propagated":
            wheel_pose = (position.x, position.y, wheel_yaw)
        else:
            wheel_pose = self.wheel_model.apply(
                position.x,
                position.y,
                wheel_yaw,
            )
        self.latest_fused_yaw = self.fusion.update_wheel(wheel_pose[2])
        if self.position_mode == "propagated":
            self.latest_wheel_pose = self.position_model.apply(
                position.x,
                position.y,
                stamp_seconds(message),
                message.twist.twist.linear.x,
                self.latest_fused_yaw,
            )
        else:
            self.latest_wheel_pose = wheel_pose

    def imu_callback(self, message: Imu) -> None:
        fused_yaw = self.fusion.update_gyro(
            self.gyro_model.apply(message.angular_velocity.z),
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
        estimate.pose.pose.position.x = self.latest_wheel_pose[0]
        estimate.pose.pose.position.y = self.latest_wheel_pose[1]
        estimate.pose.pose.position.z = self.latest_wheel_odom.pose.pose.position.z
        estimate.pose.pose.orientation.x, estimate.pose.pose.orientation.y, estimate.pose.pose.orientation.z, estimate.pose.pose.orientation.w = quaternion_from_yaw(  # noqa: E501
            wrap_angle(self.latest_fused_yaw)
        )
        estimate.twist = self.latest_wheel_odom.twist
        estimate.pose.covariance = self.latest_wheel_odom.pose.covariance
        estimate.twist.covariance = self.latest_wheel_odom.twist.covariance
        self.publisher.publish(estimate)
        self.gain_publisher.publish(Float64(data=float(self.fusion.last_gain)))
        self.bias_publisher.publish(
            Float64(data=float(self.fusion.bias_estimate))
        )
        self.innovation_publisher.publish(
            Float64(data=float(self.fusion.last_innovation))
        )
        wheel_noise_estimate = getattr(
            self.fusion,
            "wheel_yaw_noise_std_estimate",
            self.configured_wheel_yaw_noise,
        )
        self.wheel_noise_publisher.publish(Float64(data=float(wheel_noise_estimate)))
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
