#!/usr/bin/env python3
"""Compare wheel and estimated pose against /odom for V3 evaluation only."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from robotics_nav.estimation_metrics import PoseErrorStats


def yaw_from_quaternion(orientation) -> float:
    sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(sin_yaw, cos_yaw)


def parameter_bool(value: object) -> bool:
    """Parse a ROS parameter supplied either as a bool or launch string."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class EstimationLogger(Node):
    """Measure estimator error without publishing or affecting navigation."""

    def __init__(self) -> None:
        super().__init__("estimation_logger")

        self.declare_parameter("ground_truth_topic", "/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel_odom")
        self.declare_parameter("estimate_topic", "/state_estimate")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("output_path", "")
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("configured_imu_gyro_bias_rad_s", 0.0)
        self.declare_parameter("configured_imu_gyro_noise_std_rad_s", 0.0)
        self.declare_parameter("configured_imu_gyro_noise_seed", 0)
        self.declare_parameter("configured_wheel_slip_ratio", 0.0)
        self.declare_parameter("configured_position_mode", "wheel_pose")
        self.declare_parameter("configured_wheel_weight", 0.02)
        self.declare_parameter("configured_fusion_mode", "fixed")
        self.declare_parameter("configured_gyro_rate_noise_std_rad_s", 0.01)
        self.declare_parameter("configured_wheel_yaw_noise_std_rad", 0.07)
        self.declare_parameter(
            "configured_gyro_bias_random_walk_std_rad_s2", 0.001
        )
        self.declare_parameter("configured_adaptive_wheel_noise", False)
        self.declare_parameter("configured_wheel_yaw_noise_min_std_rad", 0.02)
        self.declare_parameter("configured_wheel_yaw_noise_max_std_rad", 0.20)
        self.declare_parameter("configured_wheel_noise_adaptation_rate", 0.05)
        self.declare_parameter(
            "wheel_yaw_noise_estimate_topic", "/wheel_yaw_noise_std_estimate"
        )

        ground_truth_topic = str(self.get_parameter("ground_truth_topic").value)
        wheel_odom_topic = str(self.get_parameter("wheel_odom_topic").value)
        estimate_topic = str(self.get_parameter("estimate_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        self.output_path = str(self.get_parameter("output_path").value)
        sample_rate = float(self.get_parameter("sample_rate_hz").value)
        self.configured_gyro_bias = float(
            self.get_parameter("configured_imu_gyro_bias_rad_s").value
        )
        self.configured_gyro_noise = float(
            self.get_parameter("configured_imu_gyro_noise_std_rad_s").value
        )
        self.configured_gyro_seed = int(
            float(self.get_parameter("configured_imu_gyro_noise_seed").value)
        )
        self.configured_wheel_slip = float(
            self.get_parameter("configured_wheel_slip_ratio").value
        )
        self.configured_position_mode = str(
            self.get_parameter("configured_position_mode").value
        )
        self.configured_wheel_weight = float(
            self.get_parameter("configured_wheel_weight").value
        )
        self.configured_fusion_mode = str(
            self.get_parameter("configured_fusion_mode").value
        )
        self.configured_gyro_rate_noise = float(
            self.get_parameter("configured_gyro_rate_noise_std_rad_s").value
        )
        self.configured_wheel_yaw_noise = float(
            self.get_parameter("configured_wheel_yaw_noise_std_rad").value
        )
        self.configured_bias_random_walk = float(
            self.get_parameter(
                "configured_gyro_bias_random_walk_std_rad_s2"
            ).value
        )
        self.configured_adaptive_wheel_noise = parameter_bool(
            self.get_parameter("configured_adaptive_wheel_noise").value
        )
        self.configured_wheel_yaw_noise_min = float(
            self.get_parameter("configured_wheel_yaw_noise_min_std_rad").value
        )
        self.configured_wheel_yaw_noise_max = float(
            self.get_parameter("configured_wheel_yaw_noise_max_std_rad").value
        )
        self.configured_wheel_noise_rate = float(
            self.get_parameter("configured_wheel_noise_adaptation_rate").value
        )
        wheel_noise_topic = str(
            self.get_parameter("wheel_yaw_noise_estimate_topic").value
        )

        self.latest_truth: Optional[Odometry] = None
        self.latest_wheel: Optional[Odometry] = None
        self.latest_estimate: Optional[Odometry] = None
        self.latest_imu_yaw: Optional[float] = None
        self.latest_fusion_gain: Optional[float] = None
        self.latest_bias_estimate: Optional[float] = None
        self.latest_innovation: Optional[float] = None
        self.latest_wheel_yaw_noise: Optional[float] = None
        self.fusion_gain_values: list[float] = []
        self.wheel_yaw_noise_values: list[float] = []
        self.last_imu_stamp: Optional[float] = None
        self.imu_received = False
        self.last_truth_stamp: Optional[tuple[int, int]] = None
        self.wheel_stats = PoseErrorStats()
        self.imu_stats = PoseErrorStats()
        self.estimate_stats = PoseErrorStats()
        self.report_written = False

        self.create_subscription(
            Odometry,
            ground_truth_topic,
            self.truth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            wheel_odom_topic,
            self.wheel_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            estimate_topic,
            self.estimate_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            "/heading_fusion_gain",
            self.fusion_gain_callback,
            10,
        )
        self.create_subscription(
            Float64,
            "/gyro_bias_estimate",
            self.bias_estimate_callback,
            10,
        )
        self.create_subscription(
            Float64,
            "/heading_fusion_innovation",
            self.innovation_callback,
            10,
        )
        self.create_subscription(
            Float64,
            wheel_noise_topic,
            self.wheel_noise_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / sample_rate, self.sample)

        self.get_logger().info(
            f"Estimation logger: {ground_truth_topic} vs {wheel_odom_topic} and "
            f"{estimate_topic}, plus gyro integration from {imu_topic}; "
            f"output={self.output_path or 'disabled'}."
        )

    def truth_callback(self, message: Odometry) -> None:
        self.latest_truth = message

    def wheel_callback(self, message: Odometry) -> None:
        self.latest_wheel = message
        if self.latest_imu_yaw is None:
            # Establish the same initial yaw reference even if wheel odometry
            # arrives before the first IMU callback.
            self.latest_imu_yaw = self.pose_values(message)[2]

    def estimate_callback(self, message: Odometry) -> None:
        self.latest_estimate = message

    def imu_callback(self, message: Imu) -> None:
        # Match the estimator's initialization rule: the gyro integration
        # starts from the first wheel-odometry yaw, never from /odom.
        if self.latest_imu_yaw is None:
            if self.latest_wheel is None:
                return
            self.latest_imu_yaw = self.pose_values(self.latest_wheel)[2]

        stamp = float(message.header.stamp.sec) + 1.0e-9 * float(
            message.header.stamp.nanosec
        )
        if self.last_imu_stamp is not None:
            delta_time = stamp - self.last_imu_stamp
            if 0.0 < delta_time <= 1.0:
                self.latest_imu_yaw = self.latest_imu_yaw + (
                    float(message.angular_velocity.z) * delta_time
                )
                self.latest_imu_yaw = math.atan2(
                    math.sin(self.latest_imu_yaw),
                    math.cos(self.latest_imu_yaw),
                )
        self.last_imu_stamp = stamp
        self.imu_received = True

    def fusion_gain_callback(self, message: Float64) -> None:
        self.latest_fusion_gain = float(message.data)
        self.fusion_gain_values.append(self.latest_fusion_gain)

    def bias_estimate_callback(self, message: Float64) -> None:
        self.latest_bias_estimate = float(message.data)

    def innovation_callback(self, message: Float64) -> None:
        self.latest_innovation = float(message.data)

    def wheel_noise_callback(self, message: Float64) -> None:
        self.latest_wheel_yaw_noise = float(message.data)
        self.wheel_yaw_noise_values.append(self.latest_wheel_yaw_noise)

    @staticmethod
    def stamp_key(message: Odometry) -> tuple[int, int]:
        stamp = message.header.stamp
        return int(stamp.sec), int(stamp.nanosec)

    @staticmethod
    def pose_values(message: Odometry) -> tuple[float, float, float]:
        position = message.pose.pose.position
        return (
            float(position.x),
            float(position.y),
            yaw_from_quaternion(message.pose.pose.orientation),
        )

    def sample(self) -> None:
        """Add one time-aligned comparison sample to the three accumulators.

        Ground-truth `/odom` is used only here for evaluation.  The estimator
        itself receives wheel odometry and IMU data through separate topics.
        Reusing a truth timestamp prevents the logger timer from counting the
        same truth message repeatedly when callbacks arrive at different rates.
        """
        if (
            self.latest_truth is None
            or self.latest_wheel is None
            or self.latest_estimate is None
        ):
            return

        truth_stamp = self.stamp_key(self.latest_truth)
        if self.last_truth_stamp == truth_stamp:
            return
        self.last_truth_stamp = truth_stamp

        # Compare every estimate with the same physical reference pose.  The
        # IMU comparison keeps wheel x/y only so its heading error is isolated.
        truth = self.pose_values(self.latest_truth)
        wheel = self.pose_values(self.latest_wheel)
        estimate = self.pose_values(self.latest_estimate)
        self.wheel_stats.add(*truth, *wheel)
        self.estimate_stats.add(*truth, *estimate)
        if self.imu_received and self.latest_imu_yaw is not None:
            self.imu_stats.add(
                truth[0],
                truth[1],
                truth[2],
                wheel[0],
                wheel[1],
                self.latest_imu_yaw,
            )

    def result(self) -> dict[str, object]:
        """Return aggregate diagnostics for wheel, gyro-only, and fused pose."""
        result: dict[str, object] = {}
        result.update(self.wheel_stats.summary("wheel"))
        result.update(self.imu_stats.summary("imu"))
        result.update(self.estimate_stats.summary("estimate"))
        result["samples"] = self.estimate_stats.count
        return result

    def write_report(self) -> None:
        """Print and optionally write one reproducible estimator CSV row."""
        if self.report_written:
            return
        self.report_written = True
        metrics = self.result()
        metrics = {
            "configured_imu_gyro_bias_rad_s": self.configured_gyro_bias,
            "configured_imu_gyro_noise_std_rad_s": self.configured_gyro_noise,
            "configured_imu_gyro_noise_seed": self.configured_gyro_seed,
            "configured_wheel_slip_ratio": self.configured_wheel_slip,
            "configured_position_mode": self.configured_position_mode,
            "configured_wheel_weight": self.configured_wheel_weight,
            "configured_fusion_mode": self.configured_fusion_mode,
            "configured_gyro_rate_noise_std_rad_s": self.configured_gyro_rate_noise,
            "configured_wheel_yaw_noise_std_rad": self.configured_wheel_yaw_noise,
            "configured_gyro_bias_random_walk_std_rad_s2": self.configured_bias_random_walk,
            "configured_adaptive_wheel_noise": self.configured_adaptive_wheel_noise,
            "configured_wheel_yaw_noise_min_std_rad": self.configured_wheel_yaw_noise_min,
            "configured_wheel_yaw_noise_max_std_rad": self.configured_wheel_yaw_noise_max,
            "configured_wheel_noise_adaptation_rate": self.configured_wheel_noise_rate,
            "fusion_gain_samples": len(self.fusion_gain_values),
            "fusion_gain_mean": (
                sum(self.fusion_gain_values) / len(self.fusion_gain_values)
                if self.fusion_gain_values
                else None
            ),
            "fusion_gain_min": (
                min(self.fusion_gain_values) if self.fusion_gain_values else None
            ),
            "fusion_gain_max": (
                max(self.fusion_gain_values) if self.fusion_gain_values else None
            ),
            "final_bias_estimate_rad_s": self.latest_bias_estimate,
            "final_heading_innovation_rad": self.latest_innovation,
            "wheel_yaw_noise_estimate_samples": len(self.wheel_yaw_noise_values),
            "wheel_yaw_noise_estimate_mean_std_rad": (
                sum(self.wheel_yaw_noise_values) / len(self.wheel_yaw_noise_values)
                if self.wheel_yaw_noise_values
                else None
            ),
            "wheel_yaw_noise_estimate_min_std_rad": (
                min(self.wheel_yaw_noise_values)
                if self.wheel_yaw_noise_values
                else None
            ),
            "wheel_yaw_noise_estimate_max_std_rad": (
                max(self.wheel_yaw_noise_values)
                if self.wheel_yaw_noise_values
                else None
            ),
            "final_wheel_yaw_noise_estimate_std_rad": self.latest_wheel_yaw_noise,
            **metrics,
        }
        print("Estimation diagnostic summary")
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
        print(f"Wrote estimation diagnostics to {output}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstimationLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
