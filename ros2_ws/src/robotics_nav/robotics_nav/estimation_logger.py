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
from std_msgs.msg import Bool, Float64

from robotics_nav.estimation_metrics import (
    PoseErrorStats,
    mahalanobis_squared_2d,
    normalized_squared_error,
    wrap_angle,
)


def yaw_from_quaternion(orientation) -> float:
    sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(sin_yaw, cos_yaw)


def parameter_bool(value: object) -> bool:
    """Parse a ROS parameter supplied either as a bool or launch string."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def truth_forward_increment(
    previous: tuple[float, float, float],
    current: tuple[float, float, float],
) -> float:
    """Project one ground-truth pose increment onto the robot forward axis.

    The ground-truth pose is used here only as an evaluation reference.  The
    midpoint heading avoids attributing a turn to lateral translation and
    matches the midpoint convention used by the propagated motion model.
    """
    previous_x, previous_y, previous_heading = previous
    current_x, current_y, current_heading = current
    midpoint_heading = wrap_angle(
        previous_heading
        + 0.5 * wrap_angle(current_heading - previous_heading)
    )
    delta_x = current_x - previous_x
    delta_y = current_y - previous_y
    return delta_x * math.cos(midpoint_heading) + delta_y * math.sin(
        midpoint_heading
    )


class EstimationLogger(Node):
    """Measure estimator error without publishing or affecting navigation."""

    def __init__(self) -> None:
        super().__init__("estimation_logger")

        self.declare_parameter("ground_truth_topic", "/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel_odom")
        self.declare_parameter("estimate_topic", "/state_estimate")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("output_path", "")
        self.declare_parameter("trace_output_path", "")
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("configured_imu_gyro_bias_rad_s", 0.0)
        self.declare_parameter("configured_imu_gyro_noise_std_rad_s", 0.0)
        self.declare_parameter("configured_imu_gyro_noise_seed", 0)
        self.declare_parameter("configured_wheel_slip_ratio", 0.0)
        self.declare_parameter("configured_wheel_slip_noise_std", 0.0)
        self.declare_parameter("configured_position_mode", "propagated")
        self.declare_parameter("configured_wheel_weight", 0.02)
        self.declare_parameter("configured_fusion_mode", "ekf")
        self.declare_parameter("configured_external_position_fusion", False)
        self.declare_parameter("configured_gyro_rate_noise_std_rad_s", 0.01)
        self.declare_parameter("configured_wheel_yaw_noise_std_rad", 0.20)
        self.declare_parameter("configured_gyro_bias_mode", "fixed")
        self.declare_parameter("configured_initial_gyro_bias_rad_s", 0.0)
        self.declare_parameter(
            "configured_gyro_bias_random_walk_std_rad_s2", 0.001
        )
        self.declare_parameter("configured_adaptive_wheel_noise", False)
        self.declare_parameter("configured_wheel_yaw_noise_min_std_rad", 0.02)
        self.declare_parameter("configured_wheel_yaw_noise_max_std_rad", 0.20)
        self.declare_parameter("configured_wheel_noise_adaptation_rate", 0.05)
        self.declare_parameter("configured_wheel_speed_noise_std_m_s", 0.02)
        self.declare_parameter("configured_nis_gate_threshold", 9.0)
        self.declare_parameter(
            "configured_wheel_yaw_bias_random_walk_std_rad_sqrt_s", 0.001
        )
        self.declare_parameter("configured_initial_position_variance_m2", 0.25)
        self.declare_parameter("configured_initial_heading_variance_rad2", 0.25)
        self.declare_parameter(
            "configured_initial_bias_variance_rad2_s2", 0.01
        )
        self.declare_parameter(
            "configured_initial_wheel_yaw_bias_variance_rad2", 0.01
        )
        self.declare_parameter(
            "wheel_yaw_noise_estimate_topic", "/wheel_yaw_noise_std_estimate"
        )
        self.declare_parameter(
            "wheel_yaw_bias_estimate_topic", "/wheel_yaw_bias_estimate"
        )

        ground_truth_topic = str(self.get_parameter("ground_truth_topic").value)
        wheel_odom_topic = str(self.get_parameter("wheel_odom_topic").value)
        estimate_topic = str(self.get_parameter("estimate_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self.trace_output_path = str(
            self.get_parameter("trace_output_path").value
        )
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
        self.configured_wheel_slip_noise = float(
            self.get_parameter("configured_wheel_slip_noise_std").value
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
        self.configured_external_position_fusion = parameter_bool(
            self.get_parameter("configured_external_position_fusion").value
        )
        self.configured_gyro_rate_noise = float(
            self.get_parameter("configured_gyro_rate_noise_std_rad_s").value
        )
        self.configured_wheel_yaw_noise = float(
            self.get_parameter("configured_wheel_yaw_noise_std_rad").value
        )
        self.configured_gyro_bias_mode = str(
            self.get_parameter("configured_gyro_bias_mode").value
        )
        self.configured_initial_gyro_bias = float(
            self.get_parameter("configured_initial_gyro_bias_rad_s").value
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
        self.configured_wheel_speed_noise = float(
            self.get_parameter("configured_wheel_speed_noise_std_m_s").value
        )
        self.configured_nis_gate = float(
            self.get_parameter("configured_nis_gate_threshold").value
        )
        self.configured_wheel_yaw_bias_random_walk = float(
            self.get_parameter(
                "configured_wheel_yaw_bias_random_walk_std_rad_sqrt_s"
            ).value
        )
        self.configured_initial_position_variance = float(
            self.get_parameter("configured_initial_position_variance_m2").value
        )
        self.configured_initial_heading_variance = float(
            self.get_parameter("configured_initial_heading_variance_rad2").value
        )
        self.configured_initial_bias_variance = float(
            self.get_parameter("configured_initial_bias_variance_rad2_s2").value
        )
        self.configured_initial_wheel_yaw_bias_variance = float(
            self.get_parameter(
                "configured_initial_wheel_yaw_bias_variance_rad2"
            ).value
        )
        wheel_noise_topic = str(
            self.get_parameter("wheel_yaw_noise_estimate_topic").value
        )
        wheel_yaw_bias_topic = str(
            self.get_parameter("wheel_yaw_bias_estimate_topic").value
        )

        self.latest_truth: Optional[Odometry] = None
        self.latest_wheel: Optional[Odometry] = None
        self.latest_estimate: Optional[Odometry] = None
        self.latest_imu_yaw: Optional[float] = None
        self.latest_fusion_gain: Optional[float] = None
        self.latest_bias_estimate: Optional[float] = None
        self.latest_wheel_yaw_bias_estimate: Optional[float] = None
        self.latest_innovation: Optional[float] = None
        self.latest_wheel_yaw_noise: Optional[float] = None
        self.latest_nis: Optional[float] = None
        self.latest_measurement_accepted = True
        self.latest_external_nis: Optional[float] = None
        self.latest_external_measurement_age_s: Optional[float] = None
        self.external_nis_values: list[float] = []
        self.external_measurement_age_values: list[float] = []
        self.external_position_update_count = 0
        self.external_position_update_accepted_count = 0
        self.external_measurement_replayed_count = 0
        self.fusion_gain_values: list[float] = []
        self.wheel_yaw_noise_values: list[float] = []
        self.nis_values: list[float] = []
        self.measurement_rejection_count = 0
        self.estimate_covariance_x_values: list[float] = []
        self.estimate_covariance_y_values: list[float] = []
        self.estimate_covariance_yaw_values: list[float] = []
        self.trace_handle = None
        self.trace_writer: Optional[csv.DictWriter] = None
        self.trace_sample_index = 0
        self.last_imu_stamp: Optional[float] = None
        self.last_sample_truth_timestamp_s: Optional[float] = None
        self.last_sample_estimate_timestamp_s: Optional[float] = None
        self.imu_received = False
        self.last_truth_stamp: Optional[tuple[int, int]] = None
        self.last_motion_truth: Optional[tuple[float, tuple[float, float, float]]] = None
        self.motion_audit_samples = 0
        self.motion_wheel_distance_m = 0.0
        self.motion_model_distance_m = 0.0
        self.motion_truth_forward_distance_m = 0.0
        self.motion_speed_model_values: list[float] = []
        self.motion_speed_truth_values: list[float] = []
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
            wheel_yaw_bias_topic,
            self.wheel_yaw_bias_callback,
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
            "/heading_fusion_nis",
            self.nis_callback,
            10,
        )
        self.create_subscription(
            Float64,
            "/external_position_fusion_nis",
            self.external_nis_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/external_position_measurement_accepted",
            self.external_measurement_accepted_callback,
            10,
        )
        self.create_subscription(
            Float64,
            "/external_position_measurement_age_s",
            self.external_measurement_age_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/external_position_measurement_replayed",
            self.external_measurement_replayed_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/heading_measurement_accepted",
            self.measurement_accepted_callback,
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
            f"output={self.output_path or 'disabled'}, "
            f"trace={self.trace_output_path or 'disabled'}."
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
        covariance = message.pose.covariance
        if len(covariance) >= 36:
            self.estimate_covariance_x_values.append(float(covariance[0]))
            self.estimate_covariance_y_values.append(float(covariance[7]))
            self.estimate_covariance_yaw_values.append(float(covariance[35]))

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

    def wheel_yaw_bias_callback(self, message: Float64) -> None:
        """Store the EKF's estimated wheel-yaw bias for the final report."""
        self.latest_wheel_yaw_bias_estimate = float(message.data)

    def innovation_callback(self, message: Float64) -> None:
        self.latest_innovation = float(message.data)

    def nis_callback(self, message: Float64) -> None:
        self.latest_nis = float(message.data)
        self.nis_values.append(self.latest_nis)

    def external_nis_callback(self, message: Float64) -> None:
        """Record one NIS value for an external map-position event."""
        self.latest_external_nis = float(message.data)
        self.external_nis_values.append(self.latest_external_nis)

    def external_measurement_accepted_callback(self, message: Bool) -> None:
        """Count each external-position event emitted by the coupled EKF."""
        self.external_position_update_count += 1
        if bool(message.data):
            self.external_position_update_accepted_count += 1

    def external_measurement_age_callback(self, message: Float64) -> None:
        """Record the delay between map observation time and filter time."""
        self.latest_external_measurement_age_s = float(message.data)
        self.external_measurement_age_values.append(
            self.latest_external_measurement_age_s
        )

    def external_measurement_replayed_callback(self, message: Bool) -> None:
        """Count external events handled by timestamped history replay."""
        if bool(message.data):
            self.external_measurement_replayed_count += 1

    def measurement_accepted_callback(self, message: Bool) -> None:
        accepted = bool(message.data)
        # The estimator republishes the latest gate state at its output rate,
        # so count the beginning of a rejected episode rather than every
        # repeated ``False`` message.
        if not accepted and self.latest_measurement_accepted:
            self.measurement_rejection_count += 1
        self.latest_measurement_accepted = accepted

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

    @staticmethod
    def stamp_seconds(message: Odometry) -> float:
        """Convert a ROS time stamp to seconds for trace alignment diagnostics."""
        stamp = message.header.stamp
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)

    def motion_audit_sample(
        self,
        truth: tuple[float, float, float],
        wheel: Odometry,
        truth_timestamp_s: float,
    ) -> dict[str, float | None]:
        """Compare wheel-speed propagation with truth, for evaluation only.

        The estimator consumes the wheel speed and IMU rate independently.
        This audit reconstructs the translational increment that the current
        model would use, then compares it with the ground-truth displacement
        projected along the true body-forward direction.  No audit quantity is
        sent back to the estimator or controller.
        """
        wheel_speed = float(wheel.twist.twist.linear.x)
        wheel_timestamp_s = self.stamp_seconds(wheel)
        wheel_age_s = truth_timestamp_s - wheel_timestamp_s
        result: dict[str, float | None] = {
            "truth_dt_s": None,
            "wheel_timestamp_s": wheel_timestamp_s,
            "wheel_age_s": wheel_age_s,
            "wheel_speed_m_s": wheel_speed,
            "truth_forward_speed_m_s": None,
            "wheel_speed_increment_m": None,
            "model_distance_increment_m": None,
            "truth_forward_increment_m": None,
            "cumulative_model_distance_m": self.motion_model_distance_m,
            "cumulative_truth_forward_distance_m": self.motion_truth_forward_distance_m,
            "model_truth_speed_ratio": None,
        }

        if self.last_motion_truth is None:
            self.last_motion_truth = (truth_timestamp_s, truth)
            return result

        previous_timestamp_s, previous_truth = self.last_motion_truth
        delta_time = truth_timestamp_s - previous_timestamp_s
        self.last_motion_truth = (truth_timestamp_s, truth)
        if not 0.0 < delta_time <= 1.0:
            return result

        truth_increment = truth_forward_increment(previous_truth, truth)
        wheel_increment = wheel_speed * delta_time
        model_increment = (1.0 - self.configured_wheel_slip) * wheel_increment
        truth_speed = truth_increment / delta_time
        model_speed = model_increment / delta_time

        self.motion_audit_samples += 1
        self.motion_wheel_distance_m += wheel_increment
        self.motion_model_distance_m += model_increment
        self.motion_truth_forward_distance_m += truth_increment
        self.motion_speed_model_values.append(model_speed)
        self.motion_speed_truth_values.append(truth_speed)

        result.update(
            {
                "truth_dt_s": delta_time,
                "truth_forward_speed_m_s": truth_speed,
                "wheel_speed_increment_m": wheel_increment,
                "model_distance_increment_m": model_increment,
                "truth_forward_increment_m": truth_increment,
                "cumulative_model_distance_m": self.motion_model_distance_m,
                "cumulative_truth_forward_distance_m": self.motion_truth_forward_distance_m,
                "model_truth_speed_ratio": (
                    model_speed / truth_speed
                    if abs(truth_speed) > 1.0e-6
                    else None
                ),
            }
        )
        return result

    def write_trace_sample(
        self,
        truth_message: Odometry,
        estimate_message: Odometry,
        truth: tuple[float, float, float],
        estimate: tuple[float, float, float],
        motion_audit: dict[str, float | None],
    ) -> None:
        """Write one truth-versus-estimate row for covariance calibration.

        This file is deliberately separate from the one-row report. The
        aggregate report is convenient for sweep tables, while calibration
        requires the time history of errors and the covariance claimed by the
        filter at the same sample. Ground truth enters only this logger.
        """
        if not self.trace_output_path:
            return

        covariance = estimate_message.pose.covariance
        if len(covariance) >= 36:
            covariance_xx = float(covariance[0])
            covariance_xy = 0.5 * (float(covariance[1]) + float(covariance[6]))
            covariance_yy = float(covariance[7])
            covariance_yaw = float(covariance[35])
        else:
            covariance_xx = math.nan
            covariance_xy = math.nan
            covariance_yy = math.nan
            covariance_yaw = math.nan

        error_x = estimate[0] - truth[0]
        error_y = estimate[1] - truth[1]
        error_yaw = wrap_angle(estimate[2] - truth[2])
        error_position = math.hypot(error_x, error_y)
        position_normalized = mahalanobis_squared_2d(
            error_x,
            error_y,
            covariance_xx,
            covariance_xy,
            covariance_yy,
        )
        yaw_normalized = normalized_squared_error(error_yaw, covariance_yaw)

        fieldnames = [
            "sample_index",
            "truth_timestamp_s",
            "estimate_timestamp_s",
            "truth_x_m",
            "truth_y_m",
            "truth_heading_rad",
            "estimate_x_m",
            "estimate_y_m",
            "estimate_heading_rad",
            "error_x_m",
            "error_y_m",
            "error_heading_rad",
            "position_error_m",
            "estimate_covariance_xx_m2",
            "estimate_covariance_xy_m2",
            "estimate_covariance_yy_m2",
            "estimate_covariance_yaw_rad2",
            "position_normalized_error_sq",
            "heading_normalized_error_sq",
            "truth_dt_s",
            "wheel_timestamp_s",
            "wheel_age_s",
            "wheel_speed_m_s",
            "truth_forward_speed_m_s",
            "wheel_speed_increment_m",
            "model_distance_increment_m",
            "truth_forward_increment_m",
            "cumulative_model_distance_m",
            "cumulative_truth_forward_distance_m",
            "model_truth_speed_ratio",
            "wheel_heading_rad",
            "wheel_heading_error_rad",
            "imu_heading_rad",
            "imu_heading_error_rad",
            "estimated_wheel_yaw_bias_rad",
            "heading_innovation_rad",
        ]
        if self.trace_writer is None:
            output = Path(self.trace_output_path)
            output.parent.mkdir(parents=True, exist_ok=True)
            self.trace_handle = output.open("w", newline="", encoding="utf-8")
            self.trace_writer = csv.DictWriter(
                self.trace_handle,
                fieldnames=fieldnames,
            )
            self.trace_writer.writeheader()

        self.trace_writer.writerow(
            {
                "sample_index": self.trace_sample_index,
                "truth_timestamp_s": self.stamp_seconds(truth_message),
                "estimate_timestamp_s": self.stamp_seconds(estimate_message),
                "truth_x_m": truth[0],
                "truth_y_m": truth[1],
                "truth_heading_rad": truth[2],
                "estimate_x_m": estimate[0],
                "estimate_y_m": estimate[1],
                "estimate_heading_rad": estimate[2],
                "error_x_m": error_x,
                "error_y_m": error_y,
                "error_heading_rad": error_yaw,
                "position_error_m": error_position,
                "estimate_covariance_xx_m2": covariance_xx,
                "estimate_covariance_xy_m2": covariance_xy,
                "estimate_covariance_yy_m2": covariance_yy,
                "estimate_covariance_yaw_rad2": covariance_yaw,
                "position_normalized_error_sq": position_normalized,
                "heading_normalized_error_sq": yaw_normalized,
                **motion_audit,
            }
        )
        self.trace_handle.flush()
        self.trace_sample_index += 1

    def close_trace(self) -> None:
        """Flush and close the optional per-sample trace file."""
        if self.trace_handle is not None:
            self.trace_handle.flush()
            self.trace_handle.close()
            self.trace_handle = None
            self.trace_writer = None

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
        motion_audit = self.motion_audit_sample(
            truth,
            self.latest_wheel,
            self.stamp_seconds(self.latest_truth),
        )
        wheel_heading = wheel[2]
        motion_audit.update(
            {
                "wheel_heading_rad": wheel_heading,
                "wheel_heading_error_rad": wrap_angle(wheel_heading - truth[2]),
                "imu_heading_rad": self.latest_imu_yaw,
                "imu_heading_error_rad": (
                    wrap_angle(self.latest_imu_yaw - truth[2])
                    if self.latest_imu_yaw is not None
                    else None
                ),
                "estimated_wheel_yaw_bias_rad": self.latest_wheel_yaw_bias_estimate,
                "heading_innovation_rad": self.latest_innovation,
            }
        )
        self.wheel_stats.add(*truth, *wheel)
        self.estimate_stats.add(*truth, *estimate)
        self.write_trace_sample(
            self.latest_truth,
            self.latest_estimate,
            truth,
            estimate,
            motion_audit,
        )
        self.last_sample_truth_timestamp_s = self.stamp_seconds(self.latest_truth)
        self.last_sample_estimate_timestamp_s = self.stamp_seconds(
            self.latest_estimate
        )
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
        result["last_sample_truth_timestamp_s"] = self.last_sample_truth_timestamp_s
        result["last_sample_estimate_timestamp_s"] = (
            self.last_sample_estimate_timestamp_s
        )
        result["last_sample_estimate_timestamp_offset_s"] = (
            None
            if self.last_sample_truth_timestamp_s is None
            or self.last_sample_estimate_timestamp_s is None
            else self.last_sample_estimate_timestamp_s
            - self.last_sample_truth_timestamp_s
        )
        result["motion_audit_samples"] = self.motion_audit_samples
        result["motion_wheel_distance_m"] = self.motion_wheel_distance_m
        result["motion_model_distance_m"] = self.motion_model_distance_m
        result["motion_truth_forward_distance_m"] = (
            self.motion_truth_forward_distance_m
        )
        result["motion_model_distance_error_m"] = (
            self.motion_model_distance_m - self.motion_truth_forward_distance_m
        )
        result["motion_model_truth_speed_ratio"] = (
            sum(self.motion_speed_model_values)
            / sum(self.motion_speed_truth_values)
            if self.motion_speed_truth_values
            and abs(sum(self.motion_speed_truth_values)) > 1.0e-6
            else None
        )
        result["external_position_update_samples"] = (
            self.external_position_update_count
        )
        result["external_position_update_accepted_count"] = (
            self.external_position_update_accepted_count
        )
        result["external_position_update_rejected_count"] = (
            self.external_position_update_count
            - self.external_position_update_accepted_count
        )
        result["external_position_nis_samples"] = len(self.external_nis_values)
        result["external_position_nis_mean"] = (
            sum(self.external_nis_values) / len(self.external_nis_values)
            if self.external_nis_values
            else None
        )
        result["external_position_nis_max"] = (
            max(self.external_nis_values) if self.external_nis_values else None
        )
        result["external_measurement_age_mean_s"] = (
            sum(self.external_measurement_age_values)
            / len(self.external_measurement_age_values)
            if self.external_measurement_age_values
            else None
        )
        result["external_measurement_age_max_s"] = (
            max(self.external_measurement_age_values)
            if self.external_measurement_age_values
            else None
        )
        result["external_measurement_replayed_count"] = (
            self.external_measurement_replayed_count
        )
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
            "configured_wheel_slip_noise_std": self.configured_wheel_slip_noise,
            "configured_position_mode": self.configured_position_mode,
            "configured_wheel_weight": self.configured_wheel_weight,
            "configured_fusion_mode": self.configured_fusion_mode,
            "configured_external_position_fusion": (
                self.configured_external_position_fusion
            ),
            "configured_gyro_rate_noise_std_rad_s": self.configured_gyro_rate_noise,
            "configured_wheel_yaw_noise_std_rad": self.configured_wheel_yaw_noise,
            "configured_gyro_bias_mode": self.configured_gyro_bias_mode,
            "configured_initial_gyro_bias_rad_s": self.configured_initial_gyro_bias,
            "configured_gyro_bias_random_walk_std_rad_s2": self.configured_bias_random_walk,
            "configured_adaptive_wheel_noise": self.configured_adaptive_wheel_noise,
            "configured_wheel_yaw_noise_min_std_rad": self.configured_wheel_yaw_noise_min,
            "configured_wheel_yaw_noise_max_std_rad": self.configured_wheel_yaw_noise_max,
            "configured_wheel_noise_adaptation_rate": self.configured_wheel_noise_rate,
            "configured_wheel_speed_noise_std_m_s": self.configured_wheel_speed_noise,
            "configured_nis_gate_threshold": self.configured_nis_gate,
            "configured_wheel_yaw_bias_random_walk_std_rad_sqrt_s": self.configured_wheel_yaw_bias_random_walk,
            "configured_initial_position_variance_m2": self.configured_initial_position_variance,
            "configured_initial_heading_variance_rad2": self.configured_initial_heading_variance,
            "configured_initial_bias_variance_rad2_s2": self.configured_initial_bias_variance,
            "configured_initial_wheel_yaw_bias_variance_rad2": self.configured_initial_wheel_yaw_bias_variance,
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
            "final_wheel_yaw_bias_estimate_rad": self.latest_wheel_yaw_bias_estimate,
            "final_heading_innovation_rad": self.latest_innovation,
            "nis_samples": len(self.nis_values),
            "nis_mean": (
                sum(self.nis_values) / len(self.nis_values)
                if self.nis_values
                else None
            ),
            "nis_max": max(self.nis_values) if self.nis_values else None,
            "wheel_measurement_rejection_count": self.measurement_rejection_count,
            "estimate_covariance_samples": len(self.estimate_covariance_x_values),
            "mean_estimate_covariance_x_m2": (
                sum(self.estimate_covariance_x_values)
                / len(self.estimate_covariance_x_values)
                if self.estimate_covariance_x_values
                else None
            ),
            "mean_estimate_covariance_y_m2": (
                sum(self.estimate_covariance_y_values)
                / len(self.estimate_covariance_y_values)
                if self.estimate_covariance_y_values
                else None
            ),
            "mean_estimate_covariance_yaw_rad2": (
                sum(self.estimate_covariance_yaw_values)
                / len(self.estimate_covariance_yaw_values)
                if self.estimate_covariance_yaw_values
                else None
            ),
            "final_estimate_covariance_x_m2": (
                self.estimate_covariance_x_values[-1]
                if self.estimate_covariance_x_values
                else None
            ),
            "final_estimate_covariance_y_m2": (
                self.estimate_covariance_y_values[-1]
                if self.estimate_covariance_y_values
                else None
            ),
            "final_estimate_covariance_yaw_rad2": (
                self.estimate_covariance_yaw_values[-1]
                if self.estimate_covariance_yaw_values
                else None
            ),
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
            self.close_trace()
            return
        output = Path(self.output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(metrics))
            writer.writeheader()
            writer.writerow(metrics)
        print(f"Wrote estimation diagnostics to {output}")
        self.close_trace()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstimationLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # During SIGINT, Jazzy can tear down a DDS subscription while the
        # executor is converting the next queued message.  The resulting
        # pybind11 conversion error is a shutdown race, not an estimation
        # failure; the report is still finalized in the `finally` block.
        # Preserve all other live RuntimeError failures.
        conversion_race = "Unable to convert call argument" in str(exc)
        if rclpy.ok() and not conversion_race:
            raise
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
