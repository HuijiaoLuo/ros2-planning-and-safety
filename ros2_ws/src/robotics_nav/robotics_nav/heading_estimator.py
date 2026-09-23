#!/usr/bin/env python3
"""Fuse wheel odometry and IMU yaw rate into an estimated odometry topic."""

from __future__ import annotations

import copy
import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64

from robotics_nav.heading_fusion import (
    AdaptiveHeadingFusion,
    DifferentialDrivePoseModel,
    GyroMeasurementModel,
    HeadingFusion,
    # PoseEKF is kept in its own module because V4 propagates x/y covariance,
    # while the earlier heading filters only estimate yaw and gyro bias.
    WheelSlipMeasurementModel,
    wrap_angle,
)
from robotics_nav.pose_ekf import PoseEKF


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
        self.declare_parameter("motion_prior_topic", "/state_prediction")
        self.declare_parameter("external_position_fusion", False)
        self.declare_parameter(
            "external_position_topic", "/localization_candidate"
        )
        self.declare_parameter("fusion_mode", "ekf")
        self.declare_parameter("wheel_weight", 0.02)
        self.declare_parameter("gyro_rate_noise_std_rad_s", 0.01)
        self.declare_parameter("wheel_yaw_noise_std_rad", 0.20)
        self.declare_parameter("gyro_bias_random_walk_std_rad_s2", 0.001)
        self.declare_parameter("gyro_bias_mode", "fixed")
        self.declare_parameter("initial_gyro_bias_rad_s", 0.0)
        self.declare_parameter("wheel_yaw_bias_random_walk_std_rad_sqrt_s", 0.001)
        self.declare_parameter("initial_position_variance_m2", 0.25)
        self.declare_parameter("initial_heading_variance_rad2", 0.25)
        self.declare_parameter("initial_bias_variance_rad2_s2", 0.01)
        self.declare_parameter("initial_wheel_yaw_bias_variance_rad2", 0.01)
        self.declare_parameter("adaptive_wheel_noise", False)
        self.declare_parameter("wheel_yaw_noise_min_std_rad", 0.02)
        self.declare_parameter("wheel_yaw_noise_max_std_rad", 0.20)
        self.declare_parameter("wheel_noise_adaptation_rate", 0.05)
        self.declare_parameter("wheel_speed_noise_std_m_s", 0.02)
        self.declare_parameter("wheel_slip_noise_std", 0.0)
        self.declare_parameter("nis_gate_threshold", 9.0)
        self.declare_parameter("fusion_gain_topic", "/heading_fusion_gain")
        self.declare_parameter("gyro_bias_estimate_topic", "/gyro_bias_estimate")
        self.declare_parameter(
            "wheel_yaw_bias_estimate_topic", "/wheel_yaw_bias_estimate"
        )
        self.declare_parameter("heading_innovation_topic", "/heading_fusion_innovation")
        self.declare_parameter("heading_nis_topic", "/heading_fusion_nis")
        self.declare_parameter(
            "heading_measurement_accepted_topic",
            "/heading_measurement_accepted",
        )
        self.declare_parameter(
            "wheel_yaw_noise_estimate_topic", "/wheel_yaw_noise_std_estimate"
        )
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("imu_gyro_bias_rad_s", 0.0)
        self.declare_parameter("imu_gyro_noise_std_rad_s", 0.0)
        self.declare_parameter("imu_gyro_noise_seed", 0)
        self.declare_parameter("wheel_slip_ratio", 0.0)
        self.declare_parameter("position_mode", "propagated")

        wheel_topic = str(self.get_parameter("wheel_odom_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        motion_prior_topic = str(
            self.get_parameter("motion_prior_topic").value
        ).strip()
        external_position_fusion = parameter_bool(
            self.get_parameter("external_position_fusion").value
        )
        external_position_topic = str(
            self.get_parameter("external_position_topic").value
        )
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
        gyro_bias_mode = str(self.get_parameter("gyro_bias_mode").value).lower()
        initial_gyro_bias = float(
            self.get_parameter("initial_gyro_bias_rad_s").value
        )
        wheel_yaw_bias_random_walk = float(
            self.get_parameter("wheel_yaw_bias_random_walk_std_rad_sqrt_s").value
        )
        initial_position_variance = float(
            self.get_parameter("initial_position_variance_m2").value
        )
        initial_heading_variance = float(
            self.get_parameter("initial_heading_variance_rad2").value
        )
        initial_bias_variance = float(
            self.get_parameter("initial_bias_variance_rad2_s2").value
        )
        initial_wheel_yaw_bias_variance = float(
            self.get_parameter("initial_wheel_yaw_bias_variance_rad2").value
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
        wheel_speed_noise_std = float(
            self.get_parameter("wheel_speed_noise_std_m_s").value
        )
        wheel_slip_noise_std = float(
            self.get_parameter("wheel_slip_noise_std").value
        )
        nis_gate_threshold = float(
            self.get_parameter("nis_gate_threshold").value
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

        # All modes consume the same wheel and IMU topics.  The fixed mode is
        # the transparent V3 baseline; adaptive mode tracks only heading/bias
        # covariance; EKF mode propagates the full pose plus gyro and
        # wheel-yaw bias states.
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
        elif fusion_mode == "ekf":
            self.fusion = PoseEKF(
                gyro_rate_noise_std_rad_s=gyro_rate_noise,
                wheel_yaw_noise_std_rad=wheel_yaw_noise,
                gyro_bias_random_walk_std_rad_s2=bias_random_walk,
                wheel_speed_noise_std_m_s=wheel_speed_noise_std,
                wheel_slip_ratio=wheel_slip_ratio,
                wheel_slip_noise_std=wheel_slip_noise_std,
                initial_position_variance_m2=initial_position_variance,
                initial_heading_variance_rad2=initial_heading_variance,
                initial_bias_variance_rad2_s2=initial_bias_variance,
                gyro_bias_mode=gyro_bias_mode,
                initial_gyro_bias_rad_s=initial_gyro_bias,
                initial_wheel_yaw_bias_variance_rad2=initial_wheel_yaw_bias_variance,
                wheel_yaw_bias_random_walk_std_rad_sqrt_s=wheel_yaw_bias_random_walk,
                nis_gate_threshold=nis_gate_threshold,
            )
        elif fusion_mode == "fixed":
            self.fusion = HeadingFusion(wheel_weight=wheel_weight)
        else:
            raise ValueError("fusion_mode must be 'fixed', 'adaptive', or 'ekf'")
        self.fusion_mode = fusion_mode
        # The navigation estimate may receive delayed absolute position
        # observations from a map localizer.  Keep a second EKF instance for
        # the motion prior so the localizer never feeds its own correction
        # back into the next particle prediction.  This is a structural
        # separation: both filters consume the same wheel/IMU stream, but only
        # ``self.fusion`` consumes external map-position events.
        self.motion_prior_fusion: Optional[PoseEKF] = (
            copy.deepcopy(self.fusion) if fusion_mode == "ekf" else None
        )
        # These perturbations are injected at the estimator input boundary.  In
        # particular, /odom is never used to create the navigation estimate.
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
        self.motion_prior_publisher = (
            self.create_publisher(Odometry, motion_prior_topic, 10)
            if motion_prior_topic
            else None
        )
        self.motion_prior_topic = motion_prior_topic
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
        self.wheel_yaw_bias_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("wheel_yaw_bias_estimate_topic").value),
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
        self.nis_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("heading_nis_topic").value),
            10,
        )
        self.measurement_accepted_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("heading_measurement_accepted_topic").value),
            10,
        )
        self.external_nis_publisher = self.create_publisher(
            Float64,
            "/external_position_fusion_nis",
            10,
        )
        self.external_measurement_accepted_publisher = self.create_publisher(
            Bool,
            "/external_position_measurement_accepted",
            10,
        )
        self.external_measurement_age_publisher = self.create_publisher(
            Float64,
            "/external_position_measurement_age_s",
            10,
        )
        self.external_measurement_replayed_publisher = self.create_publisher(
            Bool,
            "/external_position_measurement_replayed",
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
        if external_position_fusion and fusion_mode == "ekf":
            self.create_subscription(
                Odometry,
                external_position_topic,
                self.external_position_callback,
                qos_profile_sensor_data,
            )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_estimate)

        self.get_logger().info(
            f"Heading estimator: {wheel_topic} + {imu_topic} -> {output_topic}; "
            f"fusion_mode={fusion_mode}, "
            f"wheel_weight={wheel_weight:.3f}, "
            f"gyro_rate_noise={gyro_rate_noise:.4f} rad/s, "
            f"wheel_yaw_noise={wheel_yaw_noise:.4f} rad, "
            f"wheel_yaw_bias_rw={wheel_yaw_bias_random_walk:.4f} rad/sqrt(s), "
            f"adaptive_wheel_noise={adaptive_wheel_noise}, "
            f"wheel_noise_bounds=({wheel_yaw_noise_min_std:.4f}, "
            f"{wheel_yaw_noise_max_std:.4f}) rad, "
            f"wheel_speed_noise={wheel_speed_noise_std:.4f} m/s, "
            f"wheel_slip_noise_std={wheel_slip_noise_std:.4f}, "
            f"nis_gate={nis_gate_threshold:.3f}, "
            f"gyro_bias={gyro_bias:.4f} rad/s, "
            f"gyro_bias_mode={gyro_bias_mode}, "
            f"initial_gyro_bias={initial_gyro_bias:.4f} rad/s, "
            f"gyro_noise_std={gyro_noise:.4f} rad/s, seed={gyro_seed}, "
            f"wheel_slip_ratio={wheel_slip_ratio:.3f}."
            f" position_mode={position_mode}, "
            f"motion_prior_topic={motion_prior_topic}, "
            f"external_position_fusion={external_position_fusion}."
        )

    def report_status(self, status: str) -> None:
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def wheel_odom_callback(self, message: Odometry) -> None:
        """Update wheel yaw and position from one wheel-odometry message."""
        self.latest_wheel_odom = message
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        wheel_yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        # ``wheel_pose`` preserves the measured x/y increments.  ``propagated``
        # integrates body-forward speed using fused yaw, which exposes a
        # different position model and makes slip experiments explicit.
        if self.position_mode == "propagated":
            wheel_pose = (position.x, position.y, wheel_yaw)
        else:
            wheel_pose = self.wheel_model.apply(
                position.x,
                position.y,
                wheel_yaw,
            )
        if self.fusion_mode == "ekf":
            self.latest_fused_yaw = self.fusion.update_wheel(
                wheel_pose[2],
                x=position.x,
                y=position.y,
                linear_velocity_x=message.twist.twist.linear.x,
                stamp=stamp_seconds(message),
            )
            if self.motion_prior_fusion is not None:
                self.motion_prior_fusion.update_wheel(
                    wheel_pose[2],
                    x=position.x,
                    y=position.y,
                    linear_velocity_x=message.twist.twist.linear.x,
                    stamp=stamp_seconds(message),
                )
        else:
            self.latest_fused_yaw = self.fusion.update_wheel(wheel_pose[2])
        if self.fusion_mode == "ekf":
            # V4 always publishes the EKF state; position_mode remains a
            # diagnostic parameter for the earlier fixed/adaptive V3 modes.
            self.latest_wheel_pose = self.fusion.pose
        elif self.position_mode == "propagated":
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
        """Feed only IMU angular velocity z into the heading estimator."""
        # The orientation quaternion in sensor_msgs/Imu is deliberately
        # ignored; using it would make the simulated perfect orientation a
        # hidden ground-truth input.
        measured_rate = self.gyro_model.apply(message.angular_velocity.z)
        imu_stamp = stamp_seconds(message)
        fused_yaw = self.fusion.update_gyro(measured_rate, imu_stamp)
        if self.motion_prior_fusion is not None:
            self.motion_prior_fusion.update_gyro(measured_rate, imu_stamp)
        if fused_yaw is not None:
            self.latest_fused_yaw = fused_yaw
            if self.fusion_mode == "ekf":
                self.latest_wheel_pose = self.fusion.pose

    def external_position_callback(self, message: Odometry) -> None:
        """Fuse one accepted map-localizer event into the pose EKF.

        The MCL node publishes this topic once per accepted scan update.  It
        is intentionally not the repeated ``/localized_estimate`` fallback
        stream.  The position covariance is copied from the candidate message
        and is therefore part of the measurement model, not a hand-tuned
        correction factor.
        """
        if self.fusion_mode != "ekf":
            return
        covariance = message.pose.covariance
        if len(covariance) < 8:
            return
        accepted = self.fusion.update_external_position(
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            [
                [float(covariance[0]), float(covariance[1])],
                [float(covariance[6]), float(covariance[7])],
            ],
            measurement_stamp=stamp_seconds(message),
        )
        self.external_nis_publisher.publish(
            Float64(data=float(self.fusion.last_external_nis))
        )
        self.external_measurement_accepted_publisher.publish(
            Bool(data=bool(accepted))
        )
        self.external_measurement_age_publisher.publish(
            Float64(data=float(self.fusion.last_external_measurement_age_s))
        )
        self.external_measurement_replayed_publisher.publish(
            Bool(data=bool(self.fusion.last_external_measurement_replayed))
        )
        if accepted:
            # The external update changes the internal EKF state between IMU
            # callbacks.  Refresh the cached pose immediately so the next
            # output message exposes the corrected state.
            self.latest_fused_yaw = self.fusion.pose[2]
            self.latest_wheel_pose = self.fusion.pose

    def make_odometry_message(
        self,
        pose: tuple[float, float, float],
        covariance: list[float],
    ) -> Odometry:
        """Build an odometry message without changing its source semantics."""
        estimate = Odometry()
        estimate.header = self.latest_wheel_odom.header
        estimate.header.frame_id = self.latest_wheel_odom.header.frame_id or "odom"
        estimate.child_frame_id = self.latest_wheel_odom.child_frame_id or "base_link"
        estimate.pose.pose.position.x = pose[0]
        estimate.pose.pose.position.y = pose[1]
        estimate.pose.pose.position.z = self.latest_wheel_odom.pose.pose.position.z
        (
            estimate.pose.pose.orientation.x,
            estimate.pose.pose.orientation.y,
            estimate.pose.pose.orientation.z,
            estimate.pose.pose.orientation.w,
        ) = quaternion_from_yaw(wrap_angle(pose[2]))
        estimate.twist = self.latest_wheel_odom.twist
        estimate.pose.covariance = list(covariance)
        estimate.twist.covariance = self.latest_wheel_odom.twist.covariance
        return estimate

    def publish_estimate(self) -> None:
        """Publish the current estimated pose and fusion diagnostics."""
        if self.latest_wheel_odom is None or not self.fusion.ready:
            self.report_status("Waiting for wheel odometry and IMU samples.")
            return
        if self.latest_fused_yaw is None:
            return

        # Position comes from the selected wheel model; orientation comes from
        # the fixed/adaptive heading fusion state.  The message remains an
        # Odometry-shaped interface so the existing follower can consume it.
        if self.fusion_mode == "ekf":
            estimate_pose = self.fusion.pose
            estimate_covariance = self.fusion.pose_covariance_6x6
        else:
            estimate_pose = self.latest_wheel_pose
            estimate_covariance = self.latest_wheel_odom.pose.covariance
        estimate = self.make_odometry_message(estimate_pose, estimate_covariance)
        self.publisher.publish(estimate)
        if self.motion_prior_publisher is not None:
            if self.motion_prior_fusion is not None and self.motion_prior_fusion.ready:
                prior_pose = self.motion_prior_fusion.pose
                prior_covariance = self.motion_prior_fusion.pose_covariance_6x6
            else:
                # Fixed/adaptive V3 modes have no separate position EKF.  In
                # those modes the published motion prior is simply the same
                # wheel/IMU estimate, preserving the old behavior.
                prior_pose = estimate_pose
                prior_covariance = estimate_covariance
            self.motion_prior_publisher.publish(
                self.make_odometry_message(prior_pose, prior_covariance)
            )
        self.gain_publisher.publish(Float64(data=float(self.fusion.last_gain)))
        self.bias_publisher.publish(
            Float64(data=float(self.fusion.bias_estimate))
        )
        self.wheel_yaw_bias_publisher.publish(
            Float64(data=float(getattr(self.fusion, "wheel_yaw_bias_estimate", 0.0)))
        )
        self.innovation_publisher.publish(
            Float64(data=float(self.fusion.last_innovation))
        )
        self.nis_publisher.publish(
            Float64(data=float(getattr(self.fusion, "last_nis", 0.0)))
        )
        self.measurement_accepted_publisher.publish(
            Bool(
                data=bool(
                    getattr(self.fusion, "last_measurement_accepted", True)
                )
            )
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
