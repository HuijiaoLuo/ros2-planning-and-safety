"""Small, dependency-free heading fusion primitive for the V3 estimator."""

from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def blend_angles(reference: float, measurement: float, measurement_weight: float) -> float:
    """Blend two angles along the shortest circular distance."""
    weight = max(0.0, min(1.0, measurement_weight))
    correction = wrap_angle(measurement - reference)
    return wrap_angle(reference + weight * correction)


class HeadingFusion:
    """Integrate gyro yaw and softly anchor it to wheel-odometry yaw.

    The gyro provides short-term angular motion while wheel odometry keeps the
    integrated heading from drifting indefinitely. This is deliberately a
    transparent first V3 estimator, not a replacement for a covariance-aware
    EKF.
    """

    def __init__(self, wheel_weight: float = 0.02) -> None:
        self.wheel_weight = max(0.0, min(1.0, wheel_weight))
        self.wheel_yaw: float | None = None
        self.imu_yaw: float | None = None
        self.fused_yaw: float | None = None
        self.last_imu_stamp: float | None = None

    @property
    def ready(self) -> bool:
        return self.wheel_yaw is not None and self.imu_yaw is not None

    def update_wheel(self, wheel_yaw: float) -> float:
        """Update the wheel reference and return the current fused yaw."""
        wheel_yaw = wrap_angle(wheel_yaw)
        self.wheel_yaw = wheel_yaw
        if self.imu_yaw is None:
            self.imu_yaw = wheel_yaw
        if self.fused_yaw is None:
            self.fused_yaw = wheel_yaw
        else:
            self.fused_yaw = blend_angles(
                self.fused_yaw,
                wheel_yaw,
                self.wheel_weight,
            )
        return self.fused_yaw

    def update_gyro(self, angular_velocity_z: float, stamp: float) -> float | None:
        """Integrate a gyro sample and return fused yaw when initialized."""
        if self.wheel_yaw is None:
            return None
        if self.imu_yaw is None:
            self.imu_yaw = self.wheel_yaw
        if self.last_imu_stamp is not None:
            dt = stamp - self.last_imu_stamp
            # Ignore duplicate, backwards, or implausibly old timestamps.
            if 0.0 < dt <= 1.0:
                self.imu_yaw = wrap_angle(
                    self.imu_yaw + angular_velocity_z * dt
                )
                if self.fused_yaw is None:
                    self.fused_yaw = self.imu_yaw
                else:
                    self.fused_yaw = wrap_angle(
                        self.fused_yaw + angular_velocity_z * dt
                    )
        self.last_imu_stamp = stamp
        if self.fused_yaw is None:
            self.fused_yaw = self.wheel_yaw
        return self.fused_yaw
