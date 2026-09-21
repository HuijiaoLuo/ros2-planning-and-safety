"""Small, dependency-free heading fusion primitive for the V3 estimator."""

from __future__ import annotations

import math
import random


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def blend_angles(reference: float, measurement: float, measurement_weight: float) -> float:
    """Blend two angles along the shortest circular distance."""
    weight = max(0.0, min(1.0, measurement_weight))
    correction = wrap_angle(measurement - reference)
    return wrap_angle(reference + weight * correction)


class GyroMeasurementModel:
    """Apply reproducible bias and white noise to a gyro-rate measurement."""

    def __init__(
        self,
        bias_rad_s: float = 0.0,
        noise_std_rad_s: float = 0.0,
        seed: int = 0,
    ) -> None:
        self.bias_rad_s = float(bias_rad_s)
        self.noise_std_rad_s = max(0.0, float(noise_std_rad_s))
        self.random = random.Random(int(seed))

    def apply(self, angular_velocity_z: float) -> float:
        # The seeded generator makes every uncertainty sweep reproducible:
        # changing the seed changes the realization, not the noise model.
        noise = (
            0.0
            if self.noise_std_rad_s == 0.0
            else self.random.gauss(0.0, self.noise_std_rad_s)
        )
        return float(angular_velocity_z) + self.bias_rad_s + noise


class WheelSlipMeasurementModel:
    """Apply deterministic longitudinal slip to wheel-odometry increments.

    A slip ratio ``s`` reports only ``(1 - s)`` of each translational
    increment. This is an estimator-side measurement perturbation, not a
    Gazebo tire-friction or contact model. Wheel yaw is preserved so the
    first experiment isolates translational localization error.
    """

    def __init__(self, slip_ratio: float = 0.0) -> None:
        self.slip_ratio = max(0.0, min(0.99, float(slip_ratio)))
        self.translation_scale = 1.0 - self.slip_ratio
        self._last_raw: tuple[float, float, float] | None = None
        self._estimate: tuple[float, float, float] | None = None

    def apply(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        """Return the perturbed pose for the next raw wheel-odometry pose."""
        raw = (float(x), float(y), wrap_angle(float(yaw)))
        if self._last_raw is None or self._estimate is None:
            # The first sample defines the estimator's initial frame.  No
            # artificial slip is applied to the initial absolute pose.
            self._last_raw = raw
            self._estimate = raw
            return self._estimate

        last_x, last_y, last_yaw = self._last_raw
        estimate_x, estimate_y, estimate_yaw = self._estimate
        # Slip is applied to translation increments only.  The yaw increment
        # remains available so this experiment isolates position drift from
        # heading drift.
        self._estimate = (
            estimate_x + self.translation_scale * (raw[0] - last_x),
            estimate_y + self.translation_scale * (raw[1] - last_y),
            wrap_angle(estimate_yaw + wrap_angle(raw[2] - last_yaw)),
        )
        self._last_raw = raw
        return self._estimate


class DifferentialDrivePoseModel:
    """Propagate position from body speed and an externally estimated yaw.

    The wheel-odometry linear velocity is treated as a body-frame forward
    measurement. Translation is scaled by the configured slip ratio, while
    the heading used for integration comes from the heading fusion state.
    This keeps the position model independent of Gazebo's ideal pose.
    """

    def __init__(self, slip_ratio: float = 0.0) -> None:
        self.slip_ratio = max(0.0, min(0.99, float(slip_ratio)))
        self.translation_scale = 1.0 - self.slip_ratio
        self._initialized = False
        self._x = 0.0
        self._y = 0.0
        self._last_stamp: float | None = None
        self._last_heading: float | None = None

    def apply(
        self,
        initial_x: float,
        initial_y: float,
        stamp: float,
        linear_velocity_x: float,
        fused_yaw: float,
    ) -> tuple[float, float, float]:
        """Integrate one wheel-odometry body-speed sample."""
        fused_yaw = wrap_angle(fused_yaw)
        if not self._initialized:
            self._initialized = True
            self._x = float(initial_x)
            self._y = float(initial_y)
            self._last_stamp = float(stamp)
            self._last_heading = fused_yaw
            return self._x, self._y, fused_yaw

        previous_stamp = self._last_stamp
        previous_heading = self._last_heading
        self._last_stamp = float(stamp)
        self._last_heading = fused_yaw
        if previous_stamp is None or previous_heading is None:
            return self._x, self._y, fused_yaw

        delta_time = float(stamp) - previous_stamp
        if not 0.0 < delta_time <= 1.0:
            return self._x, self._y, fused_yaw

        # Midpoint integration reduces the first-order error caused by using
        # either the old or new heading for the complete time interval.
        delta_heading = wrap_angle(fused_yaw - previous_heading)
        midpoint_heading = wrap_angle(previous_heading + 0.5 * delta_heading)
        distance = (
            float(linear_velocity_x)
            * self.translation_scale
            * delta_time
        )
        self._x += distance * math.cos(midpoint_heading)
        self._y += distance * math.sin(midpoint_heading)
        return self._x, self._y, fused_yaw


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
        self.last_gain = self.wheel_weight
        self.last_innovation = 0.0
        self.bias_estimate = 0.0

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
            # Wheel yaw is a low-rate reference.  Fixed fusion deliberately
            # uses a transparent blend instead of pretending to be an EKF.
            self.last_innovation = wrap_angle(wheel_yaw - self.fused_yaw)
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
                # Integrate only the IMU angular velocity.  The IMU orientation
                # field is intentionally not consumed as a hidden oracle.
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


class AdaptiveHeadingFusion:
    """Two-state heading filter with an estimated gyro bias.

    The state is ``[heading, gyro_bias]``. IMU yaw rate drives the prediction;
    wheel yaw is a scalar heading measurement. The wheel correction gain is
    computed from the predicted heading variance and wheel measurement
    variance instead of being a fixed tuning weight.
    """

    def __init__(
        self,
        gyro_rate_noise_std_rad_s: float = 0.01,
        wheel_yaw_noise_std_rad: float = 0.07,
        gyro_bias_random_walk_std_rad_s2: float = 0.001,
        initial_heading_variance_rad2: float = 0.25,
        initial_bias_variance_rad2_s2: float = 0.01,
        adaptive_wheel_noise: bool = False,
        wheel_yaw_noise_min_std_rad: float = 0.02,
        wheel_yaw_noise_max_std_rad: float = 0.20,
        wheel_noise_adaptation_rate: float = 0.05,
    ) -> None:
        self.gyro_rate_noise_std = max(0.0, float(gyro_rate_noise_std_rad_s))
        self.adaptive_wheel_noise = bool(adaptive_wheel_noise)
        min_std = max(1.0e-6, float(wheel_yaw_noise_min_std_rad))
        max_std = max(min_std, float(wheel_yaw_noise_max_std_rad))
        self.wheel_yaw_noise_min_variance = min_std**2
        self.wheel_yaw_noise_max_variance = max_std**2
        self.wheel_yaw_variance = max(
            self.wheel_yaw_noise_min_variance,
            min(self.wheel_yaw_noise_max_variance, float(wheel_yaw_noise_std_rad) ** 2),
        )
        self.wheel_noise_adaptation_rate = max(
            0.0, min(1.0, float(wheel_noise_adaptation_rate))
        )
        self.bias_random_walk_variance = max(
            0.0, float(gyro_bias_random_walk_std_rad_s2) ** 2
        )
        self.heading_variance = max(1.0e-12, float(initial_heading_variance_rad2))
        self.bias_variance = max(1.0e-12, float(initial_bias_variance_rad2_s2))
        self.cross_variance = 0.0
        self.heading: float | None = None
        self.bias = 0.0
        self.last_imu_stamp: float | None = None
        self.last_gain = 0.0
        self.last_innovation = 0.0
        self.wheel_yaw: float | None = None
        # This is the covariance of the predicted-vs-wheel innovation.  It is
        # not a direct sensor-noise measurement; it is a bounded heuristic used
        # to adapt the wheel-yaw variance when innovations become persistent.
        self.innovation_variance_estimate = (
            self.heading_variance + self.wheel_yaw_variance
        )

    @property
    def ready(self) -> bool:
        return self.heading is not None and self.wheel_yaw is not None

    @property
    def bias_estimate(self) -> float:
        return self.bias

    @property
    def wheel_yaw_noise_std_estimate(self) -> float:
        return math.sqrt(self.wheel_yaw_variance)

    def update_wheel(self, wheel_yaw: float) -> float:
        """Apply a wheel-yaw measurement and return the corrected heading."""
        wheel_yaw = wrap_angle(wheel_yaw)
        self.wheel_yaw = wheel_yaw
        if self.heading is None:
            self.heading = wheel_yaw
            self.last_gain = 0.0
            self.last_innovation = 0.0
            return self.heading

        innovation = wrap_angle(wheel_yaw - self.heading)
        if self.adaptive_wheel_noise:
            # Estimate recent innovation energy, subtract the predicted heading
            # variance, and clip the inferred wheel variance to configured
            # physical bounds.  This prevents one bad sample from making the
            # filter permanently ignore wheel yaw.
            rate = self.wheel_noise_adaptation_rate
            self.innovation_variance_estimate = (
                (1.0 - rate) * self.innovation_variance_estimate
                + rate * innovation * innovation
            )
            inferred_measurement_variance = max(
                self.wheel_yaw_noise_min_variance,
                self.innovation_variance_estimate - self.heading_variance,
            )
            self.wheel_yaw_variance = max(
                self.wheel_yaw_noise_min_variance,
                min(
                    self.wheel_yaw_noise_max_variance,
                    (1.0 - rate) * self.wheel_yaw_variance
                    + rate * inferred_measurement_variance,
                ),
            )
        # Scalar Kalman-style update for the heading component.  The cross
        # covariance couples the heading correction to the gyro-bias estimate.
        innovation_variance = self.heading_variance + self.wheel_yaw_variance
        gain_heading = self.heading_variance / innovation_variance
        gain_bias = self.cross_variance / innovation_variance

        old_heading_variance = self.heading_variance
        old_cross_variance = self.cross_variance
        self.heading = wrap_angle(self.heading + gain_heading * innovation)
        self.bias += gain_bias * innovation
        self.heading_variance = max(
            1.0e-12, (1.0 - gain_heading) * old_heading_variance
        )
        self.cross_variance = (1.0 - gain_heading) * old_cross_variance
        self.bias_variance = max(
            1.0e-12,
            self.bias_variance
            - gain_bias * old_cross_variance,
        )
        self.last_gain = gain_heading
        self.last_innovation = innovation
        return self.heading

    def update_gyro(self, angular_velocity_z: float, stamp: float) -> float | None:
        """Predict heading and covariance from one IMU yaw-rate sample."""
        if self.wheel_yaw is None:
            return None
        if self.heading is None:
            self.heading = self.wheel_yaw
        if self.last_imu_stamp is not None:
            dt = stamp - self.last_imu_stamp
            if 0.0 < dt <= 1.0:
                # Prediction: gyro rate advances heading, while the current
                # bias estimate is subtracted from the measured rate.
                self.heading = wrap_angle(
                    self.heading + (angular_velocity_z - self.bias) * dt
                )
                old_heading_variance = self.heading_variance
                old_cross_variance = self.cross_variance
                # Propagate the 2x2 covariance terms for [heading, bias].
                # Rate noise grows heading uncertainty with dt^2; bias random
                # walk grows the bias variance over time.
                self.heading_variance = max(
                    1.0e-12,
                    old_heading_variance
                    - 2.0 * dt * old_cross_variance
                    + dt * dt * self.bias_variance
                    + self.gyro_rate_noise_std**2 * dt * dt,
                )
                self.cross_variance = old_cross_variance - dt * self.bias_variance
                self.bias_variance += self.bias_random_walk_variance * dt
        self.last_imu_stamp = stamp
        return self.heading
