"""Small covariance-aware EKF for wheel-speed and IMU heading fusion.

The filter is intentionally limited to the quantities available in this
project.  It estimates ``[x, y, yaw, gyro_bias]``.  Wheel forward speed and
the measured IMU yaw rate drive the prediction; wheel yaw is the scalar
measurement update.  Gazebo ground truth is never an input to this class.

This is a transparent V4 experiment rather than a replacement for a general
robot-localization package.  The explicit matrices make the assumptions and
the uncertainty propagation inspectable in tests and in the diagnostic CSV.
"""

from __future__ import annotations

import math
from typing import Sequence

from robotics_nav.heading_fusion import wrap_angle


State = list[float]
Matrix = list[list[float]]


def _identity(size: int) -> Matrix:
    return [
        [1.0 if row == column else 0.0 for column in range(size)]
        for row in range(size)
    ]


def _transpose(matrix: Matrix) -> Matrix:
    return [list(column) for column in zip(*matrix)]


def _matmul(left: Matrix, right: Matrix) -> Matrix:
    return [
        [
            sum(left[row][inner] * right[inner][column] for inner in range(len(right)))
            for column in range(len(right[0]))
        ]
        for row in range(len(left))
    ]


def _outer(vector: Sequence[float], scale: float) -> Matrix:
    return [
        [scale * vector[row] * vector[column] for column in range(len(vector))]
        for row in range(len(vector))
    ]


def _add_in_place(target: Matrix, source: Matrix) -> None:
    for row in range(len(target)):
        for column in range(len(target[row])):
            target[row][column] += source[row][column]


def _symmetrize(matrix: Matrix) -> Matrix:
    return [
        [0.5 * (matrix[row][column] + matrix[column][row]) for column in range(len(matrix))]
        for row in range(len(matrix))
    ]


class PoseEKF:
    """Estimate planar pose and gyro bias with a four-state EKF.

    The state is ``[x, y, theta, b_g]``.  The process model is a unicycle
    approximation driven by body-forward wheel speed and bias-corrected gyro
    rate.  Wheel yaw is deliberately used only as a heading measurement; this
    avoids treating the integrated wheel pose as an independent absolute
    position sensor and makes wheel slip visible in the position covariance.
    """

    def __init__(
        self,
        *,
        gyro_rate_noise_std_rad_s: float = 0.01,
        wheel_yaw_noise_std_rad: float = 0.07,
        gyro_bias_random_walk_std_rad_s2: float = 0.001,
        wheel_speed_noise_std_m_s: float = 0.02,
        wheel_slip_ratio: float = 0.0,
        initial_position_variance_m2: float = 0.25,
        initial_heading_variance_rad2: float = 0.25,
        initial_bias_variance_rad2_s2: float = 0.01,
        nis_gate_threshold: float = 9.0,
    ) -> None:
        self.gyro_rate_noise_std = max(0.0, float(gyro_rate_noise_std_rad_s))
        self.wheel_yaw_noise_variance = max(
            1.0e-12, float(wheel_yaw_noise_std_rad) ** 2
        )
        self.bias_random_walk_variance = max(
            0.0, float(gyro_bias_random_walk_std_rad_s2) ** 2
        )
        self.wheel_speed_noise_variance = max(
            0.0, float(wheel_speed_noise_std_m_s) ** 2
        )
        self.translation_scale = 1.0 - max(
            0.0, min(0.99, float(wheel_slip_ratio))
        )
        self.nis_gate_threshold = max(0.0, float(nis_gate_threshold))

        self.state: State = [0.0, 0.0, 0.0, 0.0]
        self.covariance: Matrix = _identity(4)
        self.covariance[0][0] = max(1.0e-12, float(initial_position_variance_m2))
        self.covariance[1][1] = max(1.0e-12, float(initial_position_variance_m2))
        self.covariance[2][2] = max(1.0e-12, float(initial_heading_variance_rad2))
        self.covariance[3][3] = max(1.0e-12, float(initial_bias_variance_rad2_s2))

        self.initialized = False
        self.wheel_yaw: float | None = None
        self.latest_wheel_speed_m_s = 0.0
        self.last_imu_stamp: float | None = None
        self.last_gain = 0.0
        self.last_innovation = 0.0
        self.last_nis = 0.0
        self.last_measurement_accepted = True

    @property
    def ready(self) -> bool:
        return self.initialized and self.wheel_yaw is not None

    @property
    def bias_estimate(self) -> float:
        return self.state[3]

    @property
    def wheel_yaw_noise_std_estimate(self) -> float:
        return math.sqrt(self.wheel_yaw_noise_variance)

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.state[0], self.state[1], wrap_angle(self.state[2])

    @property
    def pose_covariance_6x6(self) -> list[float]:
        """Map the planar covariance into ``nav_msgs/Odometry`` ordering.

        ROS stores pose covariance as ``x, y, z, roll, pitch, yaw``.  The EKF
        has no z/roll/pitch states, so those entries remain zero while the
        x/y/yaw variances and cross-covariances are exposed to downstream
        tools.
        """
        result = [0.0] * 36
        mapping = {0: 0, 1: 1, 2: 5}
        for state_row, ros_row in mapping.items():
            for state_column, ros_column in mapping.items():
                result[6 * ros_row + ros_column] = self.covariance[state_row][state_column]
        return result

    def update_wheel(
        self,
        wheel_yaw: float,
        *,
        x: float = 0.0,
        y: float = 0.0,
        linear_velocity_x: float = 0.0,
    ) -> float:
        """Initialize or update the filter with wheel yaw.

        The first wheel message establishes the local position and heading.
        Later wheel messages perform a one-dimensional Kalman update on yaw.
        A normalized innovation squared (NIS) gate rejects implausible yaw
        measurements without changing the predicted state or covariance.
        """
        wheel_yaw = wrap_angle(wheel_yaw)
        self.wheel_yaw = wheel_yaw
        self.latest_wheel_speed_m_s = float(linear_velocity_x)
        if not self.initialized:
            self.state[0] = float(x)
            self.state[1] = float(y)
            self.state[2] = wheel_yaw
            self.initialized = True
            self.last_gain = 0.0
            self.last_innovation = 0.0
            self.last_nis = 0.0
            self.last_measurement_accepted = True
            return self.state[2]

        innovation = wrap_angle(wheel_yaw - self.state[2])
        measurement_variance = self.covariance[2][2] + self.wheel_yaw_noise_variance
        measurement_variance = max(1.0e-12, measurement_variance)
        nis = innovation * innovation / measurement_variance
        self.last_innovation = innovation
        self.last_nis = nis

        if self.nis_gate_threshold > 0.0 and nis > self.nis_gate_threshold:
            self.last_gain = 0.0
            self.last_measurement_accepted = False
            return self.state[2]

        gain = [self.covariance[row][2] / measurement_variance for row in range(4)]
        for row in range(4):
            self.state[row] += gain[row] * innovation
        self.state[2] = wrap_angle(self.state[2])

        # Joseph-form covariance update keeps the matrix positive-semidefinite
        # more reliably than subtracting KHP directly after repeated updates.
        identity = _identity(4)
        for row in range(4):
            for column in range(4):
                identity[row][column] -= gain[row] * (1.0 if column == 2 else 0.0)
        updated = _matmul(_matmul(identity, self.covariance), _transpose(identity))
        _add_in_place(updated, _outer(gain, self.wheel_yaw_noise_variance))
        self.covariance = _symmetrize(updated)
        self.last_gain = gain[2]
        self.last_measurement_accepted = True
        return self.state[2]

    def update_gyro(self, angular_velocity_z: float, stamp: float) -> float | None:
        """Predict pose/covariance from one IMU yaw-rate sample."""
        if not self.initialized:
            return None
        if self.last_imu_stamp is not None:
            delta_time = float(stamp) - self.last_imu_stamp
            if 0.0 < delta_time <= 1.0:
                old_x, old_y, old_heading, old_bias = self.state
                corrected_rate = float(angular_velocity_z) - old_bias
                distance = (
                    self.latest_wheel_speed_m_s
                    * self.translation_scale
                    * delta_time
                )
                midpoint_heading = wrap_angle(
                    old_heading + 0.5 * corrected_rate * delta_time
                )
                self.state[0] = old_x + distance * math.cos(midpoint_heading)
                self.state[1] = old_y + distance * math.sin(midpoint_heading)
                self.state[2] = wrap_angle(
                    old_heading + corrected_rate * delta_time
                )

                # F is the Jacobian of the midpoint unicycle model.  The
                # bias derivatives show how gyro bias eventually bends x/y.
                state_transition = _identity(4)
                state_transition[0][2] = -distance * math.sin(midpoint_heading)
                state_transition[1][2] = distance * math.cos(midpoint_heading)
                state_transition[0][3] = (
                    -0.5 * distance * delta_time * math.sin(midpoint_heading)
                )
                state_transition[1][3] = (
                    -0.5 * distance * delta_time * math.cos(midpoint_heading)
                )
                state_transition[2][3] = -delta_time

                # Q contains independent gyro-rate, wheel-speed, and bias
                # random-walk contributions.  The first two are expressed as
                # state sensitivities so x/y covariance grows with motion.
                gyro_sensitivity = [
                    -0.5 * distance * delta_time * math.sin(midpoint_heading),
                    0.5 * distance * delta_time * math.cos(midpoint_heading),
                    delta_time,
                    0.0,
                ]
                speed_sensitivity = [
                    self.translation_scale * delta_time * math.cos(midpoint_heading),
                    self.translation_scale * delta_time * math.sin(midpoint_heading),
                    0.0,
                    0.0,
                ]
                process_noise = _outer(
                    gyro_sensitivity, self.gyro_rate_noise_std**2
                )
                _add_in_place(
                    process_noise,
                    _outer(speed_sensitivity, self.wheel_speed_noise_variance),
                )
                process_noise[3][3] += (
                    self.bias_random_walk_variance * delta_time
                )
                self.covariance = _symmetrize(
                    _matmul(
                        _matmul(state_transition, self.covariance),
                        _transpose(state_transition),
                    )
                )
                _add_in_place(self.covariance, process_noise)
                self.covariance = _symmetrize(self.covariance)
        self.last_imu_stamp = float(stamp)
        return self.state[2]
