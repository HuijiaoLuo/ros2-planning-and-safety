"""Small covariance-aware EKF for wheel-speed, IMU, and external fusion.

The filter is intentionally limited to the quantities available in this
project.  It estimates ``[x, y, yaw, gyro_bias, wheel_yaw_bias]``.  Wheel
forward speed and the measured IMU yaw rate drive the prediction; wheel yaw is
the scalar
measurement update.  An optional external ``[x, y]`` measurement can also be
fused through its reported covariance; this is the minimal coupling point for
a map localizer.  Gazebo ground truth is never an input to this class.

This is a transparent V4 experiment rather than a replacement for a general
robot-localization package.  The explicit matrices make the assumptions and
the uncertainty propagation inspectable in tests and in the diagnostic CSV.
"""

from __future__ import annotations

import math
from typing import Any, Sequence

from robotics_nav.heading_fusion import wrap_angle


State = list[float]
Matrix = list[list[float]]
Event = tuple[float, int, str, tuple[Any, ...]]


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
    """Estimate planar pose and sensor biases with a five-state EKF.

    The state is ``[x, y, theta, b_g, b_w]``.  ``b_g`` is the gyro rate bias;
    ``b_w`` is a slowly varying wheel-yaw bias.  In ``estimated`` mode, the
    EKF may update ``b_g`` from wheel-yaw innovations.  In ``fixed`` mode,
    ``b_g`` is a pre-calibrated constant and wheel yaw cannot change it.  The
    process model is a
    unicycle approximation driven by body-forward wheel speed and
    bias-corrected gyro rate.  Wheel yaw is deliberately used only as a
    heading measurement; this avoids treating the integrated wheel pose as an
    independent absolute position sensor and makes wheel slip visible in the
    position covariance.
    """

    def __init__(
        self,
        *,
        gyro_rate_noise_std_rad_s: float = 0.01,
        wheel_yaw_noise_std_rad: float = 0.07,
        gyro_bias_random_walk_std_rad_s2: float = 0.001,
        wheel_speed_noise_std_m_s: float = 0.02,
        wheel_slip_ratio: float = 0.0,
        wheel_slip_noise_std: float = 0.0,
        initial_position_variance_m2: float = 0.25,
        initial_heading_variance_rad2: float = 0.25,
        initial_bias_variance_rad2_s2: float = 0.01,
        gyro_bias_mode: str = "estimated",
        initial_gyro_bias_rad_s: float = 0.0,
        initial_wheel_yaw_bias_variance_rad2: float = 0.01,
        wheel_yaw_bias_random_walk_std_rad_sqrt_s: float = 0.001,
        nis_gate_threshold: float = 9.0,
    ) -> None:
        self.gyro_rate_noise_std = max(0.0, float(gyro_rate_noise_std_rad_s))
        self.wheel_yaw_noise_variance = max(
            1.0e-12, float(wheel_yaw_noise_std_rad) ** 2
        )
        self.bias_random_walk_variance = max(
            0.0, float(gyro_bias_random_walk_std_rad_s2) ** 2
        )
        self.wheel_yaw_bias_random_walk_variance = max(
            0.0, float(wheel_yaw_bias_random_walk_std_rad_sqrt_s) ** 2
        )
        self.gyro_bias_mode = str(gyro_bias_mode).strip().lower()
        if self.gyro_bias_mode not in {"estimated", "fixed"}:
            raise ValueError("gyro_bias_mode must be 'estimated' or 'fixed'")
        self.initial_gyro_bias_rad_s = float(initial_gyro_bias_rad_s)
        self.wheel_speed_noise_variance = max(
            0.0, float(wheel_speed_noise_std_m_s) ** 2
        )
        self.translation_scale = 1.0 - max(
            0.0, min(0.99, float(wheel_slip_ratio))
        )
        # This is the standard deviation of the *unknown deviation* around
        # the configured slip ratio.  The mean model still uses
        # ``wheel_slip_ratio``; this separate term only tells the covariance
        # how uncertain that mean translation is during propagation.
        self.wheel_slip_noise_variance = max(
            0.0, float(wheel_slip_noise_std) ** 2
        )
        self.nis_gate_threshold = max(0.0, float(nis_gate_threshold))

        self.state: State = [0.0, 0.0, 0.0, self.initial_gyro_bias_rad_s, 0.0]
        self.covariance: Matrix = _identity(5)
        self.covariance[0][0] = max(1.0e-12, float(initial_position_variance_m2))
        self.covariance[1][1] = max(1.0e-12, float(initial_position_variance_m2))
        self.covariance[2][2] = max(1.0e-12, float(initial_heading_variance_rad2))
        self.covariance[3][3] = (
            max(1.0e-12, float(initial_bias_variance_rad2_s2))
            if self.gyro_bias_mode == "estimated"
            else 1.0e-12
        )
        self.covariance[4][4] = max(
            1.0e-12,
            float(initial_wheel_yaw_bias_variance_rad2),
        )

        self.initialized = False
        self.wheel_yaw: float | None = None
        self.latest_wheel_speed_m_s = 0.0
        self.last_imu_stamp: float | None = None
        self.last_gain = 0.0
        self.last_innovation = 0.0
        self.last_nis = 0.0
        self.last_measurement_accepted = True
        # External-position diagnostics are kept separate from the wheel-yaw
        # diagnostics above.  A caller can therefore distinguish a rejected
        # LiDAR/MCL update from a rejected wheel-yaw update without inferring
        # the source from a shared scalar NIS value.
        self.last_external_innovation: tuple[float, float] = (0.0, 0.0)
        self.last_external_nis = 0.0
        self.last_external_measurement_accepted = True

        # Sensor callbacks do not arrive in a guaranteed order.  Keep the
        # timestamped wheel/IMU event stream so a delayed map-position
        # observation can be inserted at the time at which it was measured
        # and the state can then be propagated forward again.  This is a
        # replay-based delayed-event mechanism, not a correction applied to the
        # newest state with a hand-tuned weight.
        self._event_history: list[Event] = []
        self._next_event_sequence = 0
        self._replaying_history = False
        self._latest_event_stamp: float | None = None
        self.last_external_measurement_stamp: float | None = None
        self.last_external_measurement_age_s = 0.0
        self.last_external_measurement_replayed = False
        self._initial_core_snapshot = self._core_snapshot()

    def _core_snapshot(self) -> tuple[Any, ...]:
        """Capture only dynamic filter state needed for deterministic replay."""
        return (
            list(self.state),
            [list(row) for row in self.covariance],
            self.initialized,
            self.wheel_yaw,
            self.latest_wheel_speed_m_s,
            self.last_imu_stamp,
        )

    def _restore_core_snapshot(self, snapshot: tuple[Any, ...]) -> None:
        """Restore the state before replaying timestamped sensor events."""
        (
            state,
            covariance,
            self.initialized,
            self.wheel_yaw,
            self.latest_wheel_speed_m_s,
            self.last_imu_stamp,
        ) = snapshot
        self.state = list(state)
        self.covariance = [list(row) for row in covariance]

    def _record_event(self, stamp: float, kind: str, *arguments: Any) -> None:
        """Append one sensor event while preserving its source timestamp."""
        if self._replaying_history:
            return
        timestamp = float(stamp)
        if not math.isfinite(timestamp):
            return
        self._event_history.append(
            (timestamp, self._next_event_sequence, kind, tuple(arguments))
        )
        self._next_event_sequence += 1
        self._latest_event_stamp = (
            timestamp
            if self._latest_event_stamp is None
            else max(self._latest_event_stamp, timestamp)
        )

    def _dispatch_event(self, event: Event) -> None:
        """Replay one recorded event without recording it a second time."""
        _stamp, _sequence, kind, arguments = event
        if kind == "wheel":
            self._update_wheel_impl(
                arguments[0],
                x=arguments[1],
                y=arguments[2],
                linear_velocity_x=arguments[3],
            )
        elif kind == "gyro":
            self._update_gyro_impl(*arguments)
        elif kind == "external":
            accepted = self._apply_external_position_impl(
                arguments[0],
                arguments[1],
                arguments[2],
                nis_gate_threshold=arguments[3],
            )
            result_holder = arguments[4]
            result_holder["accepted"] = bool(accepted)

    def _replay_event_history(self) -> None:
        """Recompute the current posterior after inserting a delayed event."""
        self._restore_core_snapshot(self._initial_core_snapshot)
        self._replaying_history = True
        try:
            for event in self._event_history:
                self._dispatch_event(event)
        finally:
            self._replaying_history = False

    def _enforce_fixed_gyro_bias(self) -> None:
        """Keep a calibrated gyro bias out of wheel-yaw state updates.

        In ``fixed`` mode the bias is a calibration constant, not an EKF
        state that can absorb wheel-yaw model error.  Clearing its covariance
        row and column also removes the cross-covariance route through which a
        wheel-yaw update could otherwise change the bias indirectly.
        """
        if self.gyro_bias_mode != "fixed":
            return
        self.state[3] = self.initial_gyro_bias_rad_s
        for index in range(5):
            self.covariance[3][index] = 0.0
            self.covariance[index][3] = 0.0
        self.covariance[3][3] = 1.0e-12

    @property
    def ready(self) -> bool:
        return self.initialized and self.wheel_yaw is not None

    @property
    def bias_estimate(self) -> float:
        return self.state[3]

    @property
    def wheel_yaw_bias_estimate(self) -> float:
        """Return the estimated slowly varying wheel-yaw bias in radians."""
        return self.state[4]

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
        stamp: float | None = None,
    ) -> float:
        """Consume a wheel-yaw sample and optionally record its timestamp."""
        result = self._update_wheel_impl(
            wheel_yaw,
            x=x,
            y=y,
            linear_velocity_x=linear_velocity_x,
        )
        if stamp is not None:
            self._record_event(
                stamp,
                "wheel",
                float(wheel_yaw),
                float(x),
                float(y),
                float(linear_velocity_x),
            )
        return result

    def _update_wheel_impl(
        self,
        wheel_yaw: float,
        *,
        x: float = 0.0,
        y: float = 0.0,
        linear_velocity_x: float = 0.0,
    ) -> float:
        """Initialize or update the filter with wheel yaw.

        The first wheel message establishes the local position and heading.
        Later wheel messages perform a one-dimensional Kalman update on yaw and
        wheel-yaw bias. The measurement model is
        ``wheel_yaw = theta + wheel_yaw_bias + white_noise``. This prevents
        repeated correlated wheel measurements from falsely driving the
        heading covariance toward zero.
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

        measurement_jacobian = [0.0, 0.0, 1.0, 0.0, 1.0]
        predicted_wheel_yaw = self.state[2] + self.state[4]
        innovation = wrap_angle(wheel_yaw - predicted_wheel_yaw)
        measurement_variance = self.wheel_yaw_noise_variance
        for row in range(5):
            for column in range(5):
                measurement_variance += (
                    measurement_jacobian[row]
                    * self.covariance[row][column]
                    * measurement_jacobian[column]
                )
        measurement_variance = max(1.0e-12, measurement_variance)
        nis = innovation * innovation / measurement_variance
        self.last_innovation = innovation
        self.last_nis = nis

        if self.nis_gate_threshold > 0.0 and nis > self.nis_gate_threshold:
            self.last_gain = 0.0
            self.last_measurement_accepted = False
            return self.state[2]

        gain = [
            sum(
                self.covariance[row][column] * measurement_jacobian[column]
                for column in range(5)
            )
            / measurement_variance
            for row in range(5)
        ]
        if self.gyro_bias_mode == "fixed":
            # H has no direct gyro-bias term, but P can create an indirect
            # gain.  A fixed calibrated bias must not be changed by wheel yaw.
            gain[3] = 0.0
        for row in range(5):
            self.state[row] += gain[row] * innovation
        self.state[2] = wrap_angle(self.state[2])

        # Joseph-form covariance update keeps the matrix positive-semidefinite
        # more reliably than subtracting KHP directly after repeated updates.
        identity = _identity(5)
        for row in range(5):
            for column in range(5):
                identity[row][column] -= gain[row] * measurement_jacobian[column]
        updated = _matmul(_matmul(identity, self.covariance), _transpose(identity))
        _add_in_place(updated, _outer(gain, self.wheel_yaw_noise_variance))
        self.covariance = _symmetrize(updated)
        self._enforce_fixed_gyro_bias()
        self.last_gain = gain[2]
        self.last_measurement_accepted = True
        return self.state[2]

    def update_gyro(self, angular_velocity_z: float, stamp: float) -> float | None:
        """Consume and record one timestamped IMU yaw-rate sample."""
        result = self._update_gyro_impl(angular_velocity_z, stamp)
        if result is not None:
            self._record_event(stamp, "gyro", float(angular_velocity_z), float(stamp))
        return result

    def _update_gyro_impl(
        self, angular_velocity_z: float, stamp: float
    ) -> float | None:
        """Predict pose/covariance from one IMU yaw-rate sample."""
        if not self.initialized:
            return None
        if self.last_imu_stamp is not None:
            delta_time = float(stamp) - self.last_imu_stamp
            if 0.0 < delta_time <= 1.0:
                old_x, old_y, old_heading, old_bias, _old_wheel_yaw_bias = self.state
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
                state_transition = _identity(5)
                state_transition[0][2] = -distance * math.sin(midpoint_heading)
                state_transition[1][2] = distance * math.cos(midpoint_heading)
                state_transition[0][3] = (
                    0.5 * distance * delta_time * math.sin(midpoint_heading)
                )
                state_transition[1][3] = (
                    -0.5 * distance * delta_time * math.cos(midpoint_heading)
                )
                state_transition[2][3] = -delta_time
                if self.gyro_bias_mode == "fixed":
                    state_transition[0][3] = 0.0
                    state_transition[1][3] = 0.0
                    state_transition[2][3] = 0.0

                # Q contains independent gyro-rate, wheel-speed, and bias
                # random-walk contributions.  The first two are expressed as
                # state sensitivities so x/y covariance grows with motion.
                gyro_sensitivity = [
                    -0.5 * distance * delta_time * math.sin(midpoint_heading),
                    0.5 * distance * delta_time * math.cos(midpoint_heading),
                    delta_time,
                    0.0,
                    0.0,
                ]
                speed_sensitivity = [
                    self.translation_scale * delta_time * math.cos(midpoint_heading),
                    self.translation_scale * delta_time * math.sin(midpoint_heading),
                    0.0,
                    0.0,
                    0.0,
                ]
                # If the actual slip fraction differs from the configured
                # value by δs, the travelled distance changes by -v*dt*δs.
                # This sensitivity maps slip-fraction uncertainty into x/y
                # position covariance without pretending that slip is directly
                # observable from the available yaw measurements.
                slip_sensitivity = [
                    -self.latest_wheel_speed_m_s
                    * delta_time
                    * math.cos(midpoint_heading),
                    -self.latest_wheel_speed_m_s
                    * delta_time
                    * math.sin(midpoint_heading),
                    0.0,
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
                _add_in_place(
                    process_noise,
                    _outer(slip_sensitivity, self.wheel_slip_noise_variance),
                )
                process_noise[3][3] += (
                    self.bias_random_walk_variance * delta_time
                    if self.gyro_bias_mode == "estimated"
                    else 0.0
                )
                process_noise[4][4] += (
                    self.wheel_yaw_bias_random_walk_variance * delta_time
                )
                self.covariance = _symmetrize(
                    _matmul(
                        _matmul(state_transition, self.covariance),
                        _transpose(state_transition),
                    )
                )
                _add_in_place(self.covariance, process_noise)
                self.covariance = _symmetrize(self.covariance)
                self._enforce_fixed_gyro_bias()
        self.last_imu_stamp = float(stamp)
        return self.state[2]

    def update_external_position(
        self,
        x: float,
        y: float,
        covariance_xy: Sequence[Sequence[float]],
        *,
        measurement_stamp: float | None = None,
        nis_gate_threshold: float | None = None,
    ) -> bool:
        """Fuse a map position at its measurement time, then re-propagate.

        A timestamped external observation is inserted after all sensor
        events with an earlier or equal timestamp.  The filter is replayed
        from its initial state, so the correction affects the historical
        state where it belongs and the subsequent IMU/wheel events carry it
        to the current control time.  Untimestamped calls remain available
        for unit tests and legacy callers and are applied to the current
        state explicitly.
        """
        if measurement_stamp is None:
            self.last_external_measurement_replayed = False
            return self._apply_external_position_impl(
                x,
                y,
                covariance_xy,
                nis_gate_threshold=nis_gate_threshold,
            )

        try:
            stamp = float(measurement_stamp)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(stamp) or self._latest_event_stamp is None:
            return False
        if stamp > self._latest_event_stamp + 1.0e-6:
            # A future observation cannot be fused before the corresponding
            # motion events exist; waiting for that state is safer than
            # pretending the measurement describes the current pose.
            self.last_external_measurement_replayed = False
            self.last_external_measurement_stamp = stamp
            self.last_external_measurement_age_s = 0.0
            return False

        self.last_external_measurement_stamp = stamp
        self.last_external_measurement_age_s = max(
            0.0, self._latest_event_stamp - stamp
        )
        result_holder: dict[str, bool] = {"accepted": False}
        covariance_copy = (
            (float(covariance_xy[0][0]), float(covariance_xy[0][1])),
            (float(covariance_xy[1][0]), float(covariance_xy[1][1])),
        )
        event: Event = (
            stamp,
            self._next_event_sequence,
            "external",
            (
                float(x),
                float(y),
                covariance_copy,
                nis_gate_threshold,
                result_holder,
            ),
        )
        self._next_event_sequence += 1
        insertion_index = len(self._event_history)
        while (
            insertion_index > 0
            and self._event_history[insertion_index - 1][0] > stamp
        ):
            insertion_index -= 1
        self._event_history.insert(insertion_index, event)
        self._replay_event_history()
        self.last_external_measurement_replayed = True
        return bool(result_holder["accepted"])

    def _apply_external_position_impl(
        self,
        x: float,
        y: float,
        covariance_xy: Sequence[Sequence[float]],
        *,
        nis_gate_threshold: float | None = None,
    ) -> bool:
        """Fuse an external world-frame position observation.

        ``covariance_xy`` is the 2x2 covariance of the supplied ``(x, y)``
        measurement, not a tuning weight.  The measurement model is

        ``z = [x, y] + v`` and ``H = [[1,0,0,0,0], [0,1,0,0,0]]``.

        The Kalman gain is nevertheless five-dimensional.  Therefore any
        position/heading/bias cross-covariance already present in ``P`` can
        update the corresponding state components.  This is the intended
        small coupled-Bayesian step: the map localizer supplies a position
        observation and its uncertainty, while the EKF decides how that
        evidence should affect the correlated state.

        The update is gated with the 2-D normalized innovation squared (NIS).
        A rejected observation leaves both state and covariance unchanged.
        Joseph-form covariance algebra is used so repeated asynchronous
        external corrections remain numerically symmetric and positive
        semidefinite more reliably than a direct ``P - KHP`` subtraction.
        """
        self.last_external_innovation = (0.0, 0.0)
        self.last_external_nis = 0.0
        self.last_external_measurement_accepted = False
        if not self.initialized:
            return False

        try:
            measurement_x = float(x)
            measurement_y = float(y)
            covariance_xx = float(covariance_xy[0][0])
            covariance_xy_value = 0.5 * (
                float(covariance_xy[0][1]) + float(covariance_xy[1][0])
            )
            covariance_yy = float(covariance_xy[1][1])
        except (IndexError, TypeError, ValueError):
            return False

        values = (
            measurement_x,
            measurement_y,
            covariance_xx,
            covariance_xy_value,
            covariance_yy,
        )
        if not all(math.isfinite(value) for value in values):
            return False

        # A covariance is allowed to be semidefinite, but the innovation
        # covariance S=P_xy+R must be invertible.  Tiny diagonal floors avoid
        # treating a numerically exact zero-variance report as an exception.
        measurement_noise = [
            [max(1.0e-12, covariance_xx), covariance_xy_value],
            [covariance_xy_value, max(1.0e-12, covariance_yy)],
        ]
        innovation = [
            measurement_x - self.state[0],
            measurement_y - self.state[1],
        ]
        self.last_external_innovation = (innovation[0], innovation[1])

        innovation_covariance = [
            [
                self.covariance[0][0] + measurement_noise[0][0],
                self.covariance[0][1] + measurement_noise[0][1],
            ],
            [
                self.covariance[1][0] + measurement_noise[1][0],
                self.covariance[1][1] + measurement_noise[1][1],
            ],
        ]
        s_xx = innovation_covariance[0][0]
        s_xy = 0.5 * (
            innovation_covariance[0][1] + innovation_covariance[1][0]
        )
        s_yy = innovation_covariance[1][1]
        determinant = s_xx * s_yy - s_xy * s_xy
        if not math.isfinite(determinant) or determinant <= 1.0e-15:
            return False

        inverse = [
            [s_yy / determinant, -s_xy / determinant],
            [-s_xy / determinant, s_xx / determinant],
        ]
        nis = (
            innovation[0]
            * (inverse[0][0] * innovation[0] + inverse[0][1] * innovation[1])
            + innovation[1]
            * (inverse[1][0] * innovation[0] + inverse[1][1] * innovation[1])
        )
        self.last_external_nis = max(0.0, float(nis))
        gate = (
            self.nis_gate_threshold
            if nis_gate_threshold is None
            else max(0.0, float(nis_gate_threshold))
        )
        if gate > 0.0 and self.last_external_nis > gate:
            return False

        # H selects state entries 0 and 1, so P H^T is simply the first two
        # covariance columns.  Keeping this explicit makes the cross-state
        # coupling visible instead of hiding it in a matrix package.
        kalman_gain: Matrix = [[0.0, 0.0] for _ in range(5)]
        for row in range(5):
            kalman_gain[row][0] = (
                self.covariance[row][0] * inverse[0][0]
                + self.covariance[row][1] * inverse[1][0]
            )
            kalman_gain[row][1] = (
                self.covariance[row][0] * inverse[0][1]
                + self.covariance[row][1] * inverse[1][1]
            )
        for row in range(5):
            self.state[row] += (
                kalman_gain[row][0] * innovation[0]
                + kalman_gain[row][1] * innovation[1]
            )
        self.state[2] = wrap_angle(self.state[2])

        measurement_jacobian = [
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0],
        ]
        identity = _identity(5)
        for row in range(5):
            for column in range(5):
                identity[row][column] -= (
                    kalman_gain[row][0] * measurement_jacobian[0][column]
                    + kalman_gain[row][1] * measurement_jacobian[1][column]
                )
        gain_transpose = _transpose(kalman_gain)
        updated = _matmul(
            _matmul(identity, self.covariance), _transpose(identity)
        )
        updated_noise = _matmul(
            _matmul(kalman_gain, measurement_noise), gain_transpose
        )
        _add_in_place(updated, updated_noise)
        self.covariance = _symmetrize(updated)
        self._enforce_fixed_gyro_bias()
        self.last_external_measurement_accepted = True
        return True
