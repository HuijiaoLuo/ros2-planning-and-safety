# Covariance-aware Pose Estimator

This document describes the first full-pose EKF experiment. It is separate
from the transparent V3 estimator in [`STATE_ESTIMATION.md`](STATE_ESTIMATION.md)
and remains diagnostic-only until its covariance and closed-loop behavior are
validated under controlled uncertainty.

## State and prediction model

The filter state is

$$
\mathbf{x}_k=
\begin{bmatrix}
x_k\\y_k\\\theta_k\\b_{g,k}
\end{bmatrix},
$$

where $b_{g,k}$ is the estimated gyro bias. The IMU supplies angular rate and
wheel odometry supplies body-forward speed. The bias-corrected rate is

$$
\omega_k=\omega_{z,k}-b_{g,k}.
$$

The translational increment uses the configured wheel-slip ratio $s$:

$$
\Delta s_k=(1-s)v_{x,k}\Delta t_k.
$$

The filter uses the midpoint of consecutive headings:

$$
\theta_{\mathrm{mid},k}
=\operatorname{wrap}\left(
\theta_{k-1}
+\frac{1}{2}\operatorname{wrap}(\theta_k-\theta_{k-1})
\right).
$$

The unicycle prediction is therefore

$$
x_k^-=x_{k-1}+\Delta s_k\cos(\theta_{\mathrm{mid},k}),
$$

$$
y_k^-=y_{k-1}+\Delta s_k\sin(\theta_{\mathrm{mid},k}),
$$

$$
\theta_k^-=\operatorname{wrap}(\theta_{k-1}+\omega_k\Delta t_k).
$$

The covariance is propagated with the linearized model:

$$
P_k^-=F_kP_{k-1}F_k^{\mathsf T}+Q_k.
$$

$Q_k$ contains configured gyro-rate noise, wheel-speed noise, and gyro-bias
random-walk noise. The slip ratio is configured in this first experiment; it
is not estimated as an additional state.

## Wheel-yaw measurement and NIS gate

Wheel yaw is used as a scalar heading measurement. The wrapped innovation is

$$
\nu_k=\operatorname{wrap}\left(\theta_k^{\mathrm{wheel}}-\theta_k^-\right).
$$

With measurement matrix $H=[0\;0\;1\;0]$, the innovation variance and gain are

$$
S_k=HP_k^-H^{\mathsf T}+R_{\mathrm{wheel}},
$$

$$
K_k=P_k^-H^{\mathsf T}S_k^{-1}.
$$

The state correction is

$$
\mathbf{x}_k=\mathbf{x}_k^-+K_k\nu_k,
$$

with yaw wrapped after the update. The implementation uses a Joseph-form
covariance update to preserve symmetry and non-negative diagonal terms.

The normalized innovation squared is

$$
\operatorname{NIS}_k=\frac{\nu_k^2}{S_k}.
$$

If NIS exceeds `nis_gate_threshold` (default `9.0`), that wheel-yaw update is
rejected. This is a one-dimensional approximately three-sigma consistency
gate, not a claim that the complete simulated system is statistically
calibrated.

## ROS2 interface

With `fusion_mode:=ekf`, `heading_estimator` publishes the EKF pose through the
existing `/state_estimate` topic. It also publishes:

- `/heading_fusion_nis` — current normalized innovation squared;
- `/heading_measurement_accepted` — whether the latest wheel-yaw update passed
  the gate;
- `Odometry.pose.covariance` — the planar $x$, $y$, and yaw covariance entries.

The estimator consumes only `/wheel_odom` and `/imu/angular_velocity.z`.
Gazebo `/odom` is evaluation-only. The estimation logger records NIS, rejected
measurement episodes, covariance means/final values, and the usual RMSE and
bias metrics.

## First smoke result

The first smoke test kept navigation on `/odom`, so it tested the estimator
without allowing an unvalidated pose to control the robot. The recorded row
reported:

| Quantity | Result |
| --- | ---: |
| Navigation success | `True` |
| EKF position RMSE | `0.0735 m` |
| EKF heading RMSE | `0.0710 rad` |
| Mean NIS | `0.0785` |
| Maximum NIS | `1.2751` |
| Wheel-yaw rejection episodes | `0` |
| Mean x/y covariance | approximately `0.250 m²` |
| Mean yaw covariance | approximately `3.25e-4 rad²` |

The low NIS and zero rejections show that the nominal run did not trigger the
outlier gate. The relatively large x/y covariance also shows that this is an
interface and consistency smoke test, not evidence that the covariance is
already calibrated. The EKF heading RMSE was not lower than the wheel-yaw
reference in this row, so the filter should not yet be advertised as an
accuracy improvement.

## Reproducible smoke test

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  fusion_mode:=ekf \
  position_mode:=propagated \
  wheel_slip_ratio:=0.0 \
  imu_gyro_bias_rad_s:=0.0 \
  imu_gyro_noise_std_rad_s:=0.0 \
  wheel_speed_noise_std_m_s:=0.02 \
  nis_gate_threshold:=9.0 \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v4_ekf_smoke_eval.csv \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v4_ekf_smoke_metrics.csv
```

The next controlled comparisons are zero-slip versus
`wheel_slip_ratio:=0.10`, followed by seeded gyro bias and gyro white noise.
Only after those results are understood should
`navigation_pose_topic:=/state_estimate` be tested in closed loop.
