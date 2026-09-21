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

The symbols have the following physical meanings:

| Symbol | Physical quantity | Unit | Source or role |
| --- | --- | --- | --- |
| $x_k,y_k$ | estimated robot position in the local planar frame | m | EKF state |
| $\theta_k$ | estimated robot heading about the vertical axis | rad | EKF state |
| $b_{g,k}$ | slowly varying gyro zero-rate bias | rad/s | EKF state |
| $\omega_{z,k}$ | measured vertical-axis angular velocity | rad/s | `/imu.angular_velocity.z` |
| $v_{x,k}$ | measured forward body velocity | m/s | `/wheel_odom.twist.twist.linear.x` |
| $s$ | configured fractional wheel-slip loss | dimensionless | launch parameter |
| $\Delta t_k$ | time between consecutive IMU samples | s | message timestamps |
| $P_k$ | covariance of $[x,y,\theta,b_g]$ | mixed squared units | EKF uncertainty |
| $Q_k$ | uncertainty added by one motion step | mixed squared units | configured process noises |
| $R_{\mathrm{wheel}}$ | variance of one wheel-yaw measurement | rad$^2$ | wheel-yaw noise parameter |

The state is therefore not a perfect pose copied from Gazebo. It is the EKF's
best current estimate, and $P_k$ describes how uncertain that estimate is.

The translational increment uses the configured wheel-slip ratio $s$:

$$
\Delta s_k=(1-s)v_{x,k}\Delta t_k.
$$

The filter uses the midpoint of consecutive headings:

$$
\theta_{\mathrm{mid},k}
=\mathrm{wrap}\left(
\theta_{k-1}
+\frac{1}{2}\mathrm{wrap}(\theta_k-\theta_{k-1})
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
\theta_k^-=\mathrm{wrap}(\theta_{k-1}+\omega_k\Delta t_k).
$$

The covariance is propagated with the linearized model:

$$
P_k^-=F_kP_{k-1}F_k^{\mathsf T}+Q_k.
$$

$Q_k$ contains configured gyro-rate noise, wheel-speed noise, and gyro-bias
random-walk noise. The slip ratio is configured in this first experiment; it
is not estimated as an additional state.

## How the EKF is calculated

The filter repeats the following cycle whenever new sensor data arrive. The
important distinction is that the state is a probability estimate with a
covariance, not just a pose value.

### 1. Read the measurements

The wheel-odometry message supplies the body-forward velocity
$v_{x,k}$ and wheel yaw $\theta_k^{\mathrm{wheel}}$. The IMU supplies only
the measured yaw rate $\omega_{z,k}$. The IMU orientation quaternion is not
used. Gazebo `/odom` is never passed into the EKF.

The first wheel message initializes $x$, $y$, and $\theta$ locally. After that,
each IMU message predicts motion, while each later wheel message provides a
heading measurement update.

### 2. Predict the pose mean

The current bias estimate is subtracted from the gyro rate. The filter then
integrates forward motion over $\Delta t_k$. With

$$
\Delta s_k=(1-s)v_{x,k}\Delta t_k,
$$

the predicted pose is

$$
\begin{bmatrix}
x_k^-\\
y_k^-\\
\theta_k^-
\end{bmatrix}
=
\begin{bmatrix}
x_{k-1}+\Delta s_k\cos(\theta_{\mathrm{mid},k})\\
y_{k-1}+\Delta s_k\sin(\theta_{\mathrm{mid},k})\\
\mathrm{wrap}(\theta_{k-1}+\omega_k\Delta t_k)
\end{bmatrix}.
$$

The superscript $-$ means “before using the new wheel-yaw measurement”.

### 3. Predict the covariance

The covariance $P$ describes uncertainty in $x$, $y$, yaw, and gyro bias. The
EKF linearizes the motion model around the current estimate. In this
implementation the state-transition Jacobian is approximately

$$
F_k=\begin{bmatrix}
1 & 0 & -\Delta s_k\sin(\theta_{\mathrm{mid},k}) &
-\frac{1}{2}\Delta s_k\Delta t_k\sin(\theta_{\mathrm{mid},k})\\
0 & 1 & \Delta s_k\cos(\theta_{\mathrm{mid},k}) &
-\frac{1}{2}\Delta s_k\Delta t_k\cos(\theta_{\mathrm{mid},k})\\
0 & 0 & 1 & -\Delta t_k\\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

The first two rows show how heading uncertainty bends the predicted position.
The fourth column shows how gyro-bias uncertainty accumulates into heading and
then into position. The covariance prediction is

$$
P_k^-=F_kP_{k-1}F_k^{\mathsf T}+Q_k.
$$

The superscript $-$ means the predicted, pre-measurement quantity. Without the
superscript, $P_k$ denotes the posterior covariance after the wheel-yaw update.
The four diagonal entries of $P_k$ have units $\mathrm{m}^2$,
$\mathrm{m}^2$, $\mathrm{rad}^2$, and $(\mathrm{rad/s})^2$, respectively.
Off-diagonal entries describe correlations, for example how heading error can
become position error while the robot moves.

The process covariance $Q_k$ is assembled from three configured sources:

- gyro-rate noise, mapped through the sensitivity of position and yaw to
  angular-rate error;
- wheel-speed noise, mapped through the sensitivity of position to forward
  speed error;
- gyro-bias random walk, added to the bias variance over the time interval.

More explicitly, the implementation forms two sensitivity vectors:

$$
G_{\omega,k}=\begin{bmatrix}
-\frac{1}{2}\Delta s_k\Delta t_k\sin(\theta_{\mathrm{mid},k})\\
\frac{1}{2}\Delta s_k\Delta t_k\cos(\theta_{\mathrm{mid},k})\\
\Delta t_k\\
0
\end{bmatrix},
\qquad
G_{v,k}=\begin{bmatrix}
(1-s)\Delta t_k\cos(\theta_{\mathrm{mid},k})\\
(1-s)\Delta t_k\sin(\theta_{\mathrm{mid},k})\\
0\\
0
\end{bmatrix}.
$$

The process covariance is then calculated as

$$
Q_k=\sigma_{\omega}^{2}G_{\omega,k}G_{\omega,k}^{\mathsf T}
+\sigma_{v}^{2}G_{v,k}G_{v,k}^{\mathsf T}
+\begin{bmatrix}
0&0&0&0\\
0&0&0&0\\
0&0&0&0\\
0&0&0&q_b\Delta t_k
\end{bmatrix}.
$$

Here $\sigma_{\omega}$ is `gyro_rate_noise_std_rad_s`, $\sigma_v$ is
`wheel_speed_noise_std_m_s`, and $q_b$ is the configured gyro-bias
random-walk variance. The vector outer products distribute sensor noise into
the states that it physically affects instead of adding the same scalar noise
to every state. In the implementation, the configured standard deviations are
squared before entering these covariance terms.

Thus uncertainty grows during motion even when no wheel-yaw update is
available. The initial variances are explicit parameters in the Python class;
the default position variance is `0.25 m^2`, which explains why a first smoke
run can report a relatively large x/y covariance.

### 4. Compare wheel yaw with the prediction

Wheel yaw measures only the third state component, so the measurement matrix is

$$
H=\begin{bmatrix}0 & 0 & 1 & 0\end{bmatrix}.
$$

The wrapped difference between measurement and predicted yaw is

$$
\nu_k=\mathrm{wrap}(\theta_k^{\mathrm{wheel}}-\theta_k^-).
$$

The innovation variance combines predicted yaw uncertainty and wheel-yaw
measurement noise:

$$
S_k=HP_k^-H^{\mathsf T}+R_{\mathrm{wheel}},
$$

where

$$
R_{\mathrm{wheel}}=\left(\sigma_{\mathrm{wheel\ yaw}}\right)^2.
$$

Thus $S_k$ is the predicted variance of the yaw disagreement. It contains two
parts: uncertainty already present in the predicted heading, plus uncertainty
in the wheel-yaw sensor itself. It is measured in rad$^2$.

The Kalman gain is a four-component vector:

$$
K_k=P_k^-H^{\mathsf T}S_k^{-1}.
$$

The gain is dimensionless for the heading component. Its other components have
the units needed to map a yaw residual into a correction of position or gyro
bias. A larger predicted uncertainty in a state generally allows a larger
correction; a larger $R_{\mathrm{wheel}}$ makes the filter trust wheel yaw
less.

Although the wheel measurement is only yaw, the gain can also contain x, y,
and bias components because the covariance contains cross-correlations. The
state correction is

$$
\mathbf{x}_k=\mathbf{x}_k^-+K_k\nu_k.
$$

In plain language: a large predicted yaw uncertainty increases the correction
toward wheel yaw; a large wheel-yaw noise variance decreases it. The update is
not averaging x/y from two independent position sensors. Wheel forward speed
drives position prediction, and wheel yaw corrects heading and any correlated
state uncertainty.

### 5. Check the measurement with NIS

Before applying the update, the filter computes

$$
\mathrm{NIS}_k=\frac{\nu_k^2}{S_k}.
$$

If `NIS` is larger than `nis_gate_threshold`, the measurement is treated as an
outlier and the predicted state and covariance are kept unchanged. Otherwise
the update is accepted. The default threshold `9.0` is a transparent scalar
approximately three-sigma gate. It is a consistency check, not a learned fault
detector.

For an accepted update, the implementation uses the Joseph covariance form:

$$
P_k=(I-K_kH)P_k^-(I-K_kH)^{\mathsf T}
+K_kR_{\mathrm{wheel}}K_k^{\mathsf T}.
$$

This form is used because it better preserves covariance symmetry and
non-negative diagonal values under repeated floating-point updates.

## Wheel-yaw measurement and NIS gate

Wheel yaw is used as a scalar heading measurement. The wrapped innovation is

$$
\nu_k=\mathrm{wrap}\left(\theta_k^{\mathrm{wheel}}-\theta_k^-\right).
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
\mathrm{NIS}_k=\frac{\nu_k^2}{S_k}.
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
