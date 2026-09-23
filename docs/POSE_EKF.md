# Covariance-aware Pose Estimator

This document describes the covariance-aware full-pose estimator. It is
separate from the earlier transparent estimator in
[`STATE_ESTIMATION.md`](STATE_ESTIMATION.md) and remains diagnostic-only until
its covariance and closed-loop behavior are validated under controlled
uncertainty.

## State and prediction model

The filter state is

$$
\mathbf{x}_k=(x_k,\,y_k,\,\theta_k,\,b_{g,k},\,b_{w,k})^{\mathsf T}.
$$

where $b_{g,k}$ is the estimated gyro bias and $b_{w,k}$ is the estimated
wheel-yaw bias. The IMU supplies angular rate and wheel odometry supplies
body-forward speed and wheel yaw. The bias-corrected rate is

Here $b_{g,k}$ is not a value directly measured by the IMU. It is an internal
EKF state representing the gyro's slowly varying zero-rate offset: if the robot
were perfectly still, the real angular velocity would be zero, but the gyro
could still report a small non-zero value. The filter estimates this offset in
rad/s and subtracts it from the raw angular-rate measurement.

$$
\omega_k=\omega_{z,k}-b_{g,k}.
$$

Here $\omega_{z,k}$ is the raw IMU measurement of rotation about the vertical
axis at sample $k$, $b_{g,k}$ is the current estimated false rotation rate,
and $\omega_k$ is the angular rate used by the motion model. This equation
uses the current EKF estimate of the bias rather than subtracting a manually
chosen constant.

The symbols have the following physical meanings:

| Symbol | Physical quantity | Unit | Source or role |
| --- | --- | --- | --- |
| $x_k,y_k$ | estimated robot position in the local planar frame | m | EKF state |
| $\theta_k$ | estimated robot heading about the vertical axis | rad | EKF state |
| $b_{g,k}$ | slowly varying gyro zero-rate bias | rad/s | EKF state |
| $b_{w,k}$ | slowly varying wheel-yaw zero offset | rad | EKF state |
| $\omega_{z,k}$ | measured vertical-axis angular velocity | rad/s | `/imu.angular_velocity.z` |
| $v_{x,k}$ | measured forward body velocity | m/s | `/wheel_odom.twist.twist.linear.x` |
| $s$ | configured mean fractional wheel-slip loss | dimensionless | launch parameter |
| $\sigma_s$ | uncertainty of the configured slip fraction | dimensionless | EKF process-noise parameter |
| $\Delta t_k$ | time between consecutive IMU samples | s | message timestamps |
| $P_k$ | covariance of $[x,y,\theta,b_g,b_w]$ | mixed squared units | EKF uncertainty |
| $Q_k$ | uncertainty added by one motion step | mixed squared units | configured process noises |
| $R_{\mathrm{wheel}}$ | variance of one wheel-yaw measurement | rad$^2$ | wheel-yaw noise parameter |

The state is therefore not a perfect pose copied from Gazebo. It is the EKF's
best current estimate, and $P_k$ describes how uncertain that estimate is.
The wheel-yaw bias is an internal state, not a second sensor measurement. It
allows a persistent wheel-yaw offset to be represented separately from white
wheel-yaw measurement noise. With only gyro rate and wheel yaw, however, a
constant $b_w$ is not absolutely observable: an offset in the initial heading
can produce the same wheel-yaw sequence. Therefore this state is first a
bias-aware uncertainty model; identifying its physical value requires an
external absolute heading or map constraint.

## Gyro-bias operating mode

The symbol $b_{g,k}$ has two possible roles, selected by the launch parameter
`gyro_bias_mode`:

- `gyro_bias_mode:=estimated`: $b_{g,k}$ is an EKF state. The filter gives it a
  random-walk process model and may update it through the wheel-yaw innovation
  when the covariance contains a correlation between gyro bias and heading.
- `gyro_bias_mode:=fixed`: $b_{g,k}$ is a pre-calibrated constant. Its value is
  `initial_gyro_bias_rad_s`; the wheel-yaw update cannot change it, and its
  covariance row and column are kept zero.

In both modes, $b_{g,k}$ is not the raw IMU measurement. It is either an
estimated or pre-calibrated model parameter representing the IMU's zero-rate
offset. The raw measurement is $\omega_{z,k}$, and the motion model uses

$$
\omega_k = \omega_{z,k} - b_{g,k}.
$$

In estimated mode, the mean bias is normally propagated as constant between
updates:

$$
b_{g,k}^{-}=b_{g,k-1}+w_{b,k},
$$

where $w_{b,k}$ has zero mean and its variance is represented in $Q_k$. The
measurement update may then change the estimate. In fixed mode, the equivalent
mean model is simply

$$
b_{g,k}^{-}=b_{g,k}=b_{g,0}.
$$

Here $b_{g,0}$ is the configured initial bias supplied by
`initial_gyro_bias_rad_s`. Keeping the parameter name in ordinary Markdown
code formatting avoids putting underscores inside a math command, which is
not supported by all Markdown equation renderers.

This distinction is important for diagnosis. With only gyro rate and wheel
yaw, a persistent wheel-yaw modelling error can be absorbed by an estimated
$b_g$. The fixed mode removes that degree of freedom and asks the experiment
to evaluate wheel-yaw fusion with the gyro calibration held constant. The
simulation parameter `imu_gyro_bias_rad_s` is separate: it injects a bias into
the simulated IMU, while `initial_gyro_bias_rad_s` supplies the estimator's
initial calibration value.

The translational increment uses the configured wheel-slip ratio $s$:

$$
\Delta s_k=(1-s)v_{x,k}\Delta t_k.
$$

Here $v_{x,k}$ is the forward speed reported by wheel odometry, $\Delta t_k$
is the time elapsed since the previous IMU sample, and $s$ is a dimensionless
fractional loss. For example, $s=0.10$ means that the estimator models only
90% of the measured forward travel. In this experiment $s$ is configured; the
EKF does not estimate it.

The filter uses the midpoint of consecutive headings:

$$
\theta_{\mathrm{mid},k}
=\mathrm{wrap}\left(
\theta_{k-1}
+\frac{1}{2}\mathrm{wrap}(\theta_k-\theta_{k-1})
\right).
$$

The midpoint heading is used because the robot can rotate while it translates
during one sample interval. It approximates the direction of travel during
that interval instead of using only the heading at its beginning or end.

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

The superscript $-$ means “predicted before the new wheel-yaw measurement is
used”. Thus $x_k^-$, $y_k^-$, and $\theta_k^-$ are the pose obtained by
integrating the motion model; they are not a second sensor reading.

The covariance is propagated with the linearized model:

$$
P_k^-=F_kP_{k-1}F_k^{\mathsf T}+Q_k.
$$

$Q_k$ contains configured gyro-rate noise, wheel-speed noise, gyro-bias
random-walk noise, wheel-yaw-bias random-walk noise, and optional slip
uncertainty. The mean slip ratio is configured; it is not estimated as an
additional state.

The EKF therefore maintains both a state vector and a confidence description.
The state says where the robot is estimated to be. The covariance $P$ says how
uncertain that estimate is and which errors are correlated. Its diagonal terms
have units $\mathrm{m}^2$, $\mathrm{m}^2$, $\mathrm{rad}^2$,
$(\mathrm{rad/s})^2$, and $\mathrm{rad}^2$ for position, position, heading,
gyro bias, and wheel-yaw bias.

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
x_k^-=x_{k-1}+\Delta s_k\cos(\theta_{\mathrm{mid},k}),
$$

$$
y_k^-=y_{k-1}+\Delta s_k\sin(\theta_{\mathrm{mid},k}),
$$

$$
\theta_k^-=\mathrm{wrap}(\theta_{k-1}+\omega_k\Delta t_k).
$$

The superscript $-$ means “before using the new wheel-yaw measurement”.

### 3. Predict the covariance

The covariance $P$ describes uncertainty in $x$, $y$, yaw, gyro bias, and
wheel-yaw bias. The
EKF linearizes the motion model around the current estimate. In this
implementation the state-transition Jacobian is approximately

The non-zero entries used by the implementation are

$$
F_{11}=1,\quad F_{22}=1,\quad F_{33}=1,\quad F_{44}=1,\quad F_{55}=1,
$$

$$
F_{13}=-\Delta s_k\sin(\theta_{\mathrm{mid},k}),\quad
F_{23}=\Delta s_k\cos(\theta_{\mathrm{mid},k}),
$$

$$
F_{14}=\frac{1}{2}\Delta s_k\Delta t_k
\sin(\theta_{\mathrm{mid},k}),\quad
F_{24}=-\frac{1}{2}\Delta s_k\Delta t_k
\cos(\theta_{\mathrm{mid},k}),\quad
F_{34}=-\Delta t_k.
$$

All other entries of $F_k$ are zero. The index pair $F_{ij}$ means the
sensitivity of state component $i$ to a small change in state component $j$.

The first two rows show how heading uncertainty bends the predicted position.
The fourth column shows how gyro-bias uncertainty accumulates into heading and
then into position. The wheel-yaw-bias state does not affect motion
prediction directly; it follows a random walk and enters the measurement
model below. The covariance prediction is

$$
P_k^-=F_kP_{k-1}F_k^{\mathsf T}+Q_k.
$$

The superscript $-$ means the predicted, pre-measurement quantity. Without the
superscript, $P_k$ denotes the posterior covariance after the wheel-yaw update.
The five diagonal entries of $P_k$ have units $\mathrm{m}^2$,
$\mathrm{m}^2$, $\mathrm{rad}^2$, $(\mathrm{rad/s})^2$, and $\mathrm{rad}^2$,
respectively.
Off-diagonal entries describe correlations, for example how heading error can
become position error while the robot moves.

The process covariance $Q_k$ is assembled from five configured sources:

- gyro-rate noise, mapped through the sensitivity of position and yaw to
  angular-rate error;
- wheel-speed noise, mapped through the sensitivity of position to forward
  speed error;
- gyro-bias random walk, added to the bias variance over the time interval;
- wheel-yaw-bias random walk, added to the wheel-yaw-bias variance over the
  time interval;
- uncertainty in the configured wheel-slip fraction, mapped into x/y travel.

More explicitly, the implementation forms three motion-noise sensitivity
vectors:

$$
G_{\omega,k}=\left(
-\frac{1}{2}\Delta s_k\Delta t_k\sin(\theta_{\mathrm{mid},k}),\,
\frac{1}{2}\Delta s_k\Delta t_k\cos(\theta_{\mathrm{mid},k}),\,
\Delta t_k,\,0,\,0
\right)^{\mathsf T},
$$

$$
G_{v,k}=\left(
(1-s)\Delta t_k\cos(\theta_{\mathrm{mid},k}),\,
(1-s)\Delta t_k\sin(\theta_{\mathrm{mid},k}),\,0,\,0,\,0
\right)^{\mathsf T}.
$$

The configured slip ratio is a mean model, not a perfectly known constant. If
the actual slip differs from $s$ by a small random amount, the position is
sensitive to that amount through

$$
G_{s,k}=\left(
-v_{x,k}\Delta t_k\cos(\theta_{\mathrm{mid},k}),\,
-v_{x,k}\Delta t_k\sin(\theta_{\mathrm{mid},k}),\,0,\,0,\,0
\right)^{\mathsf T}.
$$

The process covariance is then calculated as

$$
Q_k=\sigma_{\omega}^{2}G_{\omega,k}G_{\omega,k}^{\mathsf T}
+\sigma_{v}^{2}G_{v,k}G_{v,k}^{\mathsf T}
+\sigma_{s}^{2}G_{s,k}G_{s,k}^{\mathsf T}
+q_b\Delta t_k\,e_4e_4^{\mathsf T}
+q_w\Delta t_k\,e_5e_5^{\mathsf T},
$$

where $e_4=(0,0,0,1,0)^{\mathsf T}$ selects the gyro-bias state and
$e_5=(0,0,0,0,1)^{\mathsf T}$ selects the wheel-yaw-bias state. The
parameter `wheel_yaw_bias_random_walk_std_rad_sqrt_s` is the square root of
$q_w$, the wheel-yaw-bias random-walk variance per second.

Here $\sigma_{\omega}$ is `gyro_rate_noise_std_rad_s`, $\sigma_v$ is
`wheel_speed_noise_std_m_s`, $\sigma_s$ is `wheel_slip_noise_std`, and $q_b$
is the configured gyro-bias random-walk variance. The vector outer products
distribute each uncertainty into the states that it physically affects instead
of adding the same scalar noise to every state. In the implementation, the
configured standard deviations are squared before entering these covariance
terms.

`wheel_slip_noise_std` is not a second slip estimate and is not a direct
measurement of tire slip. It is the standard deviation of the unknown
deviation around the configured mean $s$. Setting it to zero reproduces the
earlier propagated model; increasing it makes the filter report larger x/y uncertainty
during motion when the assumed slip model may be wrong.

Thus uncertainty grows during motion even when no wheel-yaw update is
available. The initial variances are explicit parameters in the Python class;
the default position variance is `0.25 m^2`, which explains why a first smoke
run can report a relatively large x/y covariance.

### 4. Compare wheel yaw with the prediction

Wheel yaw measures physical heading plus the wheel-yaw bias, so the measurement
matrix is

$$
H=(0,0,1,0,1).
$$

The predicted wheel-yaw measurement is

$$
\widehat{\theta}_{k}^{\mathrm{wheel},-}=\theta_k^-+b_{w,k}^-.
$$

The wrapped difference between measurement and predicted wheel yaw is

$$
\nu_k=\mathrm{wrap}\left(
\theta_k^{\mathrm{wheel}}-\theta_k^- - b_{w,k}^-
\right).
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

The Kalman gain is a five-component vector:

$$
K_k=P_k^-H^{\mathsf T}S_k^{-1}.
$$

The gain is dimensionless for the heading component. Its other components have
the units needed to map a yaw residual into a correction of position, gyro
bias, or wheel-yaw bias. A larger predicted uncertainty in a state generally
allows a larger correction; a larger $R_{\mathrm{wheel}}$ makes the filter
trust wheel yaw less.

Although the wheel measurement is only yaw, the gain can also contain x, y,
and bias components because the covariance contains cross-correlations. The
state correction is

$$
\mathbf{x}_k=\mathbf{x}_k^-+K_k\nu_k.
$$

In plain language: a large predicted yaw uncertainty increases the correction
toward wheel yaw; a large wheel-yaw noise variance decreases it. A persistent
wheel offset can be represented by $b_w$, whose estimate changes slowly
according to its random-walk model, but the current sensor set cannot uniquely
separate that offset from the initial heading. The update is not averaging x/y
from two independent position sensors. Wheel forward speed drives position
prediction, and wheel yaw corrects the coupled heading/bias uncertainty.

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

Wheel yaw is used as a scalar heading-plus-bias measurement. The wrapped
innovation is

$$
\nu_k=\mathrm{wrap}\left(\theta_k^{\mathrm{wheel}}-\theta_k^- - b_{w,k}^-\right).
$$

With measurement matrix $H=[0\;0\;1\;0\;1]$, the innovation variance and gain are

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
measurement episodes, covariance means/final values, the final wheel-yaw-bias
estimate, and the usual RMSE and bias metrics. The main new EKF parameters are
`initial_position_variance_m2`,
`initial_wheel_yaw_bias_variance_rad2`, and
`wheel_yaw_bias_random_walk_std_rad_sqrt_s`.

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

## Detour-drift A/B result

The first three-scenario matrix exposed a repeatable detour error of about
`0.19 m` in the original fixed-fusion configuration. A focused v4 comparison
then held the planner, controller, map, and seed fixed while changing only the
estimator mode:

| Configuration | Baseline final state error | L-corridor final state error |
| --- | ---: | ---: |
| fixed + `wheel_pose` | `0.185 m` | `0.185 m` |
| adaptive + `propagated` | `0.214 m` | `0.201 m` |
| EKF + `propagated`, estimated gyro bias | `0.165 m` | `0.163 m` |
| EKF + `propagated`, fixed gyro bias, wheel yaw σ=`0.20 rad` | `0.016 m` | `0.048 m` |

The last row also reached the physical goal in both seed-0 runs. Its final
heading errors were `-0.006 rad` and `-0.021 rad`, while the wheel-yaw errors
were still about `-0.109 rad` and `+0.074 rad`. This isolates the failure:
the earlier EKF configuration trusted wheel yaw too strongly and its estimated
gyro-bias state absorbed part of the wheel/trajectory mismatch. The fixed-bias
and loose-wheel configuration is therefore the current candidate default, but
it remains provisional until the three-seed validation completes.

## Covariance calibration trace

The one-row metrics file cannot show whether the reported covariance is
consistent with the physical error at each instant. For that diagnostic, the
logger has an optional evaluation-only trace:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  fusion_mode:=ekf \
  position_mode:=propagated \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/ekf_covariance_metrics.csv \
  estimation_trace_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/ekf_covariance_trace.csv
```

Each trace row contains the `/odom` evaluation error, the EKF x/y/yaw
covariance, the joint position normalized squared error, and the scalar
heading normalized squared error. The trace is written by the logger only;
`/odom` is never subscribed to by the estimator, planner, follower, or safety
supervisor.

Summarize one or more traces with:

```bash
python tools/summarize_covariance_calibration.py \
  --glob "results/ekf_*_trace.csv" \
  --output results/ekf_covariance_calibration.csv
```

The reported position value is the **raw joint NEES** for the two-dimensional
$\left(x,y\right)$ error; it is not divided by its two degrees of freedom. For a
reasonably calibrated Gaussian covariance, its mean should therefore be near
`2`. The scalar heading NEES has one degree of freedom, so its mean should be
near `1`. The reported position and heading coverage use the 2-D and 1-D
chi-square thresholds `5.991` and `3.841`, respectively, and should be near
`0.95` over sufficiently many independent trials.
Trajectory samples are correlated, so these are calibration diagnostics and
not independent confidence guarantees. The current smoke covariance is known
to be broad in x/y; this trace is the required evidence before using a
Mahalanobis gate for LiDAR corrections or claiming a calibrated EKF.

## Reproducible smoke test

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  fusion_mode:=ekf \
  position_mode:=propagated \
  wheel_slip_ratio:=0.0 \
  wheel_slip_noise_std:=0.0 \
  imu_gyro_bias_rad_s:=0.0 \
  imu_gyro_noise_std_rad_s:=0.0 \
  wheel_speed_noise_std_m_s:=0.02 \
  nis_gate_threshold:=9.0 \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/ekf_smoke_eval.csv \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/ekf_smoke_metrics.csv
```

The next controlled comparison keeps the mean slip at
`wheel_slip_ratio:=0.10` and adds `wheel_slip_noise_std:=0.02` to test whether
the reported position covariance responds to model uncertainty. Only after
those results are understood should
`navigation_pose_topic:=/state_estimate` be tested in closed loop.
