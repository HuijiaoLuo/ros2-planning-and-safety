# State Estimation

This document describes the transparent wheel/IMU estimator used for the V3
experiments. It intentionally stops before full pose-EKF navigation. The
covariance-aware EKF and the LiDAR-to-map localizer are documented separately
in [`POSE_EKF.md`](POSE_EKF.md) and [`LOCALIZATION.md`](LOCALIZATION.md).

## Scope and data flow

```text
/wheel_odom ─┐
             ├── heading_estimator ──> /state_estimate
/imu        ─┘

/odom ───────────────────────────────> evaluation_logger and estimation_logger
```

The estimator consumes wheel odometry and the IMU angular velocity. It does
not consume the IMU orientation field and does not consume Gazebo ground-truth
`/odom` for estimation. `/odom` is used by the loggers as an evaluation
reference.

The original estimator keeps wheel-odometry `x` and `y` and estimates heading
from two signals:

- the gyro supplies short-term angular motion;
- wheel-odometry yaw provides a slower drift reference.

This makes the measurement path explicit before introducing a full covariance
model. A controlled wheel-slip ratio can reduce translational increments, but
it is a measurement perturbation, not a Gazebo tire-contact simulation.

## Heading integration and complementary fusion

For an IMU sample at time step $k$:

$$
\theta^{\mathrm{imu}}_{k+1}
=\mathrm{wrap}\left(
\theta^{\mathrm{imu}}_{k}+\omega_{z,k}\Delta t_k
\right).
$$

The fixed fusion mode applies a small correction toward wheel yaw:

$$
\theta^{\mathrm{fused}}_{k}
=\mathrm{wrap}\left(
\theta^{\mathrm{fused}}_{k-1}
+\lambda\mathrm{wrap}\left(
\theta^{\mathrm{wheel}}_{k}-\theta^{\mathrm{fused}}_{k-1}
\right)
\right).
$$

`wheel_weight` is the transparent tuning parameter $\lambda$. The default
value is `0.02`; it is not a covariance-derived Kalman gain.

## Adaptive heading and gyro-bias fusion

With `fusion_mode:=adaptive`, the estimator keeps a two-element state:

$$
\mathbf{x}_k=(\theta_k,\,b_{g,k})^{\mathsf T},
$$

where $b_{g,k}$ is the estimated gyro bias. The IMU angular rate predicts the
state and covariance. Wheel yaw is a scalar heading measurement with gain

$$
K_k=\frac{P_k^-}{P_k^-+R_{\mathrm{wheel}}}.
$$

The estimator publishes the gain, bias estimate, and innovation on
`/heading_fusion_gain`, `/gyro_bias_estimate`, and
`/heading_fusion_innovation`. These are diagnostics, not hidden ground-truth
inputs.

## Innovation-adaptive wheel-yaw noise

The fixed `wheel_yaw_noise_std_rad` remains the reference configuration. The
optional adaptive mode updates the wheel-yaw variance from recent innovations:

$$
\widehat{S}_{k}=(1-\beta)\widehat{S}_{k-1}+\beta\nu_k^2,
$$

$$
R_{\mathrm{wheel},k}
=\mathrm{clip}\left(
\widehat{S}_{k}-P_k^-,R_{\min},R_{\max}
\right),
$$

where

$$
\nu_k=\mathrm{wrap}\left(
\theta_k^{\mathrm{wheel}}-\theta_k^-
\right).
$$

$\nu_k$ is measured in radians. $\widehat{S}_k$, $P_k^-$, and
$R_{\mathrm{wheel},k}$ are variances in $\mathrm{rad}^2$. The launch parameter
`wheel_noise_adaptation_rate` is $\beta$. The implementation uses
$R_{\min}=\sigma_{\min}^2$ and $R_{\max}=\sigma_{\max}^2$, because launch
parameters specify standard deviations.

A large recent innovation makes the wheel measurement appear less reliable;
the gain decreases. A quiet innovation sequence allows the gain to recover.
This is a measurement-quality heuristic, not a direct noise measurement and
not a full adaptive EKF. Persistent bias can also create persistent
innovation, so the adaptive variance must be interpreted with the gyro-bias
estimate and fixed-variance reference runs.

## Propagated position mode

The estimator has two position modes:

- `position_mode:=wheel_pose` reports wheel-odometry position increments;
- `position_mode:=propagated` integrates the body-forward speed using fused
  heading.

For the propagated mode:

$$
\Delta s_k=(1-s)v_{x,k}\Delta t_k,
$$

where $s$ is `wheel_slip_ratio`. The midpoint heading is

$$
\theta_{\mathrm{mid},k}
=\mathrm{wrap}\left(
\hat{\theta}_{k-1}
+\frac{1}{2}\mathrm{wrap}\left(
\hat{\theta}_{k}-\hat{\theta}_{k-1}
\right)
\right).
$$

The position update is

$$
\hat{x}_{k}=\hat{x}_{k-1}+\Delta s_k\cos(\theta_{\mathrm{mid},k}),
$$

$$
\hat{y}_{k}=\hat{y}_{k-1}+\Delta s_k\sin(\theta_{\mathrm{mid},k}).
$$

The first wheel-odometry sample initializes the local estimate. Gazebo's
ideal `/odom` is evaluation-only and is not used in this propagation model.

## Diagnostic commands

Run a V3 diagnostic while keeping the validated navigation input:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_heading_diagnostic.csv \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_estimation_metrics.csv
```

To summarize estimator experiments:

```bash
python tools/summarize_estimation.py \
  --glob "results/v3_*_metrics.csv" \
  --output results/estimation_summary.csv
```

The summary reports wheel, pure-gyro, and fused heading/position errors. A
positive heading improvement means the fused heading RMSE is lower than the
wheel-odometry heading RMSE; it is not a statistical guarantee with the
current small number of seeds.

For the covariance-aware pose filter, see
[`POSE_EKF.md`](POSE_EKF.md). For the optional known-map scan matcher, see
[`LOCALIZATION.md`](LOCALIZATION.md).
