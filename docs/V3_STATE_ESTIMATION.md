# V3 State Estimation: first milestone

V2 freezes the deterministic navigation and safety baseline. V3 starts by
separating the pose used for navigation from Gazebo's ideal `/odom` topic.

## Topics

```text
/wheel_odom ─┐
             ├── heading_estimator ──> /state_estimate ──> estimation_logger
/imu        ─┘

/odom ───────────────────────────────> evaluation_logger and estimation_logger
```

The estimator currently keeps `x` and `y` from wheel odometry and estimates
heading from two complementary signals:

- the IMU gyro provides short-term angular motion;
- wheel-odometry yaw provides a slow reference that limits gyro drift.

This is intentionally a transparent heading-fusion baseline, not yet an EKF.
The IMU orientation field is not consumed by the estimator, so Gazebo's ideal
orientation cannot silently become a controller measurement.

The estimator also has an optional first wheel-slip perturbation. For a slip
ratio `s`, each translational wheel-odometry increment is reported as
`(1 - s)` of the raw increment. This is a controlled measurement model for
localization experiments, not a Gazebo tire-friction or contact simulation;
wheel yaw is unchanged in this first version.

## Equations

For an IMU sample at time step `k`:

$$
\theta^{\mathrm{imu}}_{k+1} = \mathrm{wrap}(\theta^{\mathrm{imu}}_{k} + \omega_{z,k}\Delta t)
$$

The wheel-odometry yaw is used as a slow correction:

$$
\theta^{\mathrm{fused}}_{k} = \mathrm{wrap}(\theta^{\mathrm{fused}}_{k} +
\lambda\,\mathrm{wrap}(\theta^{\mathrm{wheel}}_{k} - \theta^{\mathrm{fused}}_{k}))
$$

The launch parameter `wheel_weight` is the complementary-fusion value
$\lambda$. The default $\lambda = 0.02$ gives the gyro the short-term role and
wheel yaw a small drift-correction role. The value is a first experimental
parameter, not a statistical covariance estimate. It is recorded as
`configured_wheel_weight` in estimator diagnostics so weight sweeps remain
reproducible.

## Running V3 explicitly

The default launch remains the V2-compatible ideal baseline:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom
```

The validated V3 diagnostic keeps navigation on `/odom` while the estimator
runs in parallel:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_heading_diagnostic.csv
```

Switching navigation to `/state_estimate` remains exploratory because the
current estimator does not yet provide validated `x/y` localization. The
`0.41 m` radius is useful for that later experiment because it is the smallest
radius that achieved consistent success in the tested V2 uncertainty cases.

To compare heading-fusion weights while keeping the navigation stack on the
validated V2 pose topic:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  wheel_weight:=0.005 \
  experiment_timeout_s:=120.0 \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_wheel_weight_005_metrics.csv
```

Use `wheel_weight:=0.0`, `0.005`, and `0.02` as the first controlled sweep.
This changes only the heading correction toward wheel yaw; it does not remove
the wheel-odometry position error.

## V3.3 adaptive heading and gyro-bias fusion

The next estimator mode is selected with `fusion_mode:=adaptive`. It keeps a
two-element state, heading and gyro bias:

$$
\mathbf{x}_k =
\begin{bmatrix}
\theta_k \\
b_{g,k}
\end{bmatrix}
$$

The IMU rate predicts the state and covariance. Wheel yaw is then treated as a
scalar heading measurement. The correction gain is computed from the predicted
heading variance and configured wheel-yaw variance:

$$
K_k = \frac{P_k^-}{P_k^- + R_{\mathrm{wheel}}}
$$

The estimator publishes the dynamic gain, estimated gyro bias, and heading
innovation on `/heading_fusion_gain`, `/gyro_bias_estimate`, and
`/heading_fusion_innovation`. These topics are diagnostics only; they do not
provide Gazebo ground truth to the estimator.

The first adaptive experiment can keep navigation on `/odom` so that heading
fusion is evaluated independently of the known wheel-odometry position error:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  fusion_mode:=adaptive \
  gyro_rate_noise_std_rad_s:=0.01 \
  wheel_yaw_noise_std_rad:=0.07 \
  gyro_bias_random_walk_std_rad_s2:=0.001 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_adaptive_heading_eval.csv \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_adaptive_heading_metrics.csv
```

The fixed-gain mode remains available for comparison. The adaptive mode is a
transparent two-state EKF-style filter, not yet a full covariance-aware
position estimator.

### V3.4 innovation-adaptive wheel-yaw noise

The fixed `wheel_yaw_noise_std_rad` is still the reference configuration. An
optional robust extension can update the wheel-yaw variance from the recent
innovation sequence:

$$
\widehat{S}_{k} = (1-\beta)\widehat{S}_{k-1} + \beta\nu_{k}^{2}
$$

$$
R_{\mathrm{wheel},k} = \mathrm{clip}(\widehat{S}_{k} - P_{k}^{-}, R_{\min}, R_{\max})
$$

The innovation used by the update is

$$
\nu_{k} = \mathrm{wrap}(\theta_{k}^{\mathrm{wheel}} - \theta_{k}^{-})
$$

Here $\nu_{k}$ is measured in radians, while $\widehat{S}_{k}$,
$P_{k}^{-}$, and $R_{\mathrm{wheel},k}$ are variances in $\mathrm{rad}^{2}$.
The parameter $\beta$ is `wheel_noise_adaptation_rate`; it controls how
quickly recent innovations change the variance estimate.

The implementation uses
$R_{\min}=\sigma_{\min}^{2}$ and $R_{\max}=\sigma_{\max}^{2}$, where the
launch parameters specify standard deviations. The clipping operation keeps
the inferred variance inside these configured physical bounds. A large recent
innovation therefore increases the apparent wheel uncertainty and reduces the
correction gain, while a quiet innovation sequence allows the gain to recover.

The corresponding scalar correction gain is

$$
K_{k} = \frac{P_{k}^{-}}{P_{k}^{-} + R_{\mathrm{wheel},k}}
$$

This is an innovation-based measurement-quality heuristic, not a direct
measurement of sensor noise and not a full adaptive EKF. In particular, a
persistent bias can produce a persistent innovation, so the adaptive variance
should be interpreted together with the estimated gyro bias and the fixed-$R$
reference run.

The current estimate is published on `/wheel_yaw_noise_std_estimate` and is
recorded by `estimation_logger`.

## V3.5 propagated position estimate

The estimator now has two explicit position modes:

- `position_mode:=wheel_pose` preserves the earlier diagnostic baseline and
  reports wheel-odometry position increments directly;
- `position_mode:=propagated` integrates the wheel-odometry body-forward speed
  using the fused heading.

For the propagated mode, the translational increment is

$$
\Delta s_k = (1-s) v_{x,k} \Delta t_k
$$

where $s$ is the configured wheel-slip ratio. The heading used for the
increment is the midpoint between consecutive fused headings:

$$
\theta_{\mathrm{mid},k} = \mathrm{wrap}(\hat{\theta}_{k-1} + \frac{1}{2}\mathrm{wrap}(\hat{\theta}_{k} - \hat{\theta}_{k-1}))
$$

and the position update is

$$
\hat{x}_{k} = \hat{x}_{k-1} + \Delta s_{k}\cos(\theta_{\mathrm{mid},k})
$$

$$
\hat{y}_{k} = \hat{y}_{k-1} + \Delta s_{k}\sin(\theta_{\mathrm{mid},k})
$$

The first wheel-odometry sample initializes the local estimate. Gazebo's ideal
`/odom` remains evaluation-only and is not used by this propagation model.
This is a unicycle/differential-drive propagation experiment, not yet a full
wheel-encoder covariance model.

## V3.6 local LiDAR--map position correction

The odometry-only navigation experiments showed a useful failure mode: the
controller can reach the goal in `/state_estimate` while the physical `/odom`
pose is still outside the goal tolerance. The next estimator therefore adds an
optional local scan-to-map correction.

The `lidar_localizer` node consumes:

```text
/state_estimate + /scan + /map
              ↓
       /localized_estimate
```

For each candidate pose, the matcher raycasts the static map and predicts the
first occupied-cell range $\hat{r}_i$. The measured range residual is

$$
e_i(x,y,\theta) = |r_i - \hat{r}_i(x,y,\theta;\mathcal{M})|
$$

For a valid LiDAR return, the corresponding map endpoint is described by its
two Cartesian components:

$$
p_{i,x}^{\mathrm{map}} = x + r_i\cos(\theta + \alpha_i)
$$

$$
p_{i,y}^{\mathrm{map}} = y + r_i\sin(\theta + \alpha_i)
$$

The matcher searches a small grid around the wheel/IMU position. For a candidate
position and candidate heading, its score is

$$
J(x,y,\theta) = \frac{1}{N}\sum_{i=1}^{N} e_i(x,y,\theta) + \lambda((x-\hat{x}_{\mathrm{odom}})^2 + (y-\hat{y}_{\mathrm{odom}})^2)
$$

The corrected position is denoted by

$$
(\hat{x}_{k},\hat{y}_{k})
$$

and is the nearby candidate with the smallest score $J$.

Only `x` and `y` are corrected; heading remains the fused wheel/IMU heading.
This is a deliberately small local scan matcher, not a general SLAM system:
it assumes a known static map in the `map` frame, a planar odometry estimate in
the `odom` frame, and a sufficiently small odometry error. It does not consume
Gazebo ground-truth `/odom`.

The node maintains a persistent planar `map → odom` correction. If the input
pose is $p_{\mathrm{odom}}$, the published pose is

$$
p_{\mathrm{map}} = T_{\mathrm{map}\leftarrow\mathrm{odom}}
\oplus p_{\mathrm{odom}}.
$$

An accepted candidate estimates a new transform from the matched map pose and
the current odometry pose. The transform is interpolated using
`localization_correction_smoothing`; rejected matches leave the previous
transform unchanged. The output is opt-in. Before publishing a correction, the node applies three
guards: the match must contain enough valid scan points, its mean range score
must be below `localization_max_match_score_m`, and its displacement from the
input pose must be below `localization_max_correction_m`. An accepted correction
must also remain consistent for
`localization_minimum_consecutive_matches` updates. An accepted correction is
applied gradually using `localization_correction_smoothing`; a rejected match
keeps the previous persistent transform. The node publishes the applied correction
magnitude on `/localization_correction_m`, the raw score on
`/localization_match_score_m`, the raw candidate displacement on
`/localization_candidate_correction_m`, and a validity flag on
`/localization_match_valid`. The human-readable reason is published on
`/localization_match_status`; the applied heading correction is published on
`/localization_heading_correction_rad`. The matcher now searches a bounded
local $(x,y,\theta)$ window, while all corrections remain gated and smoothed.
It also requires a minimum improvement over the input-pose scan residual,
reported on `/localization_score_improvement_m`, before applying a correction.
When enabled, the node broadcasts the persistent transform on `/tf` as
`map → odom`; `/localized_estimate` is the corresponding pose in the map
frame for compatibility with the current planner.
To test it as the navigation pose:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/localized_estimate \
  position_mode:=wheel_pose \
  localization_scan_stride:=6 \
  localization_max_correction_m:=0.15 \
  localization_max_match_score_m:=0.12 \
  localization_correction_smoothing:=0.25 \
  localization_minimum_consecutive_matches:=3 \
  localization_candidate_consistency_m:=0.05 \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_navigation_lidar_localized_eval.csv \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_navigation_lidar_localized_metrics.csv
```

The default `/odom` navigation path is unchanged. A successful V3.6 result
must be judged by physical `/odom` goal completion, not only by
`navigation_pose_goal_reached`.

This distinction is observable in the estimator-only control check. With
`navigation_pose_topic:=/state_estimate` and no wheel slip, the follower reached
the estimated-pose tolerance at approximately `76.43 s`, while the physical
`/odom` pose was still about `0.160 m` from the goal when the 90-second
experiment timeout ended. The run therefore remains an incomplete physical
navigation result, even though the navigation-pose goal flag was true.

The gates are deliberately conservative. They protect the controller from a
locally plausible but globally incorrect scan match; they do not turn this
matcher into a globally convergent localization algorithm. If the match is
rejected repeatedly, `/localized_estimate` is intentionally identical to the
input estimate and the correction topic remains zero.

### V3.6 validation result

The gated localizer was first run in diagnostic mode while navigation remained
on `/odom`. That baseline completed normally with a final physical error of
approximately `0.049 m`. The localizer diagnostics repeatedly showed
`inconsistent_candidate` and `insufficient_score_improvement`. Thus, a
candidate can have a low absolute residual without being consistently better
than the input pose.

Earlier closed-loop localizer trials timed out with the robot far from the
goal. Therefore `/localized_estimate` is not a validated navigation source.
This is a useful negative result: local raycast matching with a static map and
a drifting heading is insufficient for closed-loop localization on this map.
The no-slip and denser-scan trials also produced large, inconsistent candidate
corrections, so the failure is not explained only by wheel slip or scan
subsampling. The next localization step should first improve the observation
model and add covariance/observability diagnostics to the persistent transform,
then validate it over repeated runs rather than simply loosening the gates.

First validate the propagated estimate without changing the navigation input:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  position_mode:=propagated \
  wheel_slip_ratio:=0.0 \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_propagated_eval.csv \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_propagated_metrics.csv
```

Only after this diagnostic is understood should navigation be switched to
`navigation_pose_topic:=/state_estimate`. The evaluation logger will then keep
physical `/odom` success separate from the estimated-pose goal belief.

To run this mode while keeping the validated V2 pose on the navigation path:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  fusion_mode:=adaptive \
  adaptive_wheel_noise:=true \
  imu_gyro_bias_rad_s:=0.01 \
  gyro_rate_noise_std_rad_s:=0.005 \
  wheel_yaw_noise_std_rad:=0.05 \
  wheel_yaw_noise_min_std_rad:=0.03 \
  wheel_yaw_noise_max_std_rad:=0.15 \
  wheel_noise_adaptation_rate:=0.05 \
  gyro_bias_random_walk_std_rad_s2:=0.0003 \
  experiment_timeout_s:=120.0 \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_adaptive_R_metrics.csv
```

`evaluation_logger` continues to use `/odom` for closed-loop metrics. The new
`estimation_logger` also uses `/odom` only as an evaluation reference and
compares it with `/wheel_odom` and `/state_estimate`. It writes one summary row
containing position RMSE, heading RMSE, signed bias, and final error for the
wheel, pure-gyro, and fused estimates. The pure-gyro row is integrated from
`/imu/angular_velocity.z` and initialized from wheel yaw; it does not consume
the IMU orientation field. Neither logger publishes a pose or control command.

When `navigation_pose_topic` is `/state_estimate`, the evaluation CSV keeps
the physical `/odom` result as `success`, `final_error_m`, and
`time_to_goal_s`, and additionally records
`navigation_pose_goal_reached`, `navigation_pose_final_error_m`,
`navigation_pose_time_to_goal_s`, and
`navigation_pose_goal_error_gap_m`. The last field is the absolute difference
between the final estimated-pose goal error and the physical `/odom` goal
error. `navigation_pose_reached_before_ground_truth` flags the case where the
controller's pose estimate enters the tolerance before the physical pose
does. This makes premature goal belief visible when the controller stops
using an inaccurate estimated pose.

The logger labels a shell-generated SIGINT as `external_interrupt`; this is
different from the internal `experiment_timeout` and does not imply that a
person pressed Ctrl+C.

To collect the first diagnostic row while keeping navigation on the validated
V2 pose topic:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_heading_diagnostic.csv \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_estimation_metrics.csv
```

## Summarising uncertainty experiments

The estimator logger can inject a deterministic gyro bias and reproducible
zero-mean white noise into the estimator's gyro measurement model. The logger
still records the unmodified `/imu` topic as a diagnostic reference, so the
configured uncertainty is visible in the fused-estimate columns rather than
silently changing the raw sensor topic.

To aggregate completed bias/noise runs by configuration and retain their seed
list:

```bash
python tools/summarize_estimation.py \
  --glob "results/v3_*_metrics.csv" \
  --output results/estimation_summary.csv
```

The summary reports wheel, raw-gyro, and fused heading RMSE, position RMSE,
and the percentage change from wheel heading RMSE to fused heading RMSE. A
positive improvement means that the fused heading RMSE is lower than the
wheel-odometry heading RMSE; it is not a statistical guarantee from the
current small number of seeds.

To run a first wheel-slip diagnostic while keeping navigation on the validated
`/odom` topic:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  wheel_slip_ratio:=0.10 \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  estimation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/v3_wheel_slip_010_metrics.csv
```

## Next experiments

1. Keep `/odom` as the validated navigation source and retain the localizer as
   a diagnostic experiment.
2. Compare the transparent V3 estimator against a covariance-aware EKF or a
   mature map-localization package.
3. Repeat localization experiments only after adding uncertainty estimates and
   a proper `map → odom` frame treatment.
