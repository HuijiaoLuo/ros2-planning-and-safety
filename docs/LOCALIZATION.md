# Map-based Localization

This document describes the optional LiDAR-to-map localizer. It is an
experimental correction layer, not a SLAM system and not part of the frozen
navigation baseline.

## Data flow

```text
/state_estimate + /scan + /map
              ↓
       lidar_localizer
              ↓
       /localized_estimate
```

The localizer assumes a known static occupancy map, a planar odometry estimate
close to the true pose, and a sufficiently observable local scene. It does not
consume Gazebo ground-truth `/odom`.

## Local scan-to-map objective

For each valid LiDAR return, the localizer raycasts the map and predicts the
first occupied-cell range $\hat{r}_i$. The range residual is

$$
e_i(x,y,\theta)=\left|r_i-\hat{r}_i(x,y,\theta;\mathcal{M})\right|.
$$

The measured ray endpoint in map coordinates is

$$
p_{i,x}^{\mathrm{map}}=x+r_i\cos(\theta+\alpha_i),
$$

$$
p_{i,y}^{\mathrm{map}}=y+r_i\sin(\theta+\alpha_i).
$$

The matcher evaluates a bounded local grid around the input estimate. Its
score is

$$
J(x,y,\theta)=
\frac{1}{N}\sum_{i=1}^{N}e_i(x,y,\theta)
+\lambda\left((x-\hat{x}_{\mathrm{odom}})^2
+(y-\hat{y}_{\mathrm{odom}})^2\right).
$$

The corrected position $(\hat{x}_{k},\hat{y}_{k})$ is the nearby candidate
with the smallest score $J$. Only position is corrected; the heading remains
the fused wheel/IMU heading.

## Persistent frame correction and safety gates

The node maintains a planar `map → odom` correction. If the input pose is
$p_{\mathrm{odom}}$, the published pose is

$$
p_{\mathrm{map}}=
T_{\mathrm{map}\leftarrow\mathrm{odom}}
\oplus p_{\mathrm{odom}}.
$$

An accepted candidate updates the transform gradually using
`localization_correction_smoothing`. A rejected candidate leaves the previous
transform unchanged. The match must satisfy all of the following:

- enough valid scan points;
- mean range score below `localization_max_match_score_m`;
- candidate displacement below `localization_max_correction_m`;
- consistency over `localization_minimum_consecutive_matches` updates;
- sufficient score improvement over the input pose.

The node publishes the applied correction, candidate displacement, score,
score improvement, validity flag, status string, and heading correction on the
`/localization_*` diagnostic topics. When enabled, it broadcasts the persistent
transform on `/tf` and publishes the corrected pose on
`/localized_estimate`.

## Physical meaning of the front LiDAR clearance

The safety supervisor keeps rays within $\pm60^\circ$ of the robot's forward
axis and takes the closest valid return:

$$
d_{\mathrm{front}}(t)=
\min_{i:\,|\alpha_i|\le 60^\circ} r_i(t).
$$

Here $r_i(t)$ is the distance from the LiDAR origin to the first surface hit by
ray $i$, and $\alpha_i$ is the ray angle relative to the robot. NaN and
out-of-range values are rejected. Therefore this is not automatically the
distance from the robot's outer body to the obstacle.

## Validation status

Diagnostic runs on the current map repeatedly produced statuses such as
`inconsistent_candidate` and `insufficient_score_improvement`. Some candidates
had low absolute residuals but were not consistently better than the input
pose. Closed-loop runs with `/localized_estimate` timed out, so this localizer
is not a validated navigation source. Loosening the gates would hide the
failure mode rather than establish reliable localization.

An example diagnostic launch is:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  localization_scan_stride:=6 \
  localization_max_correction_m:=0.15 \
  localization_max_match_score_m:=0.12 \
  localization_correction_smoothing:=0.25 \
  localization_minimum_consecutive_matches:=3 \
  localization_candidate_consistency_m:=0.05 \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/localizer_diagnostic_eval.csv
```

The default `/odom` navigation path should remain unchanged while this
diagnostic is being studied. A future improvement should add observability and
covariance diagnostics before attempting closed-loop use again.
