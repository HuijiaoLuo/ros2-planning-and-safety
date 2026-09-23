# Map-based Localization

This document describes the optional LiDAR-to-map localizer. It is an
experimental correction layer, not a SLAM system and not part of the frozen
navigation baseline.

## Data flow

```text
/state_prediction + /scan + /map
              ↓
       lidar_localizer
              ↓
       /localized_estimate
```

The localizer assumes a known static occupancy map, a planar odometry estimate
close to the true pose, and a sufficiently observable local scene. It does not
consume Gazebo ground-truth `/odom`.

The repository also contains an independent `localization_backend:=icp`
baseline based on point-to-point Iterative Closest Point (ICP) registration.
It keeps the same input/output topic contract but replaces the ray-casting/grid
objective with deterministic point registration. Its model and limitations are
documented in [`ICP_LOCALIZATION.md`](ICP_LOCALIZATION.md). The ICP backend is
a controlled model comparison, not a claim that the current system provides
SLAM.

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

The matcher evaluates a bounded local grid around the input estimate. The
production configuration holds the fused wheel/IMU heading fixed because the
localizer applies translation only. Its score is

$$
J(x,y,\theta)=
\frac{1}{N}\sum_{i=1}^{N}e_i(x,y,\theta)
+\lambda\left((x-\hat{x}_{\mathrm{odom}})^2
+(y-\hat{y}_{\mathrm{odom}})^2\right).
$$

The corrected position $(\hat{x}_{k},\hat{y}_{k})$ is the nearby candidate
with the smallest score $J$. Only position is corrected; the heading remains
the fused wheel/IMU heading.

`localization_score_mode:=range` is the production baseline. The optional
`endpoint` mode scores measured map-frame endpoints against occupied raster
cells, while `boundary` scores them against continuous occupied/free cell
boundaries. `point_to_line` is an ICP-style diagnostic mode: each endpoint is
associated with its nearest finite map-boundary segment and contributes a
normal-distance residual plus a small penalty when its projection lies
outside that segment. The outer pose search remains the deterministic local
grid, so this is not a full iterative ICP or SLAM system. These modes are
diagnostic alternatives for testing whether the scan geometry contains more
information than the first-hit range score; they are not enabled by default.
A non-zero
`localization_yaw_search_radius_rad` is also diagnostic only: it can reveal
position/yaw coupling in the score, but the matched yaw is not applied to the
published transform.

For `point_to_line`, let $p_i$ be a measured map-frame endpoint, let
$\ell_{j(i)}$ be its nearest finite occupied/free boundary segment, let $q_i$
be the closest point on that segment, and let $n_{j(i)}$ be the segment normal.
The residual is

$$
e_i^{\mathrm{line}}=
\left|n_{j(i)}^{\mathsf T}(p_i-q_i)\right|
+\frac{1}{2}\left\|p_i-q_i\right\|_2.
$$

This is an ICP-style point-to-line correspondence score evaluated inside the
existing deterministic local grid search. It is not an iterative global ICP
solver and does not estimate a map or a loop closure.

The search optimizer is also explicit. `localization_optimizer_mode:=grid`
keeps the exhaustive bounded grid used by the baseline. The optional
`coarse_to_fine` mode is a **multi-resolution grid search**: it evaluates a
coarser global grid first and refines the best
`localization_refine_top_k` basins. This is not the numerical-PDE meaning of
multigrid; it is still a deterministic, derivative-free search-efficiency
variant, not a solution to ambiguous map geometry. Each audit result records the second-best score,
score margin, evaluated candidate count, whether the best candidate lies near
the search boundary, and the decomposition of the selected score into scan
residual and odometry-prior penalty. This decomposition is diagnostic: it
shows whether a candidate wins because it matches the map or because the
regularizer is too weak.

`localization_search_radius_m` is interpreted as a Euclidean search radius.
Candidates outside the corresponding circle are discarded even though the
implementation enumerates the grid using x/y index ranges. This keeps the
search geometry consistent with the later radial correction gate
`localization_max_correction_m`.

## Persistent frame correction and safety gates

The node maintains a planar `map → odom` correction. If the input pose is
$p_{\mathrm{odom}}$, the published pose is

$$
p_{\mathrm{map}}=
T_{\mathrm{map}\leftarrow\mathrm{odom}}
\oplus p_{\mathrm{odom}}.
$$

An accepted candidate is treated as an occasional external position
measurement, not as a correction velocity. The transform is updated gradually
using `localization_correction_smoothing`, but an unchanged candidate is not
reapplied on every scan. A new candidate must differ from the last applied
candidate by at least `localization_minimum_reapplication_change_m`, and the
accumulated translation from the startup transform is bounded by
`localization_max_total_correction_m`. A rejected candidate leaves the
previous transform unchanged. The match must satisfy all of the following:

- enough valid scan points;
- mean range score below `localization_max_match_score_m`;
- candidate displacement below `localization_max_correction_m`;
- accumulated correction below `localization_max_total_correction_m`;
- when enabled, candidate displacement is compatible with the EKF x/y
  covariance through `localization_max_candidate_mahalanobis_sq`;
- candidate centre has the configured `localization_robot_radius_m`
  clearance from occupied cells;
- candidate differs sufficiently from the last applied correction;
- match result is fresh: its age, odometry translation during matching, and
  odometry yaw change remain below the configured asynchronous-result limits;
- consistency over `localization_minimum_consecutive_matches` updates;
- sufficient score improvement over the input pose;
- when enabled, a sufficient score margin over the second-best candidate.

Because the current correction is translational only, consecutive-candidate
and repeated-candidate checks compare the candidate `x,y` displacement. The
matched LiDAR yaw is logged as a diagnostic innovation but does not invalidate
an otherwise coherent position correction.

The free-cell check is deliberately conservative: it rejects candidates
outside the known map or inside the planner's inflated obstacle region. The
clearance radius is deliberately shared with the planner's conservative
footprint model. Useful rejection statuses include
`repeated_correction`, `total_correction_too_large`, and
`candidate_in_unsafe_cell`.

The matcher also records the score margin between the best and second-best
candidate. `localization_minimum_score_margin_m` optionally requires this
margin to exceed a configured value; zero disables the ambiguity gate. A
rejected candidate receives `ambiguous_candidate`. This tests whether a local
minimum is distinguishable, rather than merely whether it improves the prior
pose, and is useful for diagnosing false convergence in repetitive geometry.

A result that arrives after the robot has moved too far is reported as
`stale_match` and leaves the persistent transform unchanged. The corresponding
limits are `localization_max_match_age_s`,
`localization_max_odom_motion_during_match_m`, and
`localization_max_odom_yaw_change_during_match_rad`. These are timing and
motion-consistency safety gates, not optimizer objectives.

The covariance gate is also a safety gate rather than a matching objective.
For candidate displacement $\delta p$ and the EKF position covariance
$P_{xy}$, the diagnostic quantity is

$$
d_M^2 = \delta p^{\mathsf T}P_{xy}^{-1}\delta p.
$$

`localization_max_candidate_mahalanobis_sq:=0.0` disables this check. A
positive value requires a valid EKF x/y covariance and rejects candidates with
larger normalized displacement using the status
`candidate_uncertainty_too_large`. This connects the external correction to
the uncertainty already estimated by the pose EKF without adding the LiDAR residual to
the EKF state update.

The node publishes the applied correction, signed candidate displacement
(`dx`, `dy`), signed applied correction (`dx`, `dy`), score, score improvement,
validity flag, status string, and heading correction on the
`/localization_*` diagnostic topics. The signed topics are important during
closed-loop diagnosis: a scalar correction magnitude cannot reveal whether the
matcher is alternating between opposite directions. When enabled, it
broadcasts the persistent transform on `/tf` and publishes the corrected pose
on `/localized_estimate`.

For a deterministic offline audit of the matcher, set
`localization_diagnostic_output` to a JSONL path. The localizer then records
the matcher configuration, static map, immutable scan/pose inputs, and the
corresponding candidate and gate decision. The audit can be replayed without
starting ROS:

```bash
ros2 launch robotics_sim sim.launch.py \
  localization_diagnostic_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/localizer_audit.jsonl \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/localizer_audit_eval.csv

python tools/replay_lidar_match.py \
  --audit results/localizer_audit.jsonl
```

The replay reruns only the pure LiDAR-to-map search and compares the matched
pose, score, and valid point count with the live process. It also reports the
gate-status counts, accepted candidates, and candidates that were found but not
applied. An exact replay means the matcher geometry is reproducible; a rejected
live status must then be explained by the explicit quality, consistency,
safety, or correction-size gates. A replay mismatch points to a changing
input, configuration, or live process problem rather than to the gate decision
itself.

## Timing architecture

Pose relay and scan matching are deliberately decoupled. The node publishes
the newest available pose at `localization_publish_rate_hz` (default `30 Hz`),
using the last safe persistent correction. The expensive ray-casting search is
started at the lower `localization_match_rate_hz` (default `2 Hz`) in a
separate worker process. The worker receives a snapshot of the pose, scan,
map, and matcher settings and returns only candidate scores; the ROS node
performs the safety gates, transform update, diagnostics, and publication.

This separation is important because the matcher is CPU-bound Python code. A
same-process or same-interpreter worker can delay pose callbacks and feed a
stale estimate back to the controller, which was observed as oscillation and
near-stationary motion. The process-backed matcher keeps the high-rate pose
relay responsive while retaining the conservative acceptance rules above.

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

The localizer is not currently a validated navigation source. In the latest
localized-navigation trial, `/localized_estimate` finished `0.0325 m` from the
goal, while independent `/state_estimate` finished `0.0664 m` and physical
`/odom` finished `0.0762 m` away. Three LiDAR correction events were applied,
with a maximum smoothed correction of `0.0225 m`; the run timed out without
collision or sustained safety recovery. This separates three issues that must
not be conflated: scan-to-map observability, asynchronous match freshness, and
closed-loop physical goal behavior.

The existing temporal replay showed that the direct map-to-odom update and the
equivalent innovation form agree to numerical precision, so the remaining
problem is not transform bookkeeping. Observability analysis found weak or
ambiguous local x/y score surfaces in several scans. The covariance gate,
score-margin gate, and freshness gates are diagnostic safeguards: they do not
change the scan optimizer or turn the localizer into an EKF measurement update.
A candidate is not promoted to `/localized_estimate` navigation merely because
it has a low residual; it must also pass the freshness, motion-consistency,
uncertainty, safety, and consecutive-match gates.

Diagnostic runs on the current map repeatedly produced statuses such as
`inconsistent_candidate`, `insufficient_score_improvement`, and
`correction_too_large`. Some candidates had low absolute residuals but were
not consistently better than the input pose. An earlier process-backed timing
experiment travelled approximately `3.16 m` in `65 s`, with mean EKF NIS
approximately `0.082`, no wheel-yaw rejections, and no collision. It still
timed out with physical final error approximately `0.805 m`; this historical
result showed that separating the high-rate pose relay from matching removed
the earlier same-process starvation failure, but it did not establish
successful localization or navigation. The current representative result is
reported in the validation-status section above. The localizer therefore
remains an experimental navigation source, not a validated replacement for
`/odom`.

The closed-loop margin experiment showed that an ambiguity gate alone is not
enough to certify goal arrival: four low-margin-filtered corrections were
accepted, but the estimated pose still entered the goal tolerance while the
physical pose remained about `0.143 m` away. When navigation consumes
`/localized_estimate`, the path follower therefore requires a current
`/localization_match_valid` signal and several consecutive *new* `accepted`
events on `/localization_match_status` after entering the goal tolerance.
It does not count repeated control-loop reads of one latched boolean. A stale,
repeated, ambiguous, or otherwise rejected match resets this confirmation.
This is a safety gate for the estimated-pose controller; ground truth remains
evaluation-only.

The confirmation sources are configured explicitly. Setting
`require_localization_match_for_goal:=true` requires a fresh valid LiDAR match
and consecutive new `accepted` events. Setting
`require_goal_reference_for_goal:=true` adds an independent pose check using
`goal_reference_topic`, `goal_reference_tolerance`, and
`goal_reference_position_sigma_max_m`. A dual-source test can therefore use
`/state_estimate` for control and `/localized_estimate` for terminal
confirmation, or reverse those roles, without changing the confirmation
contract implicitly when `navigation_pose_topic` changes. This does not use
Gazebo ground truth. If the configured estimates disagree, the robot does not
declare success and the run remains diagnostically unsuccessful.

The reference pose must also report a largest planar 1-sigma uncertainty no
greater than `0.15 m` by default. This is computed from the largest eigenvalue
of the `/state_estimate` x/y covariance ellipse. A pose whose mean is near the
goal but whose covariance is still large is therefore not treated as a
confirmed arrival. The bound is a conservative terminal policy, not a claim
that covariance alone proves physical ground-truth arrival.

The current audit also illustrates why the two diagnostic tools must be read
together. `replay_lidar_match.py` reports matcher events and found three
applied corrections in the latest run. The evaluation trace is sampled at a
higher rate and stores the latest status, so repeated `accepted` or
`repeated_correction` rows are not additional independent matches.
`diagnose_navigation_trace.py` classifies this run as
`estimated_goal_without_physical_goal` with
`localization_correction_was_smoothed_or_limited`. This separates deterministic
matcher geometry from the amount of correction that reaches the control pose.

Each JSONL result also records the individual Boolean gate checks and the
failed `rejection_reasons`. This makes the next diagnostic run able to identify
the failing predicate directly, without reconstructing it from interleaved ROS
logs.

An example diagnostic launch is:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/localized_estimate \
  localization_publish_rate_hz:=30.0 \
  localization_match_rate_hz:=2.0 \
  localization_scan_stride:=6 \
  localization_max_correction_m:=0.15 \
  localization_max_total_correction_m:=0.20 \
  localization_robot_radius_m:=0.41 \
  localization_max_match_score_m:=0.12 \
  localization_correction_smoothing:=0.25 \
  localization_minimum_consecutive_matches:=3 \
  localization_candidate_consistency_m:=0.05 \
  localization_minimum_reapplication_change_m:=0.05 \
  planning_radius_m:=0.41 \
  experiment_timeout_s:=65.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/localizer_diagnostic_eval.csv
```

The default `/odom` navigation path remains unchanged. The command above is a
controlled diagnostic of the process-backed architecture, not a robustness
claim.
Longer closed-loop trials should be interpreted using both the estimated-pose
and physical `/odom` success fields; a controller can enter the estimated
goal tolerance while the physical robot is still displaced.
