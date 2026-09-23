# Method / Tech

This document explains the engineering logic of the project. It answers two questions:

1. What problem is the robot solving?
2. How do the equations, ROS2 nodes, Gazebo model, and validation experiments fit together?

The project is deliberately incremental. At each stage, one new uncertainty is added while the rest of the system remains simple enough to reason about.

### Validation checkpoint

The frozen baseline uses `/odom` for navigation, while the safety layer
uses physical LiDAR measurements and all evaluation results are recorded with
their launch-time configuration. The state-estimation workstream evaluates
wheel/IMU estimates against `/odom` without feeding `/odom` into the estimator.
The map-localization and covariance-aware pose-filter experiments are separate
diagnostic layers; neither corrected pose is yet a validated control input.

The latest controlled evidence supports this boundary. The estimator and
localizers expose covariance, NIS, score-margin, Mahalanobis, freshness, and
applied-correction diagnostics. When external MCL corrections drove
`/state_estimate` and that topic was consumed by the controller, the physical
final error was about `0.108 m`. With the same external fusion enabled but
control using the independent `/state_prediction`, the physical error was
about `0.062 m`, close to the `0.060 m` no-external-fusion ablation. A bounded
final-approach test entered recovery and stopped safely, but MCL still reported
a near-goal pose while `/odom` remained about `0.082 m` from the goal. This
demonstrates a control-pose and local-observability failure, not a validated
replacement for `/odom`.

A separate wheel/state-estimation A/B now isolates the earlier detour drift.
The original fixed-fusion estimator ended with about `0.19 m` position error
in baseline and L-corridor scenes. With `fusion_mode:=ekf`,
`position_mode:=propagated`, `gyro_bias_mode:=fixed`, and
`wheel_yaw_noise_std_rad:=0.20`, seed-0 state error fell to `0.016 m` and
`0.048 m`, and both runs reached the physical goal. This configuration is a
validated estimator/control prior: the follow-up three-seed matrix completed
all `6/6` baseline/L-corridor runs successfully, with mean final state errors
of `0.0160 m` and `0.0435 m`. These parameters are now the source defaults for
the wheel/IMU estimator; `/odom` remains the validated physical reference.

Consequently, parameter changes are treated as controlled experiments rather
than open-ended tuning. The calibrated EKF, safety policy, and high-rate
control contract are frozen. The next model stage compares multi-hypothesis
known-map localization and point-registration models with the same map, scan,
motion prior, controller, and safety policy. Its purpose is to determine
whether the failure is specific to a single local hypothesis or is caused by
map geometry and local observability more generally.

Focused notes are split by topic: [`docs/STATE_ESTIMATION.md`](docs/STATE_ESTIMATION.md)
for transparent wheel/IMU fusion, [`docs/POSE_EKF.md`](docs/POSE_EKF.md) for
covariance-aware pose estimation, and [`docs/LOCALIZATION.md`](docs/LOCALIZATION.md)
for the gated LiDAR-to-map matcher. The independent point-to-point Iterative
Closest Point (ICP) comparison is described in
[`docs/ICP_LOCALIZATION.md`](docs/ICP_LOCALIZATION.md).

## 1. Problem definition

The initial problem is intentionally narrow:

> A differential-drive robot must move toward a known goal and stop safely when a LiDAR detects an obstacle in front of it.

This is not yet a complete autonomous navigation stack. In the current stage:

- the robot has a known goal;
- odometry is provided by simulation;
- LiDAR is the safety sensor;
- the waypoint controller is hand-written;
- the global map and SLAM are not used yet;
- Nav2 is not used yet.

The frozen baseline deliberately uses Gazebo's ideal `/odom` pose so that
planner and safety experiments can be interpreted independently. The estimator
publishes a wheel/IMU-only `/state_prediction` and a separate
`/state_estimate` that may consume external map-position events. The current
localization experiments keep the high-rate controller on `/state_prediction`;
the corrected estimate is evaluated and fused diagnostically until its global
validity is established.

The important engineering chain is:

$$
\text{physical model} \rightarrow \text{measurement} \rightarrow \text{state/control model} \rightarrow \text{decision} \rightarrow \text{validation}
$$

ROS2 is the communication architecture. Gazebo is the physics and sensor simulation. The core question is how imperfect measurements should be translated into safe motion commands.

## 2. Current repository layers

~~~
Python planning core
    ├── BFS
    ├── Dijkstra
    ├── Greedy Best-First Search
    ├── A*
    └── DFS with backtracking

C++ planning core
    └── same algorithms + benchmark hooks for future HPC work

ROS2 control layer
    ├── path_follower
    ├── waypoint_controller baseline
    ├── heading_estimator (state estimation)
    └── safety_supervisor

ROS2 planning layer
    ├── static_map_publisher
    └── global_planner → /plan

Gazebo simulation layer
    ├── differential-drive robot
    ├── LiDAR
    ├── obstacle
    └── ROS-Gazebo bridge
~~~

The grid-search planners remain a standalone benchmark layer, while the ROS2
global planner now contains the first ROS adapter for the same 4-neighbor A*
logic. The next cleanup step will extract one shared implementation so that the
benchmark and ROS2 node cannot diverge.

## 3. Differential-drive motion model

The robot is represented by the planar unicycle model:

$$
\dot{x} = v\cos\theta
$$

$$
\dot{y} = v\sin\theta
$$

$$
\dot{\theta} = \omega
$$

The controller command is:

$$
u = (v, \omega)
$$

The physical wheel speeds are related to the body command by:

$$
v_L = v - \frac{L}{2}\omega, \qquad v_R = v + \frac{L}{2}\omega
$$

where $L$ is the distance between the wheel contact points. Gazebo's DiffDrive system uses the two wheel joints, wheel radius, and wheel separation to apply this model to the simulated robot.

## 4. ROS2 computation graph

~~~
                 /odom (validated baseline) or /state_prediction (controlled estimate)
                           │
                           ▼
                 ┌────────────────────┐
                 │   path_follower    │
                 └─────────┬──────────┘
                           │
                    /cmd_vel_raw
                           │
                           ▼
 /scan ───────────► ┌────────────────────┐
                    │ safety_supervisor  │
                    └─────────┬──────────┘
                              │
                         /cmd_vel
                              │
                              ▼
                    ros_gz_bridge → Gazebo
~~~

| Topic | Message | Direction | Meaning |
|---|---|---|---|
| /odom | nav_msgs/msg/Odometry | Gazebo → ROS2 | Ground-truth pose for the ideal MVP |
| /wheel_odom | nav_msgs/msg/Odometry | Gazebo → ROS2 | DiffDrive wheel odometry for slip comparison |
| /imu | sensor_msgs/msg/Imu | Gazebo → ROS2 | Angular velocity used by the heading estimator |
| /state_prediction | nav_msgs/msg/Odometry | Estimator → localizer/controller | Wheel/IMU-only propagated motion prior |
| /state_estimate | nav_msgs/msg/Odometry | Estimator → diagnostics/optional fusion | State estimate after optional external map-position updates |
| /scan | sensor_msgs/msg/LaserScan | Gazebo → ROS2 | LiDAR range measurements |
| /localized_estimate | nav_msgs/msg/Odometry | Localizer → navigation | Opt-in gated LiDAR-map position correction |
| /localization_candidate | nav_msgs/msg/Odometry | Map localizer → optional EKF | One event per accepted map-position candidate |
| /localization_correction_m | std_msgs/msg/Float64 | Localizer → diagnostics | Applied correction magnitude in metres |
| /localization_candidate_correction_m | std_msgs/msg/Float64 | Localizer → diagnostics | Raw candidate displacement before smoothing/gating |
| /localization_candidate_dx_m | std_msgs/msg/Float64 | Localizer → diagnostics | Signed candidate x correction in the map frame |
| /localization_candidate_dy_m | std_msgs/msg/Float64 | Localizer → diagnostics | Signed candidate y correction in the map frame |
| /localization_applied_dx_m | std_msgs/msg/Float64 | Localizer → diagnostics | Signed x correction actually applied after smoothing |
| /localization_applied_dy_m | std_msgs/msg/Float64 | Localizer → diagnostics | Signed y correction actually applied after smoothing |
| /localization_match_score_m | std_msgs/msg/Float64 | Localizer → diagnostics | Raw mean LiDAR-map range residual score |
| /localization_heading_correction_rad | std_msgs/msg/Float64 | Localizer → diagnostics | Applied heading correction in radians |
| /localization_score_improvement_m | std_msgs/msg/Float64 | Localizer → diagnostics | Input-pose score minus candidate score |
| /localization_match_valid | std_msgs/msg/Bool | Localizer → diagnostics | Whether the latest match passed the safety gates |
| /localization_match_status | std_msgs/msg/String | Localizer → diagnostics | Reason for accepting or rejecting the latest match |
| /path_follower_goal_event | std_msgs/msg/String | Controller → evaluation logger | Timestamped goal-tolerance and terminal-latch events for temporal audits |
| /cmd_vel_raw | geometry_msgs/msg/Twist | Controller → safety layer | Unchecked motion request |
| /cmd_vel | geometry_msgs/msg/Twist | Safety layer → Gazebo | Command allowed to reach robot |

The distinction between /cmd_vel_raw and /cmd_vel is important. It makes the safety layer independently testable and gives the system a clear enforcement point: Gazebo never receives the controller's command directly.

The simulation deliberately exposes several pose sources. The ideal-navigation
MVP uses `/odom`, generated from Gazebo's true model pose, so planning and
control are not invalidated by wheel slip before the basic loop is verified.
The DiffDrive plugin publishes `/wheel_odom` separately. The estimator adds an
IMU and transparent heading fusion; `/state_prediction` is the independent
motion prior, while `/state_estimate` may be corrected by an external map
observation. The latter is not a validated high-rate control source.

The simulated actuator also has explicit linear and angular velocity and
acceleration limits. These keep the ideal model physically stable when the
controller changes commands; they are separate from the LiDAR safety
supervisor and do not replace its stopping envelope.

The odometry and LiDAR subscriptions use ROS2's sensor-data QoS profile. This
matters because Gazebo sensor bridges commonly publish with best-effort,
volatile QoS; a default reliable subscription may be incompatible and receive
no callbacks even when the topic appears in the graph.

## 4.1 State estimation: wheel odometry plus IMU heading

Gazebo publishes an IMU on `/imu`. The estimator consumes only the IMU angular
velocity, not its orientation field. This prevents the simulated perfect
orientation from becoming a hidden ground-truth input.

The transparent heading estimator first propagates its heading with the IMU
angular rate:

$$
\theta_k^- = \mathrm{wrap}\left(\theta_{k-1}^{\mathrm{fused}} + \omega_{z,k}\Delta t_k\right)
$$

When a wheel-yaw sample is available, it then applies a small complementary
correction:

$$
\theta_k^{\mathrm{fused}} = \mathrm{wrap}\left(\theta_k^- + \lambda\,\mathrm{wrap}\left(\theta_k^{\mathrm{wheel}} - \theta_k^-\right)\right)
$$

With no wheel update, the propagated value $\theta_k^-$ remains active. The
ROS callbacks are asynchronous; these equations express the equivalent
prediction-then-correction sequence, while the implementation applies each
operation when its corresponding message arrives.

The launch parameter `wheel_weight` is the transparent tuning parameter
$\lambda$, with default $\lambda = 0.02$. It gives the gyro short-term
responsiveness while allowing wheel yaw to limit long-term drift; it is not a
covariance-derived EKF gain. The value is recorded in estimator metrics for
reproducible fusion-weight sweeps. The first estimator publishes
wheel-odometry `x` and `y` together with the fused heading on `/state_estimate`.

The baseline-compatible launch leaves all navigation nodes on `/odom`:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom
```

The validated diagnostic keeps navigation on `/odom` while the estimator runs
in parallel:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/heading_diagnostic.csv
```

Switching navigation to `/state_estimate` remains an exploratory experiment,
not a validated full-pose navigation mode. The adaptive heading mode keeps the
state

$$
\mathbf{x}_k=(\theta_k,\,b_{g,k})^{\mathsf T}
$$

The correction is represented as a persistent planar transform
$T_{\mathrm{map}\leftarrow\mathrm{odom}}$. If the current odometry pose is
$p_{\mathrm{odom}}$, the published localized pose is

$$
p_{\mathrm{map}} = T_{\mathrm{map}\leftarrow\mathrm{odom}} \oplus p_{\mathrm{odom}}.
$$

and a covariance-derived wheel-yaw gain:

$$
K_k = \frac{P_k^-}{P_k^- + R_{\mathrm{wheel}}}
$$

The adaptive mode estimates gyro bias and publishes its gain and innovation
as diagnostics. It is still only a heading filter: wheel-odometry `x` and `y`
remain uncorrected, so it is not yet a validated full-pose localization
system.

### Innovation-adaptive wheel-yaw noise

The fixed wheel-yaw variance can be replaced by a bounded innovation-based
estimate. The wrapped innovation is

$$
\nu_{k} = \mathrm{wrap}(\theta_{k}^{\mathrm{wheel}} - \theta_{k}^{-})
$$

Its exponentially smoothed squared magnitude is

$$
\widehat{S}_{k} = (1-\beta)\widehat{S}_{k-1} + \beta\nu_{k}^{2}
$$

The wheel measurement variance is then updated as

$$
R_{\mathrm{wheel},k} = \min\left(R_{\max},\max\left(R_{\min},\widehat{S}_{k}-P_{k}^{-}\right)\right).
$$

Here $\nu_{k}$ is measured in radians, while $\widehat{S}_{k}$, $P_{k}^{-}$,
and $R_{\mathrm{wheel},k}$ are variances in $\mathrm{rad}^{2}$. The launch
parameters expose standard-deviation bounds, so
$R_{\min}=\sigma_{\min}^{2}$ and $R_{\max}=\sigma_{\max}^{2}$. The
adaptation rate is `wheel_noise_adaptation_rate`. The corresponding gain
remains

$$
K_{k} = \frac{P_{k}^{-}}{P_{k}^{-} + R_{\mathrm{wheel},k}}
$$

This makes the measurement model less trusting when recent wheel-yaw
innovations are large. It is a bounded quality heuristic, not a direct
measurement of sensor noise or a full adaptive EKF; persistent wheel and gyro
biases cannot be completely separated with only these two heading sources.
The current standard-deviation estimate is published on
`/wheel_yaw_noise_std_estimate` and included in estimator diagnostics.

### Propagated position mode

The estimator supports a `position_mode:=propagated` option. Instead of copying
wheel-odometry position increments, it integrates the wheel-odometry forward
speed using the fused heading. With wheel-slip ratio $s$:

$$
\Delta s_k = (1-s)v_{x,k}\Delta t_k
$$

The midpoint heading is

$$
\theta_{\mathrm{mid},k} = \mathrm{wrap}(\hat{\theta}_{k-1} + \frac{1}{2}\mathrm{wrap}(\hat{\theta}_{k} - \hat{\theta}_{k-1}))
$$

and the propagated position is

$$
\hat{x}_{k} = \hat{x}_{k-1} + \Delta s_{k}\cos(\theta_{\mathrm{mid},k})
$$

$$
\hat{y}_{k} = \hat{y}_{k-1} + \Delta s_{k}\sin(\theta_{\mathrm{mid},k})
$$

`position_mode:=wheel_pose` remains the backward-compatible diagnostic mode.
The propagated mode initializes from the first wheel-odometry sample and does
not consume Gazebo `/odom` for estimation.

### Local LiDAR--map position correction

The odometry-only navigation experiment exposes a false-goal-completion case:
`/state_estimate` can enter the goal tolerance while physical `/odom` has not.
The optional `lidar_localizer` provides a small absolute-position
correction using the known static map:

```text
/state_prediction + /scan + /map → /localized_estimate
```

For a valid range return $r_i$ at scan angle $\alpha_i$, a candidate pose
predicts the first occupied-cell range $\hat{r}_i$ by raycasting the static
map. Its measurement residual is

$$
e_i(x,y,\theta) = |r_i - \hat{r}_i(x,y,\theta;\mathcal{M})|
$$

The corresponding candidate endpoint in the map frame is

$$
p_{i,x}^{\mathrm{map}} = x + r_i\cos(\theta + \alpha_i)
$$

$$
p_{i,y}^{\mathrm{map}} = y + r_i\sin(\theta + \alpha_i)
$$

The local matcher searches candidate $\left(x,y\right)$ values near the odometry pose
while holding the fused wheel/IMU heading fixed. For a candidate position and
heading, the position score is

$$
J(x,y,\theta) = \frac{1}{N}\sum_{i=1}^{N} e_i(x,y,\theta) + \lambda((x-\hat{x}_{\mathrm{odom}})^2 + (y-\hat{y}_{\mathrm{odom}})^2)
$$

The corrected position is denoted by

$$
(\hat{x}_{k},\hat{y}_{k})
$$

and is the nearby candidate with the smallest score $J$.
The node is therefore a transparent local scan matcher, not a complete SLAM
or a full covariance-aware localization filter. It can optionally use the
EKF covariance as a post-match plausibility gate. A non-zero
`localization_yaw_search_radius_rad` can be enabled for diagnosis, but it must
not be interpreted as an applied heading correction: the localizer publishes
the fused heading and updates only the translation.

The production `localization_score_mode:=range` compares measured and
raycast first-hit ranges. The optional `endpoint` and `boundary` modes test
two geometric alternatives offline: measured endpoints against occupied cell
rectangles, and measured endpoints against continuous occupied/free cell
boundaries. `point_to_line` is an ICP-style alternative: each endpoint is
associated with its nearest map boundary segment and scored by the normal
distance to that segment, with a finite-segment association penalty. It uses
the same bounded pose search, so it is local registration rather than full
iterative ICP or SLAM. These alternatives remain diagnostic because sparse
scans and rasterized maps can still produce nearly tied x/y candidates.

For a measured endpoint $p_i$, let $\ell_{j(i)}$ be the nearest finite map
boundary segment, let $q_i$ be the closest point on that segment, and let
$n_{j(i)}$ be its occupied-to-free normal. The point-to-line residual used by
the implementation is

$$
e_i^{\mathrm{line}} = \left|n_{j(i)}^{\mathsf T}(p_i-q_i)\right| + \frac{1}{2}\left\|p_i-q_i\right\|_2.
$$

The first term measures normal alignment; the second discourages an endpoint
from matching the supporting infinite line outside the finite map segment.
The pose search still minimizes the same regularized objective $J$ over a
bounded local grid.

The optimizer is a bounded, derivative-free search. `grid` exhaustively
evaluates the configured circular x/y grid and is the reference behavior.
`coarse_to_fine` is a **multi-resolution grid search**: it first evaluates a
coarser global grid, then refines the best few basins at the nominal
resolution. This is not the numerical-PDE meaning of multigrid. Neither mode
assumes differentiability: the occupancy-grid objective contains
discontinuities from ray hits, invalid returns, and residual clipping. Audit
records expose the second-best score, score margin, candidate distance from the
prior, search-boundary flag, and score decomposition, so an apparently low
score is not confused with a unique physical match. The selected objective is

$$
J = e_{\mathrm{scan}} + w_{\mathrm{prior}}\left(\Delta x^2 + \Delta y^2\right) + w_{\mathrm{yaw}}\Delta\theta^2.
$$

where $e_{\mathrm{scan}}$ is the mean LiDAR/map residual, and
$\left(\Delta x, \Delta y, \Delta\theta\right)$ is the candidate displacement relative
to the input pose. The final two terms are regularizers, not additional sensor
measurements. The audit records both $J$ and $e_{\mathrm{scan}}$, allowing the
prior penalty to be inspected before changing its weight.

Because a local scan matcher can select a plausible but incorrect nearby pose,
the ROS node does not forward every raw match to the controller. It accepts a
candidate only when it has enough valid returns, its mean range score is below
`localization_max_match_score_m`, and its displacement from the input pose is
below `localization_max_correction_m`. An accepted candidate is treated as an
occasional external position measurement rather than a correction velocity. It
estimates a new map-to-odom transform, which is interpolated using
`localization_correction_smoothing`, and must remain consistent for
`localization_minimum_consecutive_matches` updates. The same candidate is not
reapplied indefinitely: the candidate must change by at least
`localization_minimum_reapplication_change_m`, and the accumulated translation
from the startup transform must remain below
`localization_max_total_correction_m`. Candidates outside the known map or
inside the planner's inflated obstacle region are rejected before they can
change the transform. The localizer uses
`localization_robot_radius_m` so this gate follows the same conservative
robot-centre clearance model as the A* planner. The matched LiDAR yaw is
retained as a diagnostic innovation; it is not applied to the map-to-odom
transform. Candidate consistency and repeated-correction checks therefore use
only the candidate translation; scan-to-scan yaw variation does not block a
coherent position update.
The covariance-aware pose EKF also publishes an x/y covariance with
`/state_estimate`. The
optional `localization_max_candidate_mahalanobis_sq` gate uses that covariance
after the matcher has selected a candidate. For displacement $\delta p$ and
map-frame covariance $P_{xy}$, the normalized correction is

$$
d_M^2 = \delta p^{\mathsf T}P_{xy}^{-1}\delta p.
$$

The gate is disabled when its launch value is zero. When enabled, a missing or
singular covariance rejects the candidate, and a value above the configured
bound receives `candidate_uncertainty_too_large`. This is a plausibility check,
not an additional optimizer penalty and not a LiDAR measurement update inside
the EKF.
Rejected matches leave the previous transform unchanged. The applied correction
magnitude is published on `/localization_correction_m`. The signed candidate
displacement is published on `/localization_candidate_dx_m` and
`/localization_candidate_dy_m`, while the signed smoothed correction is
published on `/localization_applied_dx_m` and
`/localization_applied_dy_m`. The raw score and scalar candidate magnitude
are published on `/localization_match_score_m` and
`/localization_candidate_correction_m`; and `/localization_match_valid`
reports whether the latest match passed the gates. Useful rejection statuses
include `repeated_correction`, `total_correction_too_large`, and
`candidate_in_unsafe_cell`. The candidate must also improve the mean
scan residual relative to the input pose by at least
`localization_minimum_score_improvement_m`. This prevents a different
nearby local minimum from being accepted merely because its absolute score is
below the threshold.

The matcher also reports the gap between the best and second-best candidate.
The optional `localization_minimum_score_margin_m` gate rejects a result as
`ambiguous_candidate` when that gap is too small; zero disables the gate. This
distinguishes a candidate that improves the prior from one that is uniquely
supported by the local scan geometry.

The matcher is asynchronous, so a numerically good result can still be stale
when it returns. The node therefore also rejects a result when its age exceeds
`localization_max_match_age_s`, when live odometry has translated more than
`localization_max_odom_motion_during_match_m` during the worker computation, or
when live odometry has changed yaw by more than
`localization_max_odom_yaw_change_during_match_rad`. Such a result receives
the `stale_match` status and cannot update the persistent transform. These
gates express temporal consistency of the scan/pose pair; they are separate
from the scan optimizer and do not alter the candidate score.

When enabled, the node broadcasts the persistent transform as the standard
`map → odom` TF. `/localized_estimate` is the corresponding pose in the map
frame for compatibility with the current planner interface.

The localizer uses a two-rate architecture. The ROS node relays the newest
corrected pose at `localization_publish_rate_hz` (default `30 Hz`), while a
snapshot-based matcher runs at `localization_match_rate_hz` (default `2 Hz`)
in a separate process. Matching is CPU-bound Python ray casting, so keeping it
outside the ROS callback process prevents stale pose feedback from starving
the follower. Only the candidate score is returned by the worker; acceptance,
transform smoothing, diagnostics, and pose publication remain in the ROS node.
The last accepted transform is reused between completed matching jobs, so a
temporary matcher delay does not create a second pose source or a sudden
correction.

The corrected topic is opt-in through
`navigation_pose_topic:=/localized_estimate`; the `/odom` baseline and the
diagnostic `/state_estimate` path remain unchanged by default. Physical success
must still be checked with `/odom`, because entering the estimated-pose goal
tolerance alone does not prove that the robot reached the goal.

The current validation keeps `/localized_estimate` experimental. The validated
`/odom` baseline remains the physical reference. In the final controlled
comparison, direct corrected-state control produced about `0.108 m` physical
error; separating control onto `/state_prediction` reduced it to about
`0.062 m`. A bounded final-approach run entered recovery and stopped safely,
but the localized estimate remained within about `0.029 m` while `/odom` was
about `0.082 m` from the goal. The result is evidence that local map ambiguity,
motion-model drift, and control-pose selection must be treated separately; it
is not evidence of localization robustness or successful closed-loop
navigation.

### Optional coupled map-position update

The known-map MCL backend now exposes one event topic,
`/localization_candidate`, per accepted scan update. This topic is distinct
from the repeated `/localized_estimate` navigation stream, so the same
candidate cannot be fused at the localizer publish rate. With
`external_position_fusion:=true` and `fusion_mode:=ekf`, the heading estimator
passes the candidate position and its covariance to the pose EKF. The EKF
performs a two-dimensional NIS-gated measurement update using the full state
covariance, so existing position--yaw--bias cross-covariances provide the
coupling. The default is disabled and the previous estimator baseline is
unchanged. The mathematical model and validation plan are documented in
[`docs/COUPLED_ESTIMATION.md`](docs/COUPLED_ESTIMATION.md).

External map observations are now consumed at their source timestamp. The
candidate header carries t_m, while the newest wheel/IMU state may already
be at t_k. The estimator records the timestamped sensor event stream,
inserts the candidate at t_m, applies the position update there, and
replays the subsequent motion events.

This avoids treating a delayed LiDAR result as a measurement of the current
pose. The estimator publishes the physical observation age and whether
history replay was used; the estimation report records both quantities.
This is deterministic delayed-observation handling, not an additional
correction parameter. It still assumes aligned map and odometry frames and
does not solve global map aliasing.

The read-only trace intentionally samples the latest diagnostic state at the
logger rate. Therefore repeated `accepted` or `repeated_correction` rows in a
CSV trace are not independent matcher events. `replay_lidar_match.py` is the
event-level audit, while `diagnose_navigation_trace.py` classifies the closed
loop outcome. This distinction prevents status persistence from being
mistaken for additional LiDAR evidence.

This checkpoint keeps three different facts separate. The
`ground_truth_goal_reached_any_time` field records whether /odom ever entered
the goal tolerance; it is not a terminal completion result. The
`ground_truth_final_within_goal_tolerance` field describes the final complete
trace sample. The top-level `success` field is true only when the controller
publishes `goal_reached_latched`, the evaluation terminates with
`goal_reached`, and the final complete `/odom` sample remains within the goal
tolerance. This prevents both historical tolerance entries and estimated-pose
latches without physical agreement from being reported as successful.

### Covariance-aware pose EKF

The adaptive heading filter estimates only heading and gyro bias. The
covariance-aware pose EKF extends the state to planar pose, gyro bias, and a
slowly varying wheel-yaw bias:

$$
\mathbf{x}_{k} = [x_{k},y_{k},\theta_{k},b_{g,k},b_{w,k}]^{\mathsf{T}}
$$

Here $x_k$ and $y_k$ are the estimated planar position in metres,
$\theta_k$ is the estimated heading in radians, $b_{g,k}$ is the estimated
IMU gyro zero-rate offset in rad/s, and $b_{w,k}$ is the estimated wheel-yaw
offset in radians. Both $b_g$ and $b_w$ are internal filter states; neither is
directly read as a sensor value. With only gyro rate and wheel yaw, a constant
$b_w$ is not absolutely observable because the same sequence can be explained
by a different initial heading. An external absolute heading or map constraint
is needed to identify its physical value.

The EKF predicts with wheel forward speed and bias-corrected gyro rate:

$$
\omega_{k} = \omega_{z,k} - b_{g,k}
$$

Its covariance follows the linearized process model:

$$
P_{k+1}^{-} = F_{k}P_{k}F_{k}^{\mathsf{T}} + Q_{k}
$$

The process covariance $Q_k$ includes configured gyro-rate noise, wheel-speed
noise, gyro-bias random walk, wheel-yaw-bias random walk, and optional
uncertainty in the configured wheel-slip fraction. For a slip-ratio standard
deviation $\sigma_s$, the
additional position sensitivity is

$$
G_{s,k}=\left(-v_{x,k}\Delta t_k\cos(\theta_{\mathrm{mid},k}),\,-v_{x,k}\Delta t_k\sin(\theta_{\mathrm{mid},k}),\,0,\,0,\,0\right)^{\mathsf T}.
$$

and the corresponding contribution is

$$
Q_{s,k}=\sigma_s^2G_{s,k}G_{s,k}^{\mathsf T}.
$$

`wheel_slip_ratio` is the configured mean travel loss;
`wheel_slip_noise_std` is the uncertainty around that mean. It increases
reported x/y covariance without adding an unobservable slip state. Wheel yaw
is the scalar measurement of physical heading plus wheel-yaw bias:

$$
\theta_k^{\mathrm{wheel}}=\theta_k+b_{w,k}+v_k,
\qquad v_k\sim\mathcal{N}(0,R_{\mathrm{wheel}}).
$$

$$
\nu_{k} = \mathrm{wrap}(\theta_{k}^{\mathrm{wheel}} - \theta_k^- - b_{w,k}^-)
$$

$$
S_{k} = H P_k^- H^{\mathsf T} + R_{\mathrm{wheel}}, \qquad H=[0\;0\;1\;0\;1]
$$

$$
K_{k} = P_{k}^{-}H^{\mathsf{T}}S_{k}^{-1}
$$

The predicted wheel-yaw measurement is $\theta_k^-+b_{w,k}^-$. The filter
also reports the normalized innovation squared:

$$
\mathrm{NIS}_{k} = \frac{\nu_{k}^{2}}{S_{k}}
$$

When `nis_gate_threshold` is positive, a wheel-yaw update with NIS above that
threshold is rejected. The default `9.0` is a transparent approximately
three-sigma gate for this scalar measurement. It is an integrity check, not a
replacement for a complete sensor fault model.

`fusion_mode:=ekf` publishes the EKF pose through the existing
`/state_estimate` interface and places the x/y/yaw covariance in the standard
`nav_msgs/Odometry` pose covariance field. Additional diagnostics are
`/heading_fusion_nis`, `/heading_measurement_accepted`, and
`/wheel_yaw_bias_estimate`. The estimator uses `/wheel_odom` and IMU angular
velocity only; `/odom` remains evaluation-only.

The gyro-bias state has an explicit operating mode. In
`gyro_bias_mode:=estimated`, $b_{g,k}$ is an EKF state with a random-walk
process model, and wheel-yaw innovations can update it through the state
covariance. In `gyro_bias_mode:=fixed`, $b_{g,k}$ is held at the pre-calibrated
value `initial_gyro_bias_rad_s`; its wheel-update gain is forced to zero and
its covariance row and column are cleared. Therefore

$$
\omega_k = \omega_{z,k} - b_{g,k}
$$

always subtracts a modelled zero-rate offset, not a second sensor reading.
Here $\omega_{z,k}$ is the raw IMU yaw-rate measurement in rad/s, while
$b_{g,k}$ is either the current EKF estimate or the fixed calibration constant
in rad/s. The simulation input `imu_gyro_bias_rad_s` and the estimator input
`initial_gyro_bias_rad_s` are intentionally separate: the first creates the
sensor error, and the second tells the estimator what calibration is available.

The two modes answer different diagnostic questions. The estimated mode asks
whether the available measurements can jointly identify pose and bias. The
fixed mode asks whether wheel-yaw fusion behaves correctly when gyro bias has
already been calibrated. It is especially useful here because a constant
wheel-yaw error can otherwise be mistaken for a gyro-bias error.

The first EKF smoke test keeps navigation on `/odom`:

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

The first recorded smoke row completed the `/odom` baseline with
`mean NIS≈0.079`, `max NIS≈1.275`, and zero rejected wheel-yaw updates. The
EKF reported approximately `0.0735 m` position RMSE and `0.071 rad` heading
RMSE, while mean x/y covariance remained about `0.250 m²`. This validates the
diagnostic plumbing, not covariance calibration or navigation replacement.
The next comparisons should vary one uncertainty at a time: zero slip versus
`wheel_slip_ratio:=0.10`, then seeded gyro bias/noise. Closed-loop navigation
with `/state_estimate` remains blocked until the physical `/odom` result and
the EKF covariance/innovation diagnostics are understood.

An explicit fixed-bias ablation subsequently used
`gyro_bias_mode:=fixed`, `initial_gyro_bias_rad_s:=0.0`, and zero gyro-bias
random walk. In that diagnostic run, heading RMSE decreased from approximately
`0.0676 rad` to `0.0332 rad`, and the estimator's final position error was
approximately `0.057 m`. The run kept navigation on `/odom`, so these are
estimator-diagnostic improvements rather than evidence that the estimated pose
has already replaced the navigation pose in closed loop. The result supports
the interpretation that the earlier estimated-bias mode was allowing wheel-yaw
error to leak into $b_g$.

The corresponding reproducible fixed-bias diagnostic configuration is:

```bash
ros2 launch robotics_sim sim.launch.py \
  navigation_pose_topic:=/odom \
  fusion_mode:=ekf \
  gyro_bias_mode:=fixed \
  initial_gyro_bias_rad_s:=0.0 \
  gyro_bias_random_walk_std_rad_s2:=0.0 \
  position_mode:=propagated \
  wheel_slip_ratio:=0.0 \
  wheel_slip_noise_std:=0.0 \
  experiment_timeout_s:=120.0
```

## 5. Waypoint controller

The controller reads the current pose from `/odom` by default, or from
`/state_estimate` when the estimated-pose launch override is enabled, and uses a fixed
goal.

Distance to goal:

$$
e_d = \sqrt{(x_g-x)^2 + (y_g-y)^2}
$$

Desired heading:

$$
\theta_g = \mathrm{atan2}(y_g-y,\,x_g-x)
$$

Wrapped heading error:

$$
e_\theta = \mathrm{wrap}(\theta_g-\theta)
$$

The proportional command is:

$$
v = \mathrm{clip}(K_d e_d,\,0,\,v_{\max})
$$

$$
\omega = \mathrm{clip}(K_\theta e_\theta,\,-\omega_{\max},\,\omega_{\max})
$$

There is one practical rule in addition to the equations:

~~~
large heading error → rotate in place
small heading error  → drive and rotate simultaneously
goal within tolerance → publish zero velocity
~~~

The launch parameter `goal_tolerance` is shared by the path follower and the
evaluation logger so that the controller's stopping rule and the reported
estimated-pose goal event use the same distance. Once the follower reaches the
current path endpoint, it latches zero velocity across ordinary replans with
the same endpoint. A materially different endpoint clears the latch. This
prevents estimator noise or repeated path publication from restarting terminal
motion, but it does not correct the estimator's physical position error.

The terminal confirmation contract is explicit rather than inferred from the
control-pose topic. Set `require_localization_match_for_goal:=true` to require
that `/localization_match_valid` is true and that five consecutive *new*
`accepted` messages on `/localization_match_status` arrive after the pose
enters the goal tolerance. The controller counts matcher events, not timer
ticks: a latched boolean from one old scan cannot be sampled repeatedly and
mistaken for five independent matches. Any invalid, stale, ambiguous, or
repeated LiDAR match resets the event streak and publishes a stop command
without declaring success.

For the MCL event stream, `require_timestamped_localization_evidence:=true`
adds a stronger source-time contract. The controller subscribes to
`/localization_candidate`, counts only new accepted `Odometry` events received
after entering the goal tolerance, and checks their monotonic receipt age.
The candidate header stamp is retained for correlation, but is not subtracted
from the controller clock because simulator and system clocks may use
different epochs. A fallback `/localized_estimate` pose or an old accepted
candidate therefore cannot independently prove current physical arrival.

Set `require_goal_reference_for_goal:=true` to add a second pose source. The
topic is selected by `goal_reference_topic` (for example, `/state_estimate`),
with `goal_reference_tolerance` and
`goal_reference_position_sigma_max_m` defining its distance and uncertainty
checks. This makes A/B experiments explicit: changing `navigation_pose_topic`
does not silently enable or disable terminal confirmation. A dual-source
localized-navigation test can therefore use `/state_estimate` for control and
`/localized_estimate` as the independent confirmation source, while the
controller still does not consume Gazebo ground truth.

These gates are intentionally conservative: a LiDAR match can be internally
valid yet correspond to a local map minimum, so disagreement between the two
configured sources blocks the terminal latch and keeps the run diagnostically
unsuccessful. They do not claim that estimated localization can certify
physical ground-truth arrival.

The terminal check also requires the largest planar 1-sigma uncertainty of
`/state_estimate` to be at most `0.15 m` by default. For the x/y covariance
matrix `P_xy`, the reported uncertainty is

$$
\sigma_{xy}=\sqrt{\lambda_{\max}(P_{xy})}.
$$

This prevents a mean estimate from satisfying the goal distance while its
uncertainty ellipse is still much larger than the goal tolerance. The bound is
a configurable safety policy and must be interpreted together with covariance
calibration; it is not a ground-truth measurement.

The confirmation gate also requires the navigation pose's planar speed to be
no greater than `goal_confirmation_max_speed_m_s` (default `0.05 m/s`) and
the consumed navigation pose header to be no older than
`goal_confirmation_max_pose_age_s` (default `0.15 s`). These are temporal and
motion-quality gates, not estimator accuracy claims: a pose that is geometrically
near the goal but still moving or outside the source-time bound cannot latch.

The confirmation wait is optionally bounded by the launch parameter
`goal_confirmation_timeout_s`. When it is positive, the timer starts when the
controller first enters the goal tolerance. If the required fresh LiDAR events,
independent pose, or independent covariance check do not become valid before
the deadline, the follower publishes `event=goal_confirmation_timeout` and
transitions from `CONFIRMING` to a bounded low-speed `FINAL_APPROACH` state.
It does not set `goal_reached`. The controller must leave the goal tolerance,
then re-enter it, before a new confirmation window can start. If the target is
clearly lost, the state returns to `APPROACHING`; only successful confirmation
transitions to `GOAL_LATCHED`. The evaluation trace records
`goal_state`, `confirmation_start_time_s`, `confirmation_duration_s`,
`confirmation_timeout_count`, and `final_approach_reentry_count`. A value of
`0.0` preserves the unbounded wait used by legacy comparison runs.
Every state transition is also emitted as a `goal_state_changed` diagnostic
event, so the final reported state is not inferred from a stale confirmation
event.

For a bounded validation run, `goal_confirmation_max_attempts` can limit the
number of timeout/recovery cycles. Once the limit is reached, the controller
enters `GOAL_UNCONFIRMED`, publishes `goal_confirmation_failed`, and commands
zero velocity until a new path arrives. This is an explicit safe failure: it
does not count as `goal_reached` and prevents a confirmation failure from
turning into an unbounded recovery loop.

During `FINAL_APPROACH`, the controller also preserves the target bearing at
the timeout transition. Recomputing `atan2(goal - pose)` from a residual that
is already only a few centimetres can make the bearing dominated by map-match
noise and cause repeated saturated in-place turns. The fixed bearing is a
control-stability measure for this recovery state; it is not an additional
localization measurement and does not by itself authorize goal completion.

This is intentionally a direct-goal baseline, not the default controller anymore. It is useful for comparing direct waypoint tracking against planned-path tracking.

## 6. Global planner and path message

The ROS2 global planner subscribes to:

~~~
/map   nav_msgs/msg/OccupancyGrid
/odom  nav_msgs/msg/Odometry
~~~

It performs four transformations:

1. Convert the odometry position from meters to a grid cell.
2. Convert the fixed goal from meters to a grid cell.
3. Inflate occupied cells by the configured robot radius.
4. Run 4-neighbor A* and convert the resulting cells back to meter coordinates.

The output is:

~~~
/plan   nav_msgs/msg/Path
~~~

Each pose in the path is the center of one free grid cell. The path is geometry,
not a velocity command. The path follower reads this path and publishes
`/cmd_vel_raw`.

The path follower advances through the ordered path prefix until it reaches a
`0.10 m` lookahead distance, computes the heading error from the current
odometry, and applies the same unicycle-style control idea as the direct
waypoint baseline. It does not select an arbitrary globally nearest path pose:
that can jump across a detour when a later pose is geometrically closer than
the safe intermediate path. The important difference is the source of the
target: it comes from the planned path instead of always being the final goal.

The initial ROS2 tuning uses a `0.10 m` lookahead and a maximum linear speed of
`0.20 m/s`. Forward speed is scaled down as heading error increases, and the
robot rotates in place when the error exceeds `30°`. This deliberately keeps
the controller close to the grid path. A larger lookahead can cut corners
around obstacle-inflation boundaries, causing the continuously moving robot to
enter a grid cell that the planner considers unsafe. Within `0.60 m` of the
final endpoint, the follower switches to a final-approach mode: it tracks the
endpoint directly, uses a lower heading gain and `0.60 rad/s` angular limit,
and applies a `0.03 rad` heading deadband. This prevents the desired bearing
from jumping between nearby grid waypoints during the final approach.

The static occupancy grid is published in the `map` frame. In the baseline,
`map` and Gazebo's odometry coordinates are numerically aligned, so the planner
can still consume the existing `/odom` pose without a localization correction.
When the optional localizer is enabled, it publishes the standard persistent
`map → odom → base_link` relationship; rejected scan matches leave that
transform unchanged.

## 7. Sensor-aware safety supervisor

The supervisor examines LiDAR rays in a forward angular sector. It obtains the closest valid forward measurement d_front and compares it with a speed-dependent stopping envelope.

$$
d_{\mathrm{stop}} = \frac{v^2}{2a_{\max}} + v\tau + d_{\mathrm{margin}}
$$

The terms represent:

- v: requested forward speed;
- a_max: assumed available deceleration;
- tau: sensing and command latency;
- d_margin: additional uncertainty margin.

### Front clearance

`front clearance` is not a distance read from the map. It is a measurement
made by the simulated LiDAR at one instant. The measurement pipeline is:

```text
Gazebo ray casting
        ↓
sensor_msgs/LaserScan.ranges
        ↓
keep valid rays within the forward sector
        ↓
take the shortest remaining range
        ↓
front clearance used by the safety supervisor
```

The LiDAR is expressed in the robot's local `base_link` frame. A scan contains
one range value for each ray angle:

$$
\alpha_i = \alpha_{\min} + i\Delta\alpha
$$

The supervisor keeps rays within `±60°` of the robot's forward axis. Define the
set of ray indices in that sector as

$$
I_{\mathrm{front}} = \{i \mid |\alpha_i| \le 60^\circ\}
$$

The front clearance is the closest valid return in that set:

$$
d_{\mathrm{front}}(t) = \min_{i \in I_{\mathrm{front}}} r_i(t)
$$

Here, $r_i(t)$ is the distance from the LiDAR origin to the first surface hit by
ray $i$, and $\alpha_i$ is that ray's angle relative to the robot. The set in
the minimum selects only ray indices inside the forward sector; the
implementation also rejects NaN and out-of-range readings. This value is
therefore not automatically the distance from the robot's outer body to the
obstacle.

In the current Gazebo model, the LiDAR is centered in the robot footprint and
the base collision box has dimensions `0.50 m × 0.36 m`. For a flat wall
directly ahead, the front face is approximately `0.25 m` in front of the
LiDAR origin, so the body-face gap would be roughly:

```text
body-face gap ≈ front clearance - 0.25 m
```

That approximation does not hold exactly for a corner, an angled surface, or
an obstacle seen by an off-axis ray. This is why the project reports the
sensor measurement explicitly as `minimum_clearance_m` instead of calling it
the robot's exact geometric clearance.

The current sensor has 360 horizontal rays over 360 degrees, a usable range
from `0.12 m` to `10.0 m`, and the safety supervisor only uses the forward
`±60°` subset for the stop decision.

The terminology in the evaluation CSV is:

- `front clearance`: the instantaneous value used by the safety decision;
- `minimum_clearance`: the configured lower-bound threshold;
- `minimum_clearance_m`: the smallest instantaneous front clearance observed
  during the complete run.

For example, with `minimum_clearance=0.55 m`, a measured front clearance of
`0.537 m` triggers the supervisor, while `0.633 m` does not. The dynamic
stopping envelope can still require a larger value at higher speed.

The current decision rule is conservative:

~~~
no raw command yet       → publish nothing
no LiDAR scan yet        → publish zero velocity
d_front <= max(d_stop, d_min)
                         → stop forward motion and turn toward clearer space
d_front > max(d_stop, d_min)
                         → forward the raw command
~~~

The current supervisor uses a `±60°` forward sector and a hard minimum
clearance of `0.50 m` in addition to the speed-dependent stopping envelope.
The planner separately uses a `0.35 m` robot-radius parameter when inflating
obstacles on the occupancy grid.
It also uses a `0.03 m` hysteresis band when releasing a stop, so a scan that
oscillates around the threshold does not repeatedly toggle the command.
When forward motion is blocked, it sets `linear.x=0` and selects one angular
command for that blocked episode from the side with more measured LiDAR
clearance. The safety layer owns this recovery direction even if the path
follower requests the opposite turn; this prevents a stale or corner-cutting
path command from steering into the obstacle. The latched turn prevents the
safety layer and path follower from alternating directions at the obstacle
edge. The direction remains fixed until the front clearance exceeds the
hysteresis release threshold; side clearances are not recomputed during the
same blocked episode because the rotating robot would make the two sectors
alternate. If the path follower intentionally commands pure rotation with
`linear.x=0` during a blocked episode, the supervisor keeps its locked recovery
turn instead of forwarding the competing path command. In the ideal simulation MVP, a separate tilt guard
stops the robot when roll or pitch exceeds `10°`, so a collision cannot turn
into extended tipped-state motion.

For ROS2 LaserScan messages, a positive infinite range means that the ray did
not hit an obstacle within the sensor range. The supervisor therefore treats a
front sector containing only positive infinity values as clear up to
`range_max`; NaN-only data remains invalid and causes a stop.

This first version performs an emergency stop rather than a smooth braking trajectory. That makes the safety property easy to inspect. A later version can add a commanded deceleration profile and compare it against the same envelope.

The supervisor is therefore not just a fixed rule such as “stop below 0.5 m.” It accounts for the fact that a faster robot needs more distance to stop.

## 8. Gazebo model

The SDF world contains:

~~~
world
├── physics system
├── scene broadcaster
├── sensor system
├── inline ground plane
├── inline directional light
├── obstacle box
└── diff_drive_robot
    ├── base_link
    ├── left_wheel
    ├── right_wheel
    ├── front_caster
    ├── GPU LiDAR
    └── DiffDrive system plugin
~~~

The ground and light are defined directly in the SDF instead of using external model:// references. This makes the world self-contained and avoids dependency on a local Gazebo model cache, Fuel downloads, or a particular GZ_SIM_RESOURCE_PATH.

The obstacle is deliberately placed along the initial goal direction:

~~~
start ≈ (0, 0)
goal  = (2, 0)
obstacle center ≈ (1, 0)
~~~

The expected result is a safe route around the obstacle, with the safety layer
overriding forward motion whenever the measured clearance becomes unsafe.

### Scenario profiles

The shared scenario profile in
`ros2_ws/src/robotics_nav/robotics_nav/scenario_profiles.py` defines the
obstacle rectangles used by both the static `/map` publisher and the Gazebo
world. Select a profile with:

~~~bash
ros2 launch robotics_sim sim.launch.py scenario:=l_corridor
~~~

The current profiles are `baseline_obstacle`, `l_corridor`, and
`symmetric_corridor`. The default remains `baseline_obstacle`, so existing
baseline commands retain their original geometry.

## 9. Launch sequence

When running:

~~~bash
ros2 launch robotics_sim sim.launch.py
~~~

ROS2 starts the processes in parallel:

1. The launch file renders the selected scenario into a temporary SDF, then
   Gazebo loads that world and starts physics.
2. The bridge creates the ROS/Gazebo topic translations.
3. The path follower waits for /plan and ground-truth /odom.
4. The safety supervisor waits for /scan and /cmd_vel_raw.
5. Gazebo publishes ground-truth /odom, wheel /wheel_odom, and LiDAR data.
6. The path follower publishes /cmd_vel_raw.
7. The safety supervisor either blocks or forwards the command.
8. Gazebo receives /cmd_vel and applies it to the wheel joints.

The launch file does not guarantee that every process remains alive. If Gazebo fails while parsing the SDF, the bridge and ROS2 nodes can remain alive, but the simulation is no longer producing measurements.

## 10. How to interpret diagnostics

### Build success

~~~text
Summary: 2 packages finished
~~~

This means the Python ROS2 packages were installed into the workspace. It does not prove that Gazebo can parse the SDF or that the topics contain data.

### A topic appears in ros2 topic list

This only proves that a ROS graph endpoint exists. The bridge can create a topic even if Gazebo later exits. To verify actual data, use:

~~~bash
ros2 topic echo /odom --once
ros2 topic echo /wheel_odom --once
ros2 topic echo /scan --once
~~~

Both commands should receive a message while Gazebo is running.

### Gazebo exits with Unable to find uri

This means the SDF references an external model that Gazebo cannot resolve. It is a world/resource problem, not a controller or planner problem. The current world avoids this by defining the ground and light inline.

### AMENT_PREFIX_PATH warning after cleaning

If install/ is deleted while the current shell still contains its old path, colcon may print a warning about a non-existent prefix. Re-sourcing the ROS installation and the newly generated workspace fixes the environment for that shell.

## 11. Minimal validation protocol

Run the build from the WSL terminal:

~~~bash
cd /mnt/e/HPC_simulation_porfolio/Robotics/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

Start the simulator:

~~~bash
ros2 launch robotics_sim sim.launch.py
~~~

In a second WSL terminal:

~~~bash
source /opt/ros/jazzy/setup.bash
source /mnt/e/HPC_simulation_porfolio/Robotics/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
~~~

The first validation questions are:

1. Does Gazebo stay alive?
2. Does /odom produce ground-truth messages?
3. Does /wheel_odom produce wheel-odometry messages?
4. Does /scan produce messages?
5. Does /cmd_vel_raw appear after odometry arrives?
6. Does /cmd_vel become zero when the obstacle enters the stopping envelope?

Only after these answers are positive should we tune gains or add a planner.

### Planner scaling benchmark

The offline benchmark in `tools/planner_scaling_benchmark.py` evaluates the
planning layer independently from ROS2 and Gazebo. Every planner receives the
same generated grid, start cell, goal cell, obstacle density, and random seed.
The CSV output records:

- success or failure;
- path cost and path length;
- expanded nodes;
- planning runtime;
- grid size, obstacle density, and heuristic label.

The default suite includes DFS backtracking, BFS, Dijkstra, Greedy
Best-First, A* with `h=0`, A* with Manhattan distance, A* with a
goal-directed Manhattan tie-break, and A* with Euclidean distance. The `h=0`
A* case is included explicitly to compare it with Dijkstra. On a four-neighbor
unit-cost grid, Manhattan distance is the more informative admissible
heuristic, while Euclidean distance is also admissible but generally less
informative.

The goal-directed variant orders equal `f(n)` nodes by smaller `h(n)`, using
the tuple `(f(n), h(n), counter)`.
This does not change the optimality condition because it only changes the
ordering among nodes with equal `f(n)`, but it can substantially reduce
expanded nodes on open grids.

Run a small smoke benchmark with:

~~~bash
python tools/planner_scaling_benchmark.py \
  --sizes 20,50 \
  --densities 0,0.1 \
  --seed-count 2 \
  --output results/planner_smoke.csv
~~~

The full default sweep uses grid sizes `20,50,100,200`, obstacle densities
`0,0.1,0.2,0.3`, and three seeds. Generated CSV files are ignored by Git so
that experiment outputs can be regenerated rather than silently becoming part
of the source baseline.

### Closed-loop evaluation logger

`evaluation_logger` is a read-only ROS2 node. It subscribes to `/odom`,
`/plan`, `/scan`, `/cmd_vel_raw`, `/safety_override`, and
`/collision/contacts`, and samples the run at a fixed rate. It measures:

- start and goal positions;
- final position error and goal success;
- elapsed time and time-to-goal;
- initial/latest planned path length and replan count;
- travelled distance and planned-to-executed path-length ratio;
- minimum front-sector clearance;
- safety override count, override time, and override ratio.

The logger subscribes to the supervisor's explicit `/safety_override` Boolean
status for these override metrics. It does not infer overrides by comparing
the latest `/cmd_vel_raw` and `/cmd_vel` messages, because `Twist` messages do
not contain timestamps and asynchronous callbacks can otherwise create false
override events.

The Gazebo world publishes contact messages on `/collision/contacts`. The
logger reports `collision: true` when the contact array is non-empty and
`collision: false` when the bridge publisher exists but no contact event has
arrived. If the topic is not available in another simulation, the result
remains `unknown`. This keeps clearance and collision as separate
measurements. The logger never publishes velocity commands.

The logger is included in the default simulation launch. To print the summary
only:

```bash
ros2 launch robotics_sim sim.launch.py
```

To additionally write one CSV row when the launch is stopped:

```bash
ros2 launch robotics_sim sim.launch.py \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/closed_loop_metrics.csv
```

The output path is optional and the `results/` directory is ignored by Git.

For a replay or GIF, the logger can additionally save the per-sample trajectory,
the first non-empty planned path, and the occupied map cells:

```bash
ros2 launch robotics_sim sim.launch.py \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/baseline_metrics.csv \
  trace_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/baseline_trace.csv \
  plan_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/baseline_plan.csv \
  map_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/baseline_map.csv
```

These outputs are optional diagnostic artifacts. The trace records odometry,
heading, raw velocity commands, forward LiDAR clearance, and the safety
override state; it does not publish commands or alter the controller.

For localization failure diagnosis, the same trace also records the latest
`/state_estimate` position and its goal error, the candidate and actually
applied LiDAR corrections, the signed correction components, the latest match
validity/status, the raw match score, and the score improvement. The summary
CSV contains the corresponding final values. These fields make it possible to
separate three cases that otherwise look identical in a video:

1. the wheel/IMU state estimate has drifted;
2. the scan matcher found a candidate but rejected it through a quality,
   consistency, safety, or correction-size gate; or
3. a correction was accepted, but smoothing or repeated small updates were
   insufficient to remove the physical pose error.

The diagnostic topics are sampled as latest-value state at the logger's sample
times, so they explain the estimator/controller timeline without becoming a
second control loop.

The compact read-only report can be generated from one evaluation row and its
trace with:

```bash
python tools/diagnose_navigation_trace.py \
  --evaluation results/diagnostic_instrumented_eval.csv \
  --trace results/diagnostic_instrumented_trace.csv
```

It reports whether the physical robot reached the goal, whether an estimated
pose reached it first, the final error gap, the largest candidate/applied
correction, and the distribution of localizer rejection statuses.

For a lower-level, deterministic matcher audit, the localizer can write a
JSONL record containing the static map, every scan/pose input snapshot, and the
corresponding raw match result plus gate decision:

```bash
ros2 launch robotics_sim sim.launch.py \
  localization_diagnostic_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/localizer_audit.jsonl \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/localizer_audit_eval.csv

python tools/replay_lidar_match.py \
  --audit results/localizer_audit.jsonl
```

`replay_lidar_match.py` reruns only the pure LiDAR-map matcher; it does not
start ROS or apply corrections. Zero replay error separates deterministic
matcher geometry from the online acceptance gates. In that case, statuses such
as `waiting_for_consecutive_matches`, `correction_too_large`, and
`insufficient_score_improvement` describe the actual reason a candidate was
not applied, rather than an unexplained background-thread failure. The command
also prints status counts, the number of accepted candidates, and the number
of non-zero candidates whose applied correction remained zero.

`path_efficiency` is reported only for successful runs. For an incomplete
run, the robot has not traversed the full planned route, so dividing the
initial full-path length by partial travelled distance would produce a
misleading value greater than one.

The CSV also reports `path_length_ratio`. In the equation below, this metric is
denoted by $r_{\mathrm{path}}$:

$$
r_{\mathrm{path}}=\frac{L_{\mathrm{travelled}}}{L_{\mathrm{planned}}}
$$

The initial planned path is a grid polyline, while the executed trajectory is
continuous and may cut across grid corners. Therefore the ratio can be below
one without implying that the planner found a shorter discrete path; it mainly
describes the difference between the rasterized reference and the smooth
executed trajectory.

### Reproducible safety configuration

Safety parameters can be overridden at launch time and are written into the
CSV for traceability:

```bash
ros2 launch robotics_sim sim.launch.py \
  minimum_clearance:=0.50 \
  sensor_latency:=0.10 \
  scan_delay_s:=0.00 \
  scan_noise_std_m:=0.00 \
  scan_noise_seed:=0 \
  safety_margin:=0.15 \
  recovery_timeout_s:=8.0 \
  planning_radius_m:=0.35 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/closed_loop_metrics.csv
```

This makes parameter sweeps reproducible: each result row contains both the
measured outcomes and the safety configuration that produced them.

`scan_delay_s` is separate from `sensor_latency`. `sensor_latency` changes the
stopping-distance assumption, while `scan_delay_s` makes the safety supervisor
use an older received `/scan` message. The delay is measured from message
arrival time, so it remains meaningful when ROS simulation time is enabled.
The default `scan_delay_s=0.0` uses the newest available scan and preserves the
baseline behaviour.

`scan_noise_std_m` adds zero-mean Gaussian noise to finite LiDAR ranges before
the safety supervisor evaluates them. `scan_noise_seed` makes the generated
noise reproducible. The evaluator continues to measure clearance from the
unmodified `/scan` topic, so the CSV separates physical clearance from the
noisy observation used by the safety layer.

`planning_radius_m` controls the global planner's grid obstacle inflation.
The default `0.35 m` represents the robot footprint plus discretization
margin. When a larger safety clearance is required, increasing this value
lets the planner search for a route that is compatible with the safety layer;
otherwise the planner may produce a path that the supervisor must reject.

The safety recovery also has a timeout. If the robot remains inside the
blocked state longer than `recovery_timeout_s`, the supervisor stops with zero
velocity and reports a recovery-timeout error. This is a fail-safe experiment
termination condition for infeasible planner/safety combinations; it is not a
replacement for replanning.

For repeatable batch experiments, `experiment_timeout_s` provides a separate
total-run limit in the evaluation logger. A positive value causes the logger
to finish the run and shut down the ROS graph after that many seconds from the
first odometry sample; `0.0` disables the limit. The CSV records both
`configured_experiment_timeout_s` and `termination_reason`, whose values can
include `goal_reached`, `goal_confirmation_failed`,
`final_approach_budget_exhausted`, `experiment_timeout`, or
`manual_interrupt`. A confirmation failure or exhausted final-approach budget
is a causal terminal event: the evaluator ends the run when it is received,
rather than leaving the stopped controller alive until the global timeout.
Importantly, the logger no longer terminates with `goal_reached` merely because
the ideal Gazebo `/odom` pose briefly enters the goal tolerance. That event is
recorded as `ground_truth_goal_reached_any_time` (with the legacy
`ground_truth_goal_reached` alias retained); the terminal `goal_reached` reason
requires the path follower's explicit `goal_reached_latched` event. The
additional final-sample fields distinguish historical entry from the physical
pose at shutdown. This prevents the evaluation process from cutting off
`CONFIRMING` or `FINAL_APPROACH` recovery and makes incomplete runs
diagnostically unsuccessful.

Example:

```bash
ros2 launch robotics_sim sim.launch.py \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/timeout_case.csv
```

### Representative planner/safety alignment experiment

The following runs use the same world, start pose, goal, and controller. Only
the minimum safety clearance and the planner's obstacle-inflation radius are
changed:

The result columns are defined as follows:

| Column | Meaning |
| --- | --- |
| `case` | Short label for the experiment configuration. |
| `minimum_clearance` | Configured LiDAR distance threshold used by the safety supervisor. |
| `planning_radius_m` | Configured obstacle-inflation radius used by the A* planner. |
| `success` | Terminal physical completion: controller latch, `goal_reached` termination, and final `/odom` within tolerance. |
| `controller_latched_without_physical_completion` | The controller latched, but the final `/odom` sample was outside the goal tolerance. |
| `ground_truth_goal_reached_any_time` | Whether /odom ever entered the goal tolerance during the run. |
| `ground_truth_final_within_goal_tolerance` | Whether the final complete /odom sample was inside the goal tolerance. |
| `navigation_pose_goal_reached_any_time` | Whether the navigation pose ever entered the goal tolerance. |
| `navigation_pose_final_within_goal_tolerance` | Whether the final navigation pose sample was inside the tolerance. |
| `state_estimate_goal_reached_any_time` | Whether /state_estimate ever entered the goal tolerance. |
| `state_estimate_final_within_goal_tolerance` | Whether the final state-estimate sample was inside the tolerance. |
| `time_to_goal_s` | Time from the first odometry sample to goal arrival. |
| `travelled_distance_m` | Distance accumulated from the odometry trajectory. |
| `minimum_clearance_m` | Smallest valid LiDAR return in the forward sector during the run. |
| `safety_override_ratio` | Safety-override time divided by motion time. |
| `collision` | Contact-sensor result reported by the evaluation logger. |
| `termination_reason` | Why the evaluation logger stopped the run. |

The comparison results are:

| Case | `minimum_clearance` | `planning_radius_m` | `success` | `time_to_goal_s` | `travelled_distance_m` | `minimum_clearance_m` | `safety_override_ratio` | `collision` |
| --- | ---: | ---: | :---: | ---: | ---: | ---: | ---: | :---: |
| Baseline | 0.50 m | 0.35 m | yes | 67.74 | 3.783 | 0.520 | 0.000 | false |
| Mismatched constraints | 0.55 m | 0.35 m | no | -- | 0.536 | 0.538 | 0.658 | false |
| Cell-boundary test | 0.55 m | 0.40 m | no | -- | 0.497 | 0.537 | 0.688 | false |
| First feasible boundary | 0.55 m | 0.41 m | yes | 70.32 | 3.981 | 0.633 | 0.000 | false |
| Intermediate alignment | 0.55 m | 0.45 m | yes | 68.64 | 3.983 | 0.634 | 0.000 | false |
| Aligned constraints | 0.55 m | 0.55 m | yes | 73.66 | 4.195 | 0.735 | 0.000 | false |

The mismatched case demonstrates why the safety supervisor cannot be treated
as a substitute for planning: the planner generated a route that approached
an obstacle more closely than the runtime safety threshold allowed. The
supervisor prevented collision, but the robot could not complete the route
before the recovery timeout. Increasing the planning radius produced a longer
route, but restored feasibility and increased the measured clearance. The
intermediate run also shows that the relationship is quantized by the `0.10 m`
occupancy-grid resolution: obstacle inflation is converted to an integer
number of grid cells, so a small change in the continuous parameter may have
no effect until it crosses the next cell boundary.

### Stopping-envelope stress test

A second experiment kept the baseline planner radius at `0.35 m` but increased
the safety margin from `0.15 m` to `0.55 m`:

| Case | `safety_margin` | `planning_radius_m` | `success` | `travelled_distance_m` | `minimum_clearance_m` | `safety_override_time_s` | `safety_override_ratio` | `collision` |
| --- | ---: | ---: | :---: | ---: | ---: | ---: | ---: | :--- |
| Baseline margin | 0.15 m | 0.35 m | yes | 3.783 | 0.520 | 0.000 | 0.000 | false |
| High-margin stress | 0.55 m | 0.35 m | no | 0.390 | 0.550 | 28.465 | 0.482 | false |
| Minimum tested feasible radius | 0.55 m | 0.41 m | yes | 3.979 | 0.633 | 0.000 | 0.000 | false |
| High-margin aligned | 0.55 m | 0.60 m | yes | 4.200 | 0.733 | 0.000 | 0.000 | false |

The high-margin run was stopped by the safety supervisor before reaching the
goal. This demonstrates that a planner using only footprint inflation can
still produce a route that is infeasible under a more conservative dynamic
stopping envelope. The robot remained collision-free, but spent much of the
run in safety recovery because the current supervisor does not replan from
its stopping-envelope constraint. With `planning_radius_m=0.60`, the measured
clearance exceeded the approximately `0.595 m` stopping requirement and the
run completed without a safety override.

### Sensor-latency parameter comparison

The `sensor_latency` parameter is used in the stopping-envelope model as the
time available for sensing, command transport, and actuation before braking
starts. `scan_delay_s` is the separate experimental parameter for injecting an
actual delay into the safety supervisor's LiDAR input.

Both runs below use `minimum_clearance=0.50 m`,
`safety_margin=0.55 m`, and `planning_radius_m=0.41 m`:

| Case | `sensor_latency` | `success` | `time_to_goal_s` | `travelled_distance_m` | `minimum_clearance_m` | `safety_override_ratio` | `collision` |
| --- | ---: | :---: | ---: | ---: | ---: | ---: | :---: |
| Latency 0.10 s | 0.10 s | yes | 70.48 | 3.979 | 0.633 | 0.000 | false |
| Latency 0.30 s | 0.30 s | yes | 69.75 | 3.979 | 0.633 | 0.000 | false |

In this deterministic world, increasing the modelled latency from `0.10 s` to
`0.30 s` did not trigger a safety override because the executed route retained
enough clearance. The result should not be interpreted as proof that real
sensor or network latency has no effect: a tighter route, higher speed, noisy
LiDAR, or actuator delay could make the additional stopping distance active.
The experiment shows that latency must be evaluated together with obstacle
inflation, speed limits, and safety margin rather than as an isolated number.

### Real LiDAR-delay sweep

The following sweep holds `sensor_latency=0.10 s`,
`safety_margin=0.15 m`, `minimum_clearance=0.50 m`, and
`planning_radius_m=0.35 m` fixed. Only the age of the LiDAR scan used by the
safety supervisor changes:

| `scan_delay_s` | `success` | `time_to_goal_s` | `minimum_clearance_m` | `safety_override_count` | `safety_override_time_s` | `safety_override_ratio` | `collision` |
| ---: | :---: | ---: | ---: | ---: | ---: | ---: | :---: |
| 0.00 s | yes | 65.04 | 0.521 | 0 | 0.000 | 0.0000 | false |
| 0.10 s | yes | 68.99 | 0.520 | 1 | 0.050 | 0.0007 | false |
| 0.20 s | yes | 64.88 | 0.520 | 1 | 0.150 | 0.0023 | false |
| 0.30 s | yes | 67.51 | 0.520 | 1 | 0.250 | 0.0037 | false |

All four runs reached the goal without collision. The time-to-goal variation
is not monotonic because it also includes controller and simulator timing
variation. The clearer trend is the longer safety-intervention duration as
the supervisor operates on increasingly stale scans. Here
`safety_override_count` counts distinct blocked episodes; a count of one does
not mean that the robot was overridden for the whole run.

### LiDAR-noise experiment

The next robustness sweep should hold the map, controller, safety margin, and
scan delay fixed while varying only the LiDAR range-noise standard deviation:

```bash
ros2 launch robotics_sim sim.launch.py \
  minimum_clearance:=0.50 \
  sensor_latency:=0.10 \
  scan_delay_s:=0.00 \
  scan_noise_std_m:=0.03 \
  scan_noise_seed:=1 \
  safety_margin:=0.15 \
  planning_radius_m:=0.35 \
  recovery_timeout_s:=8.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/sweep_noise_003_seed_001.csv
```

Use several fixed seeds for each noise level, for example `0.00`, `0.01`,
`0.03`, and `0.05 m`. This separates the effect of random measurement error
from the deterministic effect of stale scans.

The first aligned smoke test used `scan_noise_std_m=0.03 m`,
`scan_noise_seed=1`, and `planning_radius_m=0.41 m`:

| Noise standard deviation | Seed | `success` | `time_to_goal_s` | `minimum_clearance_m` | `safety_override_ratio` | `collision` |
| ---: | ---: | :---: | ---: | ---: | ---: | :---: |
| 0.03 m | 1 | yes | 72.23 | 0.633 | 0.000 | false |
| 0.03 m | 2 | yes | 70.10 | 0.631 | 0.000 | false |
| 0.03 m | 3 | yes | 70.69 | 0.633 | 0.000 | false |
| 0.05 m | 1 | yes | 70.53 | 0.632 | 0.000 | false |
| 0.05 m | 2 | yes | 70.04 | 0.630 | 0.005 | false |
| 0.05 m | 3 | yes | 70.90 | 0.633 | 0.000 | false |

Across these three seeds, the success rate was `3/3`, the mean time-to-goal
was approximately `71.00 s`, and the mean measured minimum clearance was
`0.632 m`. This is an initial robustness check rather than a statistically
large Monte Carlo study, but it confirms that the result is not specific to a
single noise realization.

For the `0.05 m` noise level, all three seeds also succeeded. The mean
time-to-goal was approximately `70.49 s` and the mean measured minimum
clearance was `0.632 m`. One seed produced four short safety interventions,
for a total intervention time of `0.350 s`; the other two seeds produced no
intervention. This illustrates why repeated seeds are useful even when the
success rate remains unchanged.

### Noise-induced planning boundary

To locate the feasibility boundary, three seeds were tested at both
`planning_radius_m=0.40 m` and `0.41 m` with `scan_noise_std_m=0.03 m`:

| Noise standard deviation | Planning radius | Success rate | Mean time-to-goal (successful runs) | Mean minimum clearance | Mean safety override ratio | Collision count |
| ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| 0.03 m | 0.40 m | 1/3 | 462.93 s | 0.525 m | 0.683 | 0/3 |
| 0.03 m | 0.41 m | 3/3 | 71.00 s | 0.632 m | 0.000 | 0/3 |

The `0.40 m` configuration is therefore not strictly impossible, but it is
stochastic and operationally poor. Its one successful seed took `462.93 s`,
triggered 89 safety overrides, and reached a measured minimum clearance of
`0.485 m`; the other two seeds did not reach the goal. By contrast, all three
`0.41 m` seeds succeeded in approximately 71 s without sustained safety
intervention. Under these tested seeds, `0.41 m` was the smallest tested
planning radius that achieved consistent success. This is an empirical
boundary result, not a statistical robustness guarantee.

At the higher noise level `scan_noise_std_m=0.05 m`, the `0.40 m` radius
failed for all three seeds (`0/3`). The mean safety override ratio was `0.764`
and the mean measured minimum clearance was `0.573 m`; all runs remained
collision-free. This reinforces the distinction between a marginal boundary
configuration and a robust operating configuration.

An interaction test combined `scan_delay_s=0.30 s` with
`scan_noise_std_m=0.05 m`, using `planning_radius_m=0.41 m` and three seeds:

| Seed | `success` | `termination_reason` | `time_to_goal_s` | `minimum_clearance_m` | `safety_override_time_s` | `safety_override_ratio` | `collision` |
| ---: | :---: | :--- | ---: | ---: | ---: | ---: | :---: |
| 1 | yes | goal reached | 69.15 | 0.632 | 0.250 | 0.0036 | false |
| 2 | yes | goal reached | 69.06 | 0.627 | 0.800 | 0.0116 | false |
| 3 | yes | goal reached | 69.93 | 0.633 | 0.499 | 0.0072 | false |
| Mean | 3/3 | -- | 69.38 | 0.631 | 0.516 | 0.0075 | 0/3 |

The combined uncertainty therefore produced successful, collision-free
navigation for all three seeds. The safety layer intervened briefly in each
run, but the mean override ratio remained below one percent. This is a
repeatability result for the tested configuration, not a general statistical
guarantee for arbitrary maps or uncertainty levels.

### Robustness summary tool

The one-row evaluation files can be aggregated without starting ROS or
Gazebo:

```bash
python tools/summarize_robustness.py \
  --glob "results/sweep_*.csv" \
  --output results/robustness_summary.csv
```

The tool groups runs by safety and sensing configuration, keeps the random
seeds visible, and reports success rate, collision count, mean and 95th
percentile time-to-goal, clearance, and safety-override statistics. Legacy
CSV files without the complete configuration columns are skipped so
that missing values are not mistaken for a real zero-delay or zero-noise
experiment. It also counts `goal_reached`, `experiment_timeout`, and
`manual_interrupt` termination reasons; older reports without that field are
classified as `goal_reached` when successful and `unknown` otherwise. The
summary is generated data and remains ignored by Git.

To render the current summary as a four-panel figure for the project
documentation:

```bash
python tools/plot_robustness.py \
  --summary results/robustness_summary.csv \
  --output docs/assets/robustness_summary.png
```

Compared with the earlier `0.35 m` planning radius, the larger radius leaves
enough physical clearance for the noisy safety observation without causing a
false recovery stop in this run. This is why noise experiments should first
use a planner/safety configuration with a known clearance margin, then test
the tighter boundary configuration separately.

## 12. Current status and roadmap

The current ROS2 milestone includes:

- a deterministic occupancy-grid publisher;
- an A* global planner publishing `nav_msgs/Path`;
- ordered path following for the differential-drive robot;
- LiDAR-based safety supervision with a speed-dependent stopping envelope;
- clearance hysteresis, latched recovery turning, and a tilt guard;
- a wheel/IMU heading estimator combining `/wheel_odom` and `/imu`;
- an evaluation-only logger reporting wheel and estimated pose RMSE against `/odom`;
- configurable, seeded gyro bias and white-noise perturbations for estimator diagnostics;
- an estimator-summary tool that groups runs by uncertainty configuration;
- a five-state pose EKF with covariance propagation, NIS gating, and
  measurement-acceptance diagnostics;
- LiDAR score alternatives (`range`, `endpoint`, `boundary`, and an
  ICP-style `point_to_line` diagnostic) inside the same bounded local search;
- local-score observability reports with axis gains, curvature, candidate
  spread, score margin, and covariance-gate diagnostics;
- synchronized evaluation traces containing estimator timestamps, executed
  commands, localization status, and candidate/applied correction fields;
- temporal decision diagnostics containing source-pose and LiDAR timestamps,
  logger receipt ages, controller goal events, and event history;
- planner/safety clearance alignment through one effective planning-clearance
  model, eliminating the previously observed recovery-turn loop;
- a Gazebo goal marker that remains visible but is excluded from the LiDAR mask.

The current limitations are equally important:

- the numerical gate values are frozen experiment configuration, not claims
  of map-independent optimality; cross-map validation changes the map and
  task while keeping those values fixed;
- `/odom` remains the only validated physical navigation reference;
- `/state_estimate` has useful covariance and NIS diagnostics but still shows
  systematic position error under wheel-motion uncertainty;
- `/localized_estimate` can reduce the reported pose error while remaining
  wrong enough to place the physical robot outside the goal tolerance;
- sparse raster-map scans can produce ambiguous nearby minima, and the
  asynchronous matcher can return stale or repeated candidates;
- a localized pose may enter the goal tolerance while the controller is still
  waiting for independent confirmation; the optional confirmation deadline
  now leaves that wait through bounded `FINAL_APPROACH` recovery, but the
  recovery is not yet latency-compensated and does not prove physical arrival;
- LiDAR corrections are external map-to-odom updates, not LiDAR measurements
  fused inside the pose EKF;
- the bounded local matcher is not full ICP, SLAM, loop closure, or globally
  observable localization.

### Probabilistic known-map localization

The probabilistic localization backend is developed separately from the frozen
deterministic local matcher. It uses known-map Monte Carlo Localization:
wheel/IMU increments propagate a particle set, LiDAR endpoints receive
likelihood-field weights from the occupancy map, and systematic resampling
maintains plausible pose hypotheses. The weighted pose, covariance, effective
sample size, and normalized entropy expose ambiguity instead of selecting one
local minimum.

The dependency-free mathematical core is in
`ros2_ws/src/robotics_nav/robotics_nav/mcl_localization.py` and is tested
without Gazebo. The ROS adapter is `mcl_localizer`; selecting
`localization_backend:=mcl` preserves the existing `/localized_estimate`
contract while replacing only the localization model. It is not SLAM: the map
is still assumed known and static, and the first adapter assumes map/odom
alignment. Controller and evaluation behavior can therefore be compared
without changing their parameters. The deterministic local matcher remains the
baseline; the particle model is evaluated across different maps and tasks
with the same frozen configuration rather than tuned until one map succeeds.

The next layers are intentionally separated so that each experiment remains
interpretable:

- transparent wheel/IMU uncertainty experiments and diagnostic local
  LiDAR-map matching;
- gated, process-backed LiDAR-map correction remains experimental;
- covariance-aware pose EKF and comparison with the transparent estimator;
- fixed-configuration temporal consistency and failure-propagation experiments
  connecting message age to goal decisions and physical stopping error;
- only after the timing failure is isolated, a minimal freshness, dwell, or
  latency-compensation change;
- multi-seed validation of localization-to-control and localization-to-safety
  failure propagation;
- only after that, a separate SLAM/Nav2 integration study;
- later camera-based safety events and perception/sensor fusion.

If the robot stops too early, we can inspect the LiDAR measurement,
stopping-distance calculation, and command velocity separately instead of
debugging a complete autonomous stack at once.

## 13. Future HPC connection

A single A* search has frontier-ordering dependencies, so it is not the first thing to parallelize. Better early HPC experiments are:

- independent planning queries over many start/goal pairs;
- Monte Carlo trials with LiDAR noise and latency;
- parameter sweeps over a_max, tau, and d_margin;
- wavefront-style BFS or cost propagation;
- GPU-friendly grid-cost updates.

The validation output should eventually include:

~~~text
collision rate
minimum clearance
intervention count
goal success rate
path length
planning runtime
expanded nodes
~~~

That connects the ROS2 system back to the original portfolio theme:

$$
\text{measurement} \rightarrow \text{model} \rightarrow \text{decision} \rightarrow \text{quantitative validation}
$$

## 14. Meaning of the main WSL commands

The standard startup sequence is:

~~~bash
cd /mnt/e/HPC_simulation_porfolio/Robotics/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robotics_sim sim.launch.py
~~~

The commands have different roles:

1. `cd` changes the current directory to the ROS2 workspace. The `/mnt/e`
   prefix is WSL's view of the Windows E drive.
2. The first `source` loads the system ROS2 Jazzy installation into the current
   shell. It sets paths such as `PATH`, `AMENT_PREFIX_PATH`, and `ROS_DISTRO`.
3. `colcon build` discovers packages under `src/` and generates build products.
   `--symlink-install` keeps Python packages linked to the source tree during
   development. It creates `build/`, `install/`, and `log/` inside the workspace.
4. The second `source` overlays the local workspace on top of system ROS2, so
   commands such as `ros2 pkg list` can find this project's packages.
5. `ros2 launch` loads `sim.launch.py` from `robotics_sim` and starts Gazebo,
   the bridge, the map publisher, global planner, path follower, and safety
   supervisor.

The `source` commands only change the current terminal. They do not modify
Conda or permanently edit the shell configuration. A new WSL terminal therefore
needs the two `source` commands again.
