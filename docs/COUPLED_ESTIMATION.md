# Coupled state estimation

The repository now has an opt-in coupling point between the pose EKF and the
known-map localizer. This is not SLAM: the occupancy map is still known, and
the localizer estimates a robot pose in that map rather than adding landmark
states to the filter.

## Why this is a coupled Bayesian update

The pose filter state is

$$
\mathbf{x}_k =
\begin{bmatrix}
x_k \\
y_k \\
\theta_k \\
b_{g,k} \\
b_{w,k}
\end{bmatrix}.
$$

Here $x_k$ and $y_k$ are the world-frame position, $\theta_k$ is fused yaw,
$b_{g,k}$ is the estimated IMU gyro-rate bias, and $b_{w,k}$ is the slowly
varying wheel-yaw bias. The EKF covariance $P_k$ contains both the individual
uncertainties and their cross-covariances.

The wheel and IMU streams first produce the predicted state

$$
\mathbf{x}_k^- = f(\mathbf{x}_{k-1}, u_k),
\qquad
P_k^- = F_k P_{k-1} F_k^T + Q_k.
$$

An accepted map-localizer event supplies only a position observation:

$$
\mathbf{z}_k =
\begin{bmatrix}
x_k^{\mathrm{map}} \\
y_k^{\mathrm{map}}
\end{bmatrix},
\qquad
H =
\begin{bmatrix}
1&0&0&0&0 \\
0&1&0&0&0
\end{bmatrix}.
$$

The localizer also supplies its $2 \times 2$ position covariance $R_k$. The
EKF computes

$$
\mathbf{r}_k = \mathbf{z}_k - H\mathbf{x}_k^- ,
\qquad
S_k = H P_k^- H^T + R_k,
$$

$$
K_k = P_k^- H^T S_k^{-1},
\qquad
\mathbf{x}_k = \mathbf{x}_k^- + K_k\mathbf{r}_k.
$$

Because $K_k$ uses the complete covariance matrix, a non-zero position-to-yaw
or position-to-bias cross-covariance can update yaw or bias when position is
corrected. No separate hand-tuned correction weight is used for this update.
The covariance is updated with the Joseph form:

$$
P_k = (I-K_kH)P_k^-(I-K_kH)^T + K_kR_kK_k^T.
$$

The two-dimensional NIS is

$$
d_k^2 = \mathbf{r}_k^T S_k^{-1}\mathbf{r}_k.
$$

An event above the existing NIS gate is rejected without changing either the
state or covariance. This gate is a consistency test, not an optimizer and not
a replacement for map ambiguity handling.

## Event semantics

`/localized_estimate` remains a navigation output and may repeat the latest
posterior or fall back to `/state_estimate` when the localizer has no accepted
evidence. It must not be treated as a new measurement on every publication.

For that reason, an accepted MCL update publishes exactly one
`nav_msgs/msg/Odometry` event on `/localization_candidate`. Its pose
covariance is copied into the message. With
`external_position_fusion:=true` and `fusion_mode:=ekf`, the heading estimator
consumes this event and publishes the resulting corrected `/state_estimate`.
The default is `false`, so the existing estimator and navigation baselines are
unchanged.

The current coupling is deliberately conservative:

- only accepted known-map MCL events are consumed;
- only $(x,y)$ is measured, so a map alias cannot directly overwrite yaw;
- the MCL prior-vs-candidate evidence gate remains active;
- the EKF NIS gate provides a second consistency check;
- the map and odometry frames are assumed aligned in this experiment;
- delayed-measurement replay is implemented; a bounded fixed-lag smoother and
  a jointly estimated map-to-odometry transform are not implemented yet.

The motion-prior topic is intentionally separate from the fused output. The
heading estimator publishes `/state_prediction` from the wheel/IMU-only EKF
instance. The MCL backend consumes that topic, while its accepted
`/localization_candidate` events update the separate fused `/state_estimate`.
Thus the information flow is

```text
wheel + IMU -> state prediction -> MCL -> accepted map position -> EKF
                                      \_____________________________/
                                                state estimate
```

The corrected state is not fed back into the MCL motion prior. This prevents a
wrong but locally plausible map hypothesis from recursively contaminating the
next prediction. The separation is structural rather than a new gate or a
manually chosen correction weight.

The high-rate controller must preserve the same separation. In the controlled
comparison, allowing `/state_estimate` to drive the controller produced about
`0.108 m` physical final error, while keeping control on the independent
`/state_prediction` produced about `0.062 m` with external fusion still
enabled. Therefore `/state_prediction` is the current control contract;
`/state_estimate` is a diagnostic/fusion output until global map ambiguity is
resolved.

## Timestamped external observations

The map localizer's candidate is not necessarily computed at the same time as
the newest EKF state. Its message header carries the source pose time
$t_m$, while the newest wheel/IMU event may already be at $t_k$. Applying the
candidate directly to $\mathbf{x}_k^-$ would treat an old observation as if it
were current and can create an artificial jump in the state used by the
controller.

The estimator therefore records the timestamped wheel and IMU event stream.
For a candidate received at time $t_m$, it inserts the position update at the
corresponding point in that stream:

$$
\begin{aligned}
\mathbf{x}_{m}^{+}
&=\mathbf{x}_{m}^{-}
+K_m\left(\mathbf{z}_m-H\mathbf{x}_{m}^{-}\right).
\end{aligned}
$$

and then reapplies the subsequent motion events to obtain the current state:

$$
\begin{aligned}
\mathbf{x}_{k}
&=f_{k}\left(\dots f_{m+1}
\left(\mathbf{x}_{m}^{+},u_{m+1}\right)\dots,u_k\right).
\end{aligned}
$$

The corresponding covariance is replayed through the same prediction and
Joseph-form measurement-update operations. This is a deterministic delayed-
measurement replay, not a manually weighted pose blend. The reported external
measurement age is

$$
a_m=t_k-t_m,
$$

and is published on `/external_position_measurement_age_s`; the logger records
its mean and maximum together with the number of replayed external events.
Future-dated observations are not applied because the required future motion
history does not yet exist.

This estimator freshness contract is separate from terminal confirmation.
`/state_estimate` may be a valid current EKF state even when no accepted map
observation has arrived recently. Therefore a run that uses MCL to validate
absolute arrival can enable `require_timestamped_localization_evidence` in the
controller. The controller then requires new accepted `/localization_candidate`
events after entering the goal tolerance and checks their monotonic receipt
age. A fallback `/localized_estimate` sample or an old accepted event cannot
by itself latch the goal.

This removes one important source of non-physical behavior, but it does not
make a local map match globally observable. A compact but aliased MCL posterior
can still be wrong, and a future fixed-lag smoother should replace the current
run-history replay for long-running systems.

## Why this is not yet SLAM

SLAM would estimate a map or landmark variables together with the robot state,
for example

$$
p(\mathbf{x}_{0:k}, m \mid z_{1:k}, u_{1:k}).
$$

This repository currently receives a static occupancy map, so adding unknown
landmarks would change the experiment from known-map localization to a
different problem. The present coupled update is the smallest meaningful step
before that expansion: it tests whether an external spatial observation can
reduce state error without hiding estimator, localizer, or controller failure.

## Validation plan

The first comparison should keep the existing motion, MCL, and EKF settings
fixed and compare only:

1. EKF baseline with `external_position_fusion:=false`;
2. the same run with `external_position_fusion:=true`;
3. an MCL-rejection run in which no candidate event reaches the EKF.

The important outputs are external-update count, external NIS, rejected-event
count, covariance calibration, physical `/odom` error, independent estimate
error, and terminal-state behavior. A lower estimated error is not sufficient
if the physical pose or goal-confirmation logic still fails.
