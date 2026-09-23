# Probabilistic known-map localization

This document describes a Monte Carlo localization backend that replaces the
single-candidate local grid correction for controlled experiments. The map is
still known and static, so this is not SLAM. The goal is to test whether a
multi-hypothesis model handles map ambiguity more honestly and more generally
than a local optimizer.

## Why this is the next model

The last deterministic-matcher comparison produced repeated confirmation
timeouts and final-approach re-entries. Only a small number of LiDAR matches
were accepted, while the local matcher reported many ambiguous or
uncertainty-gated candidates. The physical final error remained larger than
the estimated-pose error. This is evidence of a model limitation, not a reason
to keep sweeping score thresholds.

The deterministic matcher selects one nearby minimum:

```text
wheel/IMU pose + local scan search -> one corrected pose
```

The particle filter maintains a distribution:

```text
wheel/IMU motion prior -> particle prediction
LiDAR + occupancy map  -> particle likelihoods
                         -> normalization and resampling
                         -> weighted pose, covariance, and ambiguity metrics
```

The first implementation is in
`robotics_nav/mcl_localization.py`. It is dependency-free so the motion model,
likelihood-field model, systematic resampling, circular heading mean, and
covariance calculation can be tested without launching Gazebo. The static map
distance field is cached once per map, so the particle/beam loop does not
re-scan every occupied cell on every update.

## State and motion model

Each particle represents a planar map-frame hypothesis:

```text
p_i = (x_i, y_i, theta_i, w_i)
```

For a wheel/IMU increment `(Delta s, Delta theta)`, the particle uses the
midpoint differential-drive propagation:

$$
\theta_{\mathrm{mid},k}^{(j)}
= \theta_{k-1}^{(j)} + \frac{1}{2}\Delta\theta_k^{(j)}
$$

$$
x_k^{(j)} = x_{k-1}^{(j)} + \Delta s_k^{(j)}\cos\left(\theta_{\mathrm{mid},k}^{(j)}\right)
$$

$$
y_k^{(j)} = y_{k-1}^{(j)} + \Delta s_k^{(j)}\sin\left(\theta_{\mathrm{mid},k}^{(j)}\right)
$$

$$
\theta_k^{(j)} = \mathrm{wrap}\left(\theta_{k-1}^{(j)} + \Delta\theta_k^{(j)}\right)
$$

Independent motion noise is sampled per particle. This is process uncertainty,
not a parameter chosen to make one map succeed.

## LiDAR likelihood

For each valid range return, a particle transforms the measured endpoint into
the map frame. The endpoint is assigned to its map cell, and the cached
distance field supplies the distance from that cell centre to the nearest
occupied-cell centre. This rasterized distance is used in a Gaussian
likelihood field:

$$
p_{i,x}^{\mathrm{map},(j)} = x^{(j)} + r_i\cos\left(\theta^{(j)}+\alpha_i\right)
$$

$$
p_{i,y}^{\mathrm{map},(j)} = y^{(j)} + r_i\sin\left(\theta^{(j)}+\alpha_i\right)
$$

$$
p\left(z_i \mid \mathbf{x}^{(j)}, M\right) = \exp\left(-\frac{d_i\left(\mathbf{x}^{(j)},M\right)^2}{2\sigma_{\mathrm{range}}^2}\right) + \varepsilon
$$

The particle weight is updated by the mean log likelihood over valid beams.
Invalid and out-of-range measurements do not create artificial evidence.

## Short temporal observation window

A single scan near a corridor, doorway, or wall can support several plausible
particle poses. The adapter therefore supports a small fixed window of recent
scans. Each scan is retained with the state-estimate pose available when the
scan arrived. Before scoring, its odometry motion to the current pose is
recorded as a relative transform. For a current particle pose
$\mathbf{x}_k$, the historical scan pose is reconstructed as

$$
\mathbf{x}_{k-j}^{(i)} = \mathbf{x}_k^{(i)} \boxminus \Delta\mathbf{x}_{k-j\rightarrow k},
$$

where $\boxminus$ denotes composition with the inverse planar rigid-body
transform, including the wrapped yaw difference. The old scan is therefore
evaluated at the pose where it was measured, while all particles still
represent the current pose.

For a window of $W$ independent scans, the measurement evidence is combined
as the product of the per-scan likelihood blocks. In the log domain this is:

$$
\ell_{\mathrm{window}}(\mathbf{x}_k^{(i)}) = \sum_{j=0}^{W-1} \ell_j\left(\mathbf{x}_{k-j}^{(i)}\right).
$$

The prior particle weight is combined with this value before normalization
and systematic resampling. A window of one is exactly the original single-scan
update. The window is an observation-model change, not a relaxation of the
uncertainty, entropy, timestamp, or valid-beam gates. If the combined posterior
is still broad, the update remains rejected and the previous trusted estimate
is preserved.

This mechanism addresses local geometric ambiguity by accumulating independent
views while the robot moves. It does not solve global relocalization, estimate
a map, or make a static known-map localizer into SLAM.

## Temporal validity and update contract

The adapter compares the scan timestamp with the newest state-estimate
timestamp before changing particle weights. The scan age is

$$
a_k = t_k^{\mathrm{pose}} - t_k^{\mathrm{scan}}.
$$

A scan is rejected as `stale_scan` when

$$
a_k > a_{\max},
$$

or as `scan_ahead_of_pose` when its timestamp is materially newer than the
pose used for prediction. A scan with fewer than the configured number of
valid sampled beams is rejected as `insufficient_points`. Rejected scans are
logged but do not modify the particle posterior. This is important because an
invalid observation must not silently become process history for later scans.

The same rule applies to an ambiguous but numerically valid update. Let
$P_{xx}$ and $P_{yy}$ be the particle covariance entries after the candidate
measurement update. The candidate planar one-sigma bound is

$$
\sigma_{xy,k} = \max\left(\sqrt{P_{xx,k}},\sqrt{P_{yy,k}}\right).
$$

The navigation adapter applies the candidate only when

$$
\sigma_{xy,k} \leq \sigma_{xy,\max}.
$$

Otherwise it reports `candidate_uncertainty_too_large`, restores the prior
particle posterior, and continues publishing the last trusted prediction.
The candidate pose and covariance remain in the diagnostic record. A broad
distribution is therefore treated as ambiguity, not as permission to use its
weighted mean as a precise planning pose. The adapter also rejects a large
correction when the normalized particle entropy $H_k$ remains above the
configured information bound $H_{\max}$:

$$
\mathrm{apply}_k = \left(\sigma_{xy,k} \leq \sigma_{xy,\max}\right) \land \left(H_k \leq H_{\max} \ \lor\ \left\|\Delta \mathbf{p}_k\right\|_2 \leq \sigma_{xy,\max}\right).
$$

This preserves small, non-disruptive updates while preventing an almost
uniform particle set from moving the planner by a large unsupported amount.

After an accepted LiDAR update, subsequent wheel/IMU predictions continue to
move the particle set. The high-rate publisher recomputes the current weighted
mean from that predicted set and uses the current state-estimate timestamp;
it does not relabel an old scan result as a new pose. The diagnostics therefore
separate scan age, update latency, measurement application, candidate
uncertainty, ESS, entropy, and correction magnitude.

## Motion-prior separation

The localizer and the coupled EKF have different roles. The estimator publishes
`/state_prediction` from the wheel/IMU propagation stream before any external
map-position update. MCL consumes this topic for particle prediction and scan
registration. An accepted MCL result is emitted once on
`/localization_candidate`; it may then update the fused `/state_estimate`
through the EKF. The corrected `/state_estimate` is therefore an output of the
map update, not the next MCL motion prior:

```text
wheel + IMU -> /state_prediction -> MCL -> /localization_candidate
                                      |
                                      v
                               EKF -> /state_estimate
```

This prevents a wrong local map branch from being fed back into its own motion
model. It is a model-boundary decision, not an additional acceptance threshold.
When external position fusion is disabled, `/state_prediction` and
`/state_estimate` follow the same wheel/IMU estimate, so the baseline behavior
is preserved.

## Candidate evidence versus the propagated state

Posterior compactness is not the same as absolute correctness. In a map with
repeated walls, corridors, or sparse returns, the particle filter can collapse
around a geometrically plausible map alias. A small covariance and moderate
entropy then describe the spread around that alias; they do not prove that the
alias is the robot's physical location.

Each update therefore records a direct comparison against the continuously
propagated `/state_prediction`. This topic is published by the estimator's
wheel/IMU-only motion model and is deliberately not changed by an accepted
MCL correction. Let
$\ell_{\mathrm{prior}}$ be the summed log-likelihood of the current
state-prediction pose and let $\ell_{\mathrm{candidate}}$ be the likelihood of
the MCL candidate, both computed from the same temporal scan window. The
diagnostic evidence gain is

$$
\Delta \ell_k
= \ell_{\mathrm{candidate},k}-\ell_{\mathrm{prior},k}.
$$

A positive value means that the candidate explains the current LiDAR window
better under this likelihood-field model. It is not a metric error and it is
not sufficient by itself to accept a correction: an aliased map location may
also have a positive gain. The gain must be interpreted together with the
correction magnitude, scan diversity, temporal consistency, and independent
evaluation against the physical trajectory.

### Odometry-prior consistency diagnostic

The LiDAR likelihood comparison above is intentionally not treated as a
complete posterior. A map alias can have a better scan likelihood while being
far outside the uncertainty of the continuously propagated state. The adapter
therefore records a covariance-weighted displacement diagnostic. Let

$$
\delta p_k
=\left(
x_k^{\mathrm{candidate}}-x_k^{\mathrm{source}},
y_k^{\mathrm{candidate}}-y_k^{\mathrm{source}}
\right)^{\mathsf T}
$$

and let the approximate displacement covariance be

$$
P_{\Delta,k} =
P_{\mathrm{source},k} + P_{\mathrm{candidate},k}.
$$

The diagnostic Mahalanobis distance is

$$
d_k^2 = \delta p_k^{\mathsf T}
P_{\Delta,k}^{-1}\delta p_k.
$$

Combining this prior-consistency term with the LiDAR evidence gives the
diagnostic Bayesian gain

$$
\Delta \log p_k = \Delta \ell_k - \frac{1}{2}d_k^2.
$$

Here `source` is the current `/state_estimate` position and `candidate` is the
MCL weighted-mean position. The covariance sum is an approximate diagnostic,
because the MCL particles are initialized from the source estimate and are
therefore not statistically independent. The comparison is now used as a
model-based acceptance condition: a candidate with non-positive Bayesian gain
is rolled back. A positive LiDAR gain with a strongly negative Bayesian gain
indicates a candidate that improves the local scan fit but is not supported by
the propagated odometry uncertainty; this is the signature expected from a
compact map alias.

The adapter now enforces the weaker but necessary condition

$$

\Delta \ell_k > 0

$$

before applying a candidate. A candidate with zero or negative gain receives
`candidate_not_better_than_prior`; its particle update is rolled back. This is
not a tuned quality threshold. It prevents an observation with no
discriminative evidence from changing the navigation pose, while leaving the
harder positive-gain map-alias problem visible for later cross-model
validation.

The correction path also requires the model-based posterior comparison to be
positive:

$$
\Delta \ell_k > 0
\quad\land\quad
\Delta \log p_k > 0.
$$

When the LiDAR gain is positive but the Bayesian gain is non-positive, the
candidate receives `candidate_inconsistent_with_prior` and the particle update
is rolled back. This rejects a local map alias because its scan improvement
does not compensate for the displacement implied by the propagated-state
prior. The boundary at zero comes from comparing two model scores; it is not
an empirical correction-distance parameter.

The final bounded-recovery run produced a `/localized_estimate` goal distance
of about `0.029 m`, while the physical `/odom` distance was about `0.082 m`.
The accepted MCL posterior was therefore locally plausible but not globally
validated. The control path remained structurally safer after separating
`control_pose_topic` from the asynchronous navigation pose, while
`/localized_estimate` still exposed a localization-model failure.

This closes the current diagnostic comparison. The next implementation should
change the localization model or representation, not sweep the existing MCL
thresholds: retain multiple global hypotheses, or introduce a pose-graph /
factor-graph layer that keeps wheel, IMU, and LiDAR constraints over time.

## Estimate and diagnostics

The filter publishes a weighted mean pose and a weighted covariance. Heading is
averaged on the unit circle rather than as a linear scalar, so hypotheses near
`-pi` and `+pi` do not create a false heading near zero.

The core also reports:

- effective sample size (ESS), indicating weight degeneracy;
- maximum particle weight;
- normalized weight entropy, indicating concentration versus ambiguity;
- valid beam count;
- the full planar covariance, including position-heading cross terms.

A broad or multi-modal particle set is therefore exposed as uncertainty rather
than hidden behind one apparently precise local minimum.

## Validation policy

The baseline configuration is frozen before map comparisons. The map and
task are the validation variables, not reasons to retune the filter until a
single case succeeds. The same seeds and sensor configuration should be used
for both localization backends on:

- an open room;
- an L corridor;
- a T junction;
- a dead end;
- a symmetric corridor;
- a multi-branch maze.

The comparison must include physical final error, strict terminal success,
false terminal success, confirmation deadlock, localization latency, pose age,
covariance coverage, ESS/entropy, collision, and safety intervention. An
Iterative Closest Point (ICP) baseline can later be compared with this
probabilistic baseline, but it should not replace it or be presented as SLAM.

## Current scope and next implementation step

The first ROS adapter is now available as `mcl_localizer`. It consumes
`/state_estimate`, `/scan`, and `/map`, preserves the source pose timestamp,
and publishes the same `/localized_estimate` contract as the deterministic
matcher. Select it with `localization_backend:=mcl`; the default deterministic
backend remains unchanged.

The adapter supports two explicit prior models. The default
`mcl_initialization_mode:=local` samples a Gaussian around the propagated
state and is appropriate when the map branch is already known. The
`mcl_initialization_mode:=global` experiment samples free map cells and heading
uniformly, so the LiDAR model must select a map hypothesis rather than being
given one by odometry. In global mode the local odometry-prior gate is not used
as an acceptance condition, because odometry is not a valid global prior; the
same covariance and entropy diagnostics remain active. This is a model
comparison, not a parameter sweep.

Global initialization still does not make this node SLAM. It assumes a known
static map and aligned map/odom coordinates, and its output can remain
multi-modal until enough geometric evidence accumulates. A broad posterior is
therefore kept out of navigation by the existing uncertainty contract.
The first adapter assumes the simulator's `map` and `odom` coordinates are
aligned. It does not yet estimate a general map-to-odom transform or perform
global relocalization.

The adapter writes `mcl_update` records containing update latency, valid beam
count, observation-window size, scan age, measurement application, ESS,
normalized entropy, resampling, correction magnitude, the propagated source
pose, both log-likelihoods, and `log_likelihood_gain`. It also writes a
parallel `mcl_belief` record for every attempted scan update. This record is
the offline equivalent of `/localization_belief`; it is emitted for accepted,
ambiguous, stale, and uncertainty-rejected updates alike. Use
`tools/summarize_mcl_diagnostics.py` for the particle-filter summary; the
local-matcher replay tool is intentionally not used for these records. The
controller remains unchanged for the first A/B comparison, keeping
localization-model effects separate from controller and terminal-state logic.

The same records include `source_covariance_xy`,
`correction_mahalanobis_sq`, and `bayesian_log_gain`. These fields support a
model-based decision about whether an absolute correction is compatible with
the propagated state; they are not a new collection of manually tuned
thresholds.

When an update is rejected, `/localized_estimate` deliberately falls back to
the current `/state_estimate` so that an uncertain particle posterior cannot
steer the controller. Evaluation therefore records both the geometric error
of `/localized_estimate` and whether that sample was an independent accepted
localization result. A fallback pose can be close to the goal numerically, but
it must not be described as independent LiDAR confirmation.

An accepted update is also emitted once on `/localization_candidate`. This is
an event stream for the optional coupled pose EKF, not a second high-rate pose
stream. The EKF consumes the candidate only when
`external_position_fusion:=true`, using the candidate's x/y covariance and its
own NIS gate. See [`COUPLED_ESTIMATION.md`](COUPLED_ESTIMATION.md) for the
measurement model and the distinction between known-map localization and SLAM.

The two event streams have deliberately different meanings:

- `/localization_candidate` means that the candidate passed the correction
  gates and was published for downstream fusion.
- `/localization_belief` means that an MCL update was attempted and reports
  posterior quality, whether or not the candidate was applied.

Posterior evidence must not be inferred from correction application alone. A
candidate can be too uncertain or ambiguous to move the navigation pose while
still providing useful evidence for diagnosing the map likelihood, latency,
and terminal-confirmation contract. Conversely, a compact posterior is not
proof of global correctness because a repeated map structure can produce a
plausible alias.

For terminal decisions, an accepted event must not be confused with a current
localized pose. A controller may enable
`require_timestamped_localization_evidence:=true` together with
`require_localization_match_for_goal:=true`. In that mode, only new
`/localization_candidate` events received after entering the goal tolerance
count toward confirmation. The controller checks their monotonic receipt age,
while retaining the candidate's simulation header stamp for offline analysis.
This avoids comparing timestamps from different clock epochs and prevents an
old MCL correction, or an odometry fallback, from proving current physical
arrival.
