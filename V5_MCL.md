# V5: probabilistic known-map localization

V5 replaces the V4 single-candidate local grid correction with a Monte Carlo
localization backend. The map is still known and static; V5 is therefore not
SLAM. The goal is to test whether a multi-hypothesis localization model handles
map ambiguity more honestly and more generally than a local optimizer.

## Why this is the next model

The last V4 run (`v4_goal_fsm_heading_lock_v2`) produced three confirmation
timeouts and three final-approach re-entries. Only three LiDAR matches were
accepted, while the local matcher reported many ambiguous or uncertainty-gated
candidates. The physical final error was about `0.136 m`, despite the estimated
poses entering the goal region. This is evidence of a model limitation, not a
reason to keep sweeping score thresholds.

V4 selects one nearby minimum:

```text
wheel/IMU pose + local scan search -> one corrected pose
```

V5 maintains a distribution:

```text
wheel/IMU motion prior -> particle prediction
LiDAR + occupancy map  -> particle likelihoods
                         -> normalization and resampling
                         -> weighted pose, covariance, and ambiguity metrics
```

The first implementation is in
`robotics_nav/mcl_localization.py`. It is dependency-free so the motion model,
likelihood-field model, systematic resampling, circular heading mean, and
covariance calculation can be tested without launching Gazebo.

## State and motion model

Each particle represents a planar map-frame hypothesis:

```text
p_i = (x_i, y_i, theta_i, w_i)
```

For a wheel/IMU increment `(Delta s, Delta theta)`, the particle uses the
midpoint differential-drive propagation:

```text
theta_mid = theta_i + 0.5 Delta theta
x_i'      = x_i + Delta s cos(theta_mid)
y_i'      = y_i + Delta s sin(theta_mid)
theta_i'  = wrap(theta_i + Delta theta)
```

Independent motion noise is sampled per particle. This is process uncertainty,
not a parameter chosen to make one map succeed.

## LiDAR likelihood

For each valid range return, a particle transforms the measured endpoint into
the map frame. The endpoint is scored against the nearest occupied-cell centre
using a Gaussian likelihood field:

```text
p(z_i | x, M) = exp(-0.5 d_i(x, M)^2 / sigma_range^2) + epsilon
```

The particle weight is updated by the mean log likelihood over valid beams.
Invalid and out-of-range measurements do not create artificial evidence.

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

The V5 baseline configuration is frozen before map comparisons. The map and
task are the validation variables, not reasons to retune the filter until a
single case succeeds. The same seeds and sensor configuration should be used
for V4 and V5 on:

- an open room;
- an L corridor;
- a T junction;
- a dead end;
- a symmetric corridor;
- a multi-branch maze.

The comparison must include physical final error, strict terminal success,
false terminal success, confirmation deadlock, localization latency, pose age,
covariance coverage, ESS/entropy, collision, and safety intervention. ICP can
later be added as a second localization baseline, but it should not replace
this probabilistic baseline or be presented as SLAM.

## Current scope and next implementation step

The current V5 milestone is the tested mathematical core. The next step is a
ROS adapter that consumes `/state_estimate`, `/scan`, and `/map`, preserves
source timestamps, and publishes the same `/localized_estimate` contract as
V4. The controller will remain unchanged for the first A/B comparison. This
keeps localization-model effects separate from controller and terminal-state
logic.
