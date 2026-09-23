# Classical Path Planning and Sensor-Aware Safety

[![CI](https://github.com/HuijiaoLuo/ros2-planning-and-safety/actions/workflows/ci.yml/badge.svg)](https://github.com/HuijiaoLuo/ros2-planning-and-safety/actions/workflows/ci.yml)

A robotics portfolio project that connects classical path planning with a
simulated differential-drive robot.

The central question is:

> How should a robot turn map and sensor measurements into safe motion commands?

At the system level, this is a ROS 2 robotics validation framework spanning
classical navigation, closed-loop control, LiDAR safety, wheel/IMU EKF state
estimation, uncertainty calibration, and failure diagnostics.

<p align="center">
  <img src="docs/assets/baseline_navigation.gif" alt="Baseline A* navigation with a differential-drive robot" width="820">
</p>

<p align="center"><em>Baseline run: A* path planning, closed-loop path following, and LiDAR-aware safety supervision.</em></p>

<p align="center"><sub>Gray: occupied cells · dashed blue: A* path · red: executed trajectory · cyan: robot · orange: goal. The overlay reports time, front clearance, and safety override state.</sub></p>

### How to read the demo

`front clearance` is the closest valid LiDAR return within `±60°` of the
robot's forward direction. It is measured from the LiDAR origin to the first
obstacle surface, not from the robot's outer body. In this simulation the
LiDAR is centered in a `0.50 m`-long base, so for a flat wall directly ahead:

```text
approximate body-to-wall gap = displayed front clearance - 0.25 m
```

The safety supervisor compares this sensor measurement with the configured
clearance threshold and the speed-dependent stopping envelope. See
[`METHOD_TECH.md`](METHOD_TECH.md) for the complete measurement model.

## Current status

The reproducible planning and safety baseline is frozen. The repository also
contains explicitly separated state-estimation
and map-localization workstreams; these are not silently presented as validated
navigation replacements.

The current implementation includes:

- Python and C++ implementations of DFS, BFS, Dijkstra, Greedy Best-First, and A*;
- path, runtime, and expanded-node benchmarks;
- a ROS2 Jazzy + Gazebo differential-drive simulation;
- occupancy-grid A* planning and `nav_msgs/Path` publishing;
- path following through `/cmd_vel_raw`;
- LiDAR-based safety supervision through `/cmd_vel`;
- physics-informed stopping distance, clearance hysteresis, and recovery turning;
- Python unit tests, C++ tests, and GitHub Actions CI.
- an offline planner-scaling benchmark for map-size, density, and heuristic sweeps;
- a read-only ROS2 evaluation logger for closed-loop metrics;
- a shared launch-configurable goal tolerance and latched terminal stop in the
  path follower;
- a heading-estimation diagnostic using `/wheel_odom` and `/imu` while
  navigation remains on the validated `/odom` baseline;
- an evaluation-only estimator logger reporting wheel and fused-pose RMSE;
- seeded gyro bias/noise, wheel-slip, adaptive fusion, and propagated-position
  experiments with configuration values recorded in CSV output;
- a gated LiDAR-to-static-map localizer with persistent `map → odom` state and
  diagnostic match-status topics;
- a covariance-aware pose EKF with x/y/yaw covariance, NIS gating, and
  wheel-measurement acceptance diagnostics;

The robot has been tested in simulation from the start position to the goal at
approximately `(2.0, 0.0)`, with a final position error within the configured
`0.05 m` tolerance.

### Robustness snapshot

The evaluation suite measures goal completion, time-to-goal, physical LiDAR
clearance, safety-layer intervention, and collision status. The summary below
shows the main planning-radius boundary on the current map:

| Configuration | Success | Mean time-to-goal (successful runs) | Mean measured clearance | Safety override ratio |
| --- | :---: | ---: | ---: | ---: |
| Baseline: radius `0.35 m`, no noise | 1/1 | 65.04 s | 0.521 m | 0.000 |
| Noise `0.03 m`, radius `0.40 m` | 1/3 | 462.93 s (n=1) | 0.525 m | 0.683 |
| Noise `0.03 m`, radius `0.41 m` | 3/3 | 71.00 s | 0.632 m | 0.000 |
| Noise `0.05 m`, radius `0.40 m` | 0/3 | -- | 0.573 m | 0.764 |
| Noise `0.05 m`, radius `0.41 m` | 3/3 | 70.49 s | 0.632 m | 0.002 |
| Delay `0.30 s` + noise `0.05 m`, radius `0.41 m` | 3/3 | 69.38 s | 0.631 m | 0.008 |

The `0.40 m` configuration is a marginal boundary case: its single
successful run took much longer and required sustained safety intervention.
The time-to-goal value above is therefore calculated from one successful
seed, not averaged over all three trials. Under the tested seeds and
uncertainty settings, `planning_radius=0.40 m` exhibited boundary behavior,
while `0.41 m` was the smallest tested radius that achieved consistent
success. This is an empirical result, not a statistical robustness guarantee.
All listed runs were collision-free.

![Robustness summary](docs/assets/robustness_summary.png)

The evaluation logger records `termination_reason` (`goal_reached`,
`goal_confirmation_failed`, `experiment_timeout`, or `manual_interrupt`) and
supports an explicit `experiment_timeout_s` launch parameter. The top-level
`success` field now means terminal physical completion: the path follower
published `goal_reached_latched`, the logger ended with `goal_reached`, and
the final complete `/odom` sample remained within the goal tolerance.
Historical tolerance crossings are reported separately as
`ground_truth_goal_reached_any_time`, while
`ground_truth_final_within_goal_tolerance` describes the final complete sample.
This prevents a run that briefly entered the tolerance and later timed out
from being counted as a successful completion. A controller latch without
final physical agreement is reported separately.

## System architecture

```text
/map
  ↓
global_planner (A*)
  ↓ /plan
path_follower
  ↓ /cmd_vel_raw
safety_supervisor ← /scan, /odom
  ↓ /cmd_vel
Gazebo differential-drive robot
```

The green goal marker is visible in Gazebo but excluded from the LiDAR
visibility mask, so it is not treated as a physical obstacle.

The baseline uses `/odom` for navigation. The state-estimation workstream
additionally provides:

```text
/wheel_odom + /imu → heading_estimator → /state_estimate
```

An optional map-localization experiment adds a known-map position correction:

```text
/state_estimate + /scan + /map
        ↓
 lidar_localizer → persistent map→odom correction → /localized_estimate
```

Passing `navigation_pose_topic:=/state_estimate` switches the planner,
controller, and safety layer to the estimated pose, but this full estimated-
pose navigation mode is not part of the validated baseline. The estimated
pose can enter the goal tolerance while the physical `/odom` pose is still
outside it. The LiDAR localizer is an opt-in experiment and is not a
validated SLAM replacement. It keeps a stateful `map→odom` correction and can
broadcast it on TF, but remains diagnostic-only while its asynchronous match
latency and acceptance gates are being validated. See
[`docs/STATE_ESTIMATION.md`](docs/STATE_ESTIMATION.md),
[`docs/POSE_EKF.md`](docs/POSE_EKF.md), and
[`docs/LOCALIZATION.md`](docs/LOCALIZATION.md).

The static occupancy grid is published in the `map` frame; in the baseline,
`map` and Gazebo odometry are numerically aligned, while the optional localizer
provides the standard `map → odom → base_link` transform.

### Latest estimation evidence

The current calibrated pose-EKF configuration uses propagated wheel speed,
explicit slip uncertainty, wheel-yaw bias, a wheel-yaw NIS gate, and covariance
logging. The latest localized-navigation trial finished with physical `/odom`
error `0.0762 m`, independent `/state_estimate` error `0.0664 m`, and
`/localized_estimate` error `0.0325 m`. Three LiDAR corrections were applied,
with a maximum smoothed correction of `0.0225 m`; the run timed out without
collision or sustained safety recovery.

The result is interpreted as a model and integration diagnostic, not as a
reason to keep tuning isolated weights. It shows a real estimated-goal versus
physical-goal gap: the localizer improved the reported pose but did not yet
provide a trustworthy closed-loop navigation source. The calibrated EKF,
planner, and safety configuration are therefore frozen for the next multi-seed
failure-propagation study. Existing replay, trace-diagnosis, observability,
and score-consistency tools are used before any further algorithm change.

The frozen numerical values are not claimed to be map-independent. A new map
or maze is a validation input, not a reason to retune until it succeeds. The
generalization test keeps the estimator, matcher, controller, and safety
configuration fixed while varying only the map and task, then classifies the
failure layer from the recorded evidence.

The next diagnostic layer records the temporal chain behind a terminal
decision: source pose stamps, LiDAR stamps, logger receipt ages, controller
events for entering and latching the goal tolerance, and the controller pose
age embedded in each event. This makes it possible to distinguish estimator
error, localization latency, stale state consumption, and premature terminal
logic without changing the frozen estimator or matcher configuration.

### State-estimation and localization status

The estimator compares `/wheel_odom`, pure gyro integration, and
`/state_estimate` against `/odom` without feeding `/odom` into the estimator.
An estimated-pose navigation check reached its configured tolerance, but the
physical `/odom` pose remained outside the goal tolerance. Evaluation therefore
reports historical entry, final-sample proximity, and controller completion
separately; these are not interchangeable success definitions.

The LiDAR localizer remains diagnostic-only. Its score alternatives and
covariance/freshness gates help explain ambiguous or stale candidates, but they
do not turn the bounded local matcher into SLAM. The covariance-aware pose EKF
is also diagnostic-only until its physical error, covariance calibration, NIS
values, and measurement rejection behavior are validated across fixed seeds.
The controller exposes this timing evidence and now supports an optional finite
`goal_confirmation_timeout_s`. When enabled, failure to obtain the required
fresh LiDAR/independent-estimate confirmation produces a
`goal_confirmation_timeout` event and transitions the controller into a
bounded low-speed `FINAL_APPROACH` state. It does not latch success; the robot
must leave and re-enter the confirmation region before another confirmation
attempt. The default `0.0` keeps the legacy wait behavior for controlled
comparisons. An optional positive `goal_confirmation_max_attempts` bounds the
number of failed recovery cycles; exhaustion enters `GOAL_UNCONFIRMED`, stops
safely, and reports `goal_confirmation_failed` instead of looping indefinitely.
Confirmation also requires the consumed navigation pose to be recent and the
estimated planar speed to be below the configured confirmation limit, so a
moving or stale estimate cannot directly latch the goal.

### Probabilistic localization

An experimental known-map particle-filter localizer is available as an
alternative to the deterministic local matcher. It maintains multiple pose
hypotheses and reports covariance and ambiguity; see
[`MCL_LOCALIZATION.md`](docs/MCL_LOCALIZATION.md) for the model and limitations.

## Quick start

### Python planning core

```bash
conda env create -f environment.yml
conda activate robotics-portfolio

python -m robotics_planning.demo
python -m unittest discover -s tests -v

python tools/planner_scaling_benchmark.py \
  --sizes 20,50 \
  --densities 0,0.1 \
  --seed-count 2

python tools/summarize_estimation.py \
  --glob "results/*_metrics.csv" \
  --output results/estimation_summary.csv
```

### C++ planning core

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build --config Release
ctest --test-dir cpp/build -C Release --output-on-failure
./cpp/build/Release/planning_demo.exe
```

### ROS2 simulation in WSL2

ROS2 Jazzy and Gazebo Harmonic are installed in WSL2 Ubuntu. In a WSL2
terminal:

```bash
cd /mnt/e/HPC_simulation_porfolio/Robotics/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robotics_sim sim.launch.py
```

In another WSL2 terminal, the LiDAR diagnostic helper can be run with:

```bash
cd /mnt/e/HPC_simulation_porfolio/Robotics
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 tools/scan_debug.py
```

`scan_debug.py` only reads `/scan` and reports minimum front, left, and right
sector distances. It does not publish commands or modify the simulation.

To render a baseline navigation GIF from the optional CSV trace:

```bash
python tools/render_baseline_gif.py \
  --trace results/baseline_trace.csv \
  --plan results/baseline_plan.csv \
  --map results/baseline_map.csv \
  --output docs/assets/baseline_navigation.gif
```

To save one closed-loop evaluation row when the simulation is stopped:

```bash
ros2 launch robotics_sim sim.launch.py \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/closed_loop_metrics.csv
```

For bounded robustness experiments, let the evaluation logger close the full
ROS2/Gazebo launch automatically:

```bash
ros2 launch robotics_sim sim.launch.py \
  experiment_timeout_s:=120.0 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/robustness_case.csv
```

## Repository layout

```text
robotics_planning/     Python planners and benchmarks
cpp/                   C++17 planners, demo, and tests
ros2_ws/src/          ROS2 navigation and simulation packages
tools/                Diagnostic scripts
tests/                Python unit tests
.github/workflows/    GitHub Actions CI
METHOD_TECH.md        Detailed methods, equations, diagnostics, and roadmap
```

## Documentation

See [METHOD_TECH.md](METHOD_TECH.md) for:

- differential-drive and controller equations;
- the overall system equations, planner, safety supervisor, and validation protocol;
- ROS2 topic flow and launch sequence;
- parameter meanings and validation protocol;
- current limitations and the roadmap toward uncertainty, HPC, SLAM, Nav2,
  and vision.

For focused estimation documentation, see:

- [State Estimation](docs/STATE_ESTIMATION.md) for transparent wheel/IMU fusion;
- [Pose EKF](docs/POSE_EKF.md) for covariance propagation and NIS diagnostics;
- [Map-based Localization](docs/LOCALIZATION.md) for the gated LiDAR matcher.

## License

This project is released under the [Apache License 2.0](LICENSE).
