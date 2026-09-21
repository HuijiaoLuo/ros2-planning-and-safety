# V3 State Estimation: first milestone

V2 freezes the deterministic navigation and safety baseline. V3 starts by
separating the pose used for navigation from Gazebo's ideal `/odom` topic.

## Topics

```text
/wheel_odom ─┐
             ├── heading_estimator ──> /state_estimate
/imu        ─┘                              │
                                           ├── global_planner
                                           ├── path_follower
                                           └── safety_supervisor

/odom ───────────────────────────────> evaluation_logger only
```

The estimator currently keeps `x` and `y` from wheel odometry and estimates
heading from two complementary signals:

- the IMU gyro provides short-term angular motion;
- wheel-odometry yaw provides a slow reference that limits gyro drift.

This is intentionally a transparent heading-fusion baseline, not yet an EKF.
The IMU orientation field is not consumed by the estimator, so Gazebo's ideal
orientation cannot silently become a controller measurement.

## Equations

For an IMU sample at time step `k`:

$$
\theta^{\mathrm{imu}}_{k+1} = \mathrm{wrap}\left(\theta^{\mathrm{imu}}_{k} + \omega_{z,k}\Delta t\right)
$$

The wheel-odometry yaw is used as a slow correction:

$$
\theta^{\mathrm{fused}}_k = \mathrm{wrap}\left(\theta^{\mathrm{fused}}_k + \lambda\,\mathrm{wrap}\left(\theta^{\mathrm{wheel}}_k - \theta^{\mathrm{fused}}_k\right)\right)
$$

The default $\lambda = 0.02$ gives the gyro the short-term role and wheel yaw a
small drift-correction role. The value is a first experimental parameter, not
a statistical covariance estimate.

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

`evaluation_logger` continues to use `/odom` as evaluation-only ground truth.
The first validation should compare `/odom`, `/wheel_odom`, and
`/state_estimate` while checking that the controller remains on `/odom`.

## Next experiments

1. Validate the deterministic IMU bridge and heading estimate.
2. Add a configurable gyro bias and white noise model.
3. Add wheel-slip perturbation and report `RMSE_theta` and pose RMSE.
4. Replace the complementary fusion with a covariance-aware EKF only after
   the error mechanisms are visible in the simpler estimator.
