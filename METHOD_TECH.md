# Method / Tech

This document explains the engineering logic of the project. It answers two questions:

1. What problem is the robot solving?
2. How do the equations, ROS2 nodes, Gazebo model, and validation experiments fit together?

The project is deliberately incremental. At each stage, one new uncertainty is added while the rest of the system remains simple enough to reason about.

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

The important engineering chain is:

$$
\text{physical model}
\rightarrow
\text{measurement}
\rightarrow
\text{state/control model}
\rightarrow
\text{decision}
\rightarrow
\text{validation}
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
v_L = v - \frac{L}{2}\omega,
\qquad
v_R = v + \frac{L}{2}\omega
$$

where $L$ is the distance between the wheel contact points. Gazebo's DiffDrive system uses the two wheel joints, wheel radius, and wheel separation to apply this model to the simulated robot.

## 4. ROS2 computation graph

~~~
                         /odom
                           │
                           ▼
                 ┌────────────────────┐
                 │ waypoint_controller│
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
| /scan | sensor_msgs/msg/LaserScan | Gazebo → ROS2 | LiDAR range measurements |
| /cmd_vel_raw | geometry_msgs/msg/Twist | Controller → safety layer | Unchecked motion request |
| /cmd_vel | geometry_msgs/msg/Twist | Safety layer → Gazebo | Command allowed to reach robot |

The distinction between /cmd_vel_raw and /cmd_vel is important. It makes the safety layer independently testable and gives the system a clear enforcement point: Gazebo never receives the controller's command directly.

The simulation deliberately exposes two pose sources. The ideal-navigation
MVP uses `/odom`, generated from Gazebo's true model pose, so planning and
control are not invalidated by wheel slip before the basic loop is verified.
The DiffDrive plugin publishes `/wheel_odom` separately. Comparing these two
topics makes collision-induced odometry error observable and creates a clean
transition to the later noisy-localization experiments.

The simulated actuator also has explicit linear and angular velocity and
acceleration limits. These keep the ideal model physically stable when the
controller changes commands; they are separate from the LiDAR safety
supervisor and do not replace its stopping envelope.

The odometry and LiDAR subscriptions use ROS2's sensor-data QoS profile. This
matters because Gazebo sensor bridges commonly publish with best-effort,
volatile QoS; a default reliable subscription may be incompatible and receive
no callbacks even when the topic appears in the graph.

## 5. Waypoint controller

The controller reads the current pose from /odom and uses a fixed goal.

Distance to goal:

$$
e_d = \sqrt{(x_g-x)^2 + (y_g-y)^2}
$$

Desired heading:

$$
\theta_g = \mathrm{atan2}(y_g-y, x_g-x)
$$

Wrapped heading error:

$$
e_\theta = \mathrm{wrap}(\theta_g-\theta)
$$

The proportional command is:

$$
v = \mathrm{clip}(K_d e_d, 0, v_{max})
$$

$$
\omega = \mathrm{clip}(K_\theta e_\theta,
-\omega_{max}, \omega_{max})
$$

There is one practical rule in addition to the equations:

~~~
large heading error → rotate in place
small heading error  → drive and rotate simultaneously
goal within tolerance → publish zero velocity
~~~

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

The first map uses `odom` as its frame and is aligned with the Gazebo world. This
avoids introducing TF and localization before the planner itself has been
validated. Later, the map frame will be separated from `odom` and transformed
through the standard `map → odom → base_link` chain.

## 7. Sensor-aware safety supervisor

The supervisor examines LiDAR rays in a forward angular sector. It obtains the closest valid forward measurement d_front and compares it with a speed-dependent stopping envelope.

$$
d_{stop} = \frac{v^2}{2a_{max}} + v\tau + d_{margin}
$$

The terms represent:

- v: requested forward speed;
- a_max: assumed available deceleration;
- tau: sensing and command latency;
- d_margin: additional uncertainty margin.

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

## 9. Launch sequence

When running:

~~~bash
ros2 launch robotics_sim sim.launch.py
~~~

ROS2 starts the processes in parallel:

1. Gazebo loads the SDF world and starts physics.
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

`path_efficiency` is reported only for successful runs. For an incomplete
run, the robot has not traversed the full planned route, so dividing the
initial full-path length by partial travelled distance would produce a
misleading value greater than one.

The CSV also reports `path_length_ratio`, defined as

$$
\mathrm{path\_length\_ratio}=\frac{L_{\mathrm{travelled}}}{L_{\mathrm{planned}}}
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
  safety_margin:=0.15 \
  recovery_timeout_s:=8.0 \
  planning_radius_m:=0.35 \
  evaluation_output:=/mnt/e/HPC_simulation_porfolio/Robotics/results/closed_loop_metrics.csv
```

This makes parameter sweeps reproducible: each result row contains both the
measured outcomes and the safety configuration that produced them.

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

## 12. Current status and roadmap

The current ROS2 milestone includes:

- a deterministic occupancy-grid publisher;
- an A* global planner publishing `nav_msgs/Path`;
- ordered path following for the differential-drive robot;
- LiDAR-based safety supervision with a speed-dependent stopping envelope;
- clearance hysteresis, latched recovery turning, and a tilt guard;
- a Gazebo goal marker that remains visible but is excluded from the LiDAR mask.

The next layers are intentionally separated so that each experiment remains
interpretable:

- V3: richer reactive obstacle avoidance and local planning;
- V4: LiDAR noise, odometry drift, actuator saturation, and control latency;
- V5: Monte Carlo validation of collision rate, clearance, and intervention count;
- V6: IMU and wheel-odometry fusion with covariance handling;
- V7: SLAM and Nav2 integration;
- V8: camera-based safety events and perception/sensor fusion.

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
\text{measurement}
\rightarrow
\text{model}
\rightarrow
\text{decision}
\rightarrow
\text{quantitative validation}
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
