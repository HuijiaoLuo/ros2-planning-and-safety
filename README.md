# Classical Path Planning and Sensor-Aware Safety

[![CI](https://github.com/HuijiaoLuo/ros2-planning-and-safety/actions/workflows/ci.yml/badge.svg)](https://github.com/HuijiaoLuo/ros2-planning-and-safety/actions/workflows/ci.yml)

An incremental robotics portfolio project built around a differential-drive mobile robot.

The central question is:

> How should a robot turn a map and imperfect sensor measurements into safe motion commands?

The project connects classical algorithms, robot motion, ROS2 integration, and quantitative validation.

## System idea

```text
occupancy grid
      ↓
global path planner
      ↓
geometric path
      ↓
path follower
      ↓
raw velocity command
      ↓
sensor-aware safety supervisor
      ↓
safe velocity command
```

The architecture separates three responsibilities:

- The planner decides where the robot should go.
- The path follower converts a path into $(v, \omega)$ commands.
- The safety supervisor can override unsafe commands using LiDAR and stopping-distance estimates.

## Current implementation

The current milestones are a standalone planning core and the first ROS2 control layer.

The planning core is implemented in both Python and C++:

- BFS
- Dijkstra
- Greedy Best-First Search
- A*
- DFS with backtracking
- path reconstruction through parent pointers
- weighted grid costs
- expanded-node and runtime benchmarks

- ROS2 package `robotics_nav` contains the map publisher, A* global planner,
  path follower, and sensor-aware safety supervisor.
- Gazebo integration provides ground-truth `/odom`, wheel `/wheel_odom`, and
  `/scan` through `ros_gz_bridge`.
- The first path-following configuration uses a `0.10 m` lookahead and a
  `0.20 m/s` speed limit. It also scales forward speed with heading error and
  rotates in place for errors above `30°` to reduce corner cutting near
  inflated obstacles. The follower also advances through the ordered path
  prefix instead of selecting a geometrically nearer point beyond a detour.
  Within `0.60 m` of the final goal, it switches to a damped final-approach
  controller that tracks the endpoint directly, limits angular speed to
  `0.60 rad/s`, and applies a small heading deadband.

## Search algorithms

For a 4-neighbor grid, the default A* heuristic is Manhattan distance:

$$
h(n) = |x - x_g| + |y - y_g|
$$

A* evaluates:

$$
f(n) = g(n) + h(n)
$$

where $g(n)$ is the cost already paid and $h(n)$ estimates the remaining cost.

- BFS is optimal for equal edge costs.
- Dijkstra is the weighted shortest-path baseline.
- Greedy Best-First Search uses only $h(n)$ and is not generally optimal.
- A* is optimal when the heuristic is admissible under the usual graph-search assumptions.
- Dijkstra is A* with $h(n) = 0$.

With a binary heap, Dijkstra and A* have worst-case complexity:

$$
O((V + E)\log V)
$$

On a regular grid, $E$ is proportional to $V$, giving approximately $O(V\log V)$.

## Setup

The standard Conda workflow is:

```bash
conda env create -f environment.yml
conda activate robotics-portfolio
```

The environment pins Python 3.11. No editable package installation is required when running from the repository root.

## Python demo

```bash
python -m robotics_planning.demo
python -m unittest discover -s tests -v
```

The demo renders the explored cells and final path for each algorithm, followed by a benchmark table.

## C++ planning core

The C++17 implementation is under `cpp/`. It uses a flat cell-indexed grid so the data layout can later be reused by OpenMP or CUDA kernels.

Build with CMake and the Visual Studio generator:

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build --config Release
ctest --test-dir cpp/build -C Release --output-on-failure
```

Run the demo:

```bash
./cpp/build/Release/planning_demo.exe
```

The OpenMP option is reserved for future kernels:

```bash
cmake -S cpp -B cpp/build-openmp \
  -DROBOTICS_PLANNING_ENABLE_OPENMP=ON
cmake --build cpp/build-openmp --config Release
```

The current search kernels remain serial. A single A* search has frontier-ordering dependencies, so useful first parallel targets include:

- batches of independent map queries;
- Monte Carlo noise and latency experiments;
- wavefront-style BFS;
- GPU-friendly cost propagation.

## ROS2 environment and first package

ROS2 Jazzy and Gazebo Harmonic are installed natively inside WSL2 Ubuntu 24.04. The
Windows Conda environment is intentionally not used by ROS2.

Load ROS2 in each new WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

Build the ROS2 workspace from the repository root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

The first packages are deliberately small. They contain:

- `path_follower`: `/plan` plus `/odom` to `/cmd_vel_raw`;
- `waypoint_controller`: direct-goal baseline kept for comparison;
- `safety_supervisor`: `/scan` plus `/cmd_vel_raw` to `/cmd_vel`;
- `static_map_publisher`: deterministic `nav_msgs/OccupancyGrid` on `/map`;
- `global_planner`: A* over `/map` and `/odom`, publishing `/plan`;
- `robotics_sim`: a Gazebo Harmonic world, differential-drive robot, obstacle,
  ground-truth `/odom`, wheel `/wheel_odom`, and ROS-Gazebo bridge.

Run the package after building:

```bash
ros2 launch robotics_nav bringup.launch.py
```

The controller and safety nodes will hold the robot stopped until both odometry
and LiDAR data are available. Start the first simulation with:

```bash
ros2 launch robotics_sim sim.launch.py
```

This launches Gazebo, bridges `/cmd_vel`, `/odom`, `/wheel_odom`, and `/scan`, and starts the
two control nodes. The world contains one obstacle directly along the initial
goal direction so the safety stop can be observed.

The planner output is now connected to `path_follower`. The direct
`waypoint_controller` remains available as a baseline, but is not started by the
default bringup launch.

The detailed method, data flow, equations, and validation protocol are documented
in [METHOD_TECH.md](METHOD_TECH.md).

## ROS2 architecture

```text
/map
  ↓
global_planner  →  nav_msgs/Path
                         ↓
                    path_follower
                         ↓
                    /cmd_vel_raw
                         ↓
               safety_supervisor  ←  /scan
                         ↓
                      /cmd_vel
```

The safety layer uses a speed-dependent stopping-distance envelope:

$$
d_{stop} = \frac{v^2}{2a_{max}} + v\tau + d_{margin}
$$

It also applies a `0.35 m` minimum clearance aligned with the planner's inflated
robot radius, plus a hysteresis band to
avoid stop/release chatter near the threshold. When forward motion is blocked,
it owns the recovery direction for the duration of the blocked state. It
selects the side with more
measured LiDAR clearance, even if the path follower requests the opposite
turn, and holds that direction until the front clearance exceeds the release
threshold. The safety layer owns the angular command for the complete blocked
episode; it does not forward a competing path-follower turn. Later experiments will compare this layer with noisy sensors, control
latency, and wheel-odometry drift. When
the path follower intentionally commands pure rotation (`linear.x=0`) during a
blocked episode, the safety layer keeps ownership of the recovery turn. In the
simulation MVP, it also stops if the estimated roll or pitch
exceeds `10°`, preventing a tipped robot from continuing to receive commands.

## Roadmap

### V0 — Ideal motion

- unicycle/differential-drive kinematics;
- waypoint and path following;
- known pose and known goal.

### V1 — Classical planning

- occupancy grid;
- BFS, Dijkstra, Greedy, A*;
- same-map benchmark and visualization.

### V2 — ROS2/Gazebo integration

- `/map`, `/odom`, `/scan`;
- custom waypoint controller and safety supervisor;
- `nav_msgs/Path`;
- differential-drive simulation.

### V3 — Sensor-aware safety

- LiDAR safety supervisor;
- fixed threshold versus physics-informed stopping distance;
- minimum clearance and collision metrics;
- reactive side-clearance recovery.

### V4 — Validation under uncertainty

- LiDAR noise;
- odometry drift;
- control latency;
- actuator saturation;
- Monte Carlo evaluation.

### V5 — Extensions

- richer reactive obstacle avoidance and local planning;
- IMU and wheel-odometry fusion;
- SLAM;
- Nav2;
- camera-based safety events.

## Repository structure

```text
.
├── README.md
├── environment.yml
├── pyproject.toml
├── robotics_planning/
│   ├── grid.py
│   ├── planners.py
│   ├── benchmark.py
│   └── demo.py
├── cpp/
│   ├── CMakeLists.txt
│   ├── include/robotics_planning/
│   ├── src/
│   ├── apps/
│   └── tests/
├── ros2_ws/
│   └── src/
│       ├── robotics_nav/
│       │   ├── launch/
│       │   ├── robotics_nav/
│       │   ├── package.xml
│       │   └── setup.py
│       └── robotics_sim/
│           ├── launch/
│           ├── robotics_sim/
│           ├── worlds/
│           ├── package.xml
│           └── setup.py
└── tests/
    └── test_planners.py
```
