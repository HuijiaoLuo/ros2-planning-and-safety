# Classical Path Planning and Sensor-Aware Safety

[![CI](https://github.com/HuijiaoLuo/ros2-planning-and-safety/actions/workflows/ci.yml/badge.svg)](https://github.com/HuijiaoLuo/ros2-planning-and-safety/actions/workflows/ci.yml)

A robotics portfolio project that connects classical path planning with a
simulated differential-drive robot.

The central question is:

> How should a robot turn map and sensor measurements into safe motion commands?

<p align="center">
  <img src="docs/assets/baseline_navigation.gif" alt="Baseline A* navigation with a differential-drive robot" width="820">
</p>

<p align="center"><em>Baseline run: A* path planning, closed-loop path following, and LiDAR-aware safety supervision.</em></p>

<p align="center"><sub>Gray: occupied cells · dashed blue: A* path · red: executed trajectory · cyan: robot · orange: goal. The overlay reports time, front clearance, and safety override state.</sub></p>

## Current status

The current milestone includes:

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

The robot has been tested in simulation from the start position to the goal at
approximately `(2.0, 0.0)`, with a final position error within the configured
`0.05 m` tolerance.

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
- planner and safety-supervisor logic;
- ROS2 topic flow and launch sequence;
- parameter meanings and validation protocol;
- current limitations and the roadmap toward uncertainty, HPC, SLAM, Nav2,
  and vision.

## License

This project is released under the [Apache License 2.0](LICENSE).
