# Visual assets

These visuals are generated from recorded ROS 2 traces with the single
canonical renderer [`tools/render_baseline_gif.py`](../../tools/render_baseline_gif.py).
Run [`tools/render_navigation_comparison.sh`](../../tools/render_navigation_comparison.sh)
to regenerate all four homepage comparison GIFs with identical map geometry,
axis limits, labels, and legend styling. The PowerShell
[`tools/render_navigation_gif.ps1`](../../tools/render_navigation_gif.ps1) is a
dependency-free fallback for individual previews, not the canonical homepage
comparison renderer.

Each navigation GIF uses the same visual language:

- gray cells: occupied cells in the selected map profile;
- blue dashed line: the planned A* path;
- red line: the executed `/odom` trajectory;
- cyan marker: the simulated robot;
- yellow star: the goal.

The homepage comparison is arranged by scenario, not by map variant: the top
row is `baseline_obstacle`, the bottom row is `l_corridor`; the left column is
the original fixed-fusion run and the right column is the validated EKF run.
The red line is the physical `/odom` trajectory, while the blue dashed line is
the planned path. The GIF does not draw `/state_estimate` as a second line;
the estimator improvement is reflected in the robot's closed-loop trajectory.

## Navigation replays

| Asset | Map profile and interpretation |
| --- | --- |
| [`baseline_navigation.gif`](baseline_navigation.gif) | Original baseline replay used by the project overview. |
| [`baseline_obstacle_navigation.gif`](baseline_obstacle_navigation.gif) | Single central obstacle. The v4 seed-0 replay reaches the estimated goal, but the physical `/odom` endpoint remains about 0.19 m away. |
| [`l_corridor_navigation.gif`](l_corridor_navigation.gif) | L-shaped walls requiring a detour. It shows the same systematic state-estimation drift under a more constrained route. |
| [`symmetric_corridor_navigation.gif`](symmetric_corridor_navigation.gif) | Two parallel walls forming a corridor. This profile is the cleanest of the three matrix scenes and reaches the physical goal tolerance in the recorded v4 seed-0 replay. |

The validated estimator replays use the same maps and seed-0 trajectories as
the left-column GIFs, but with `fusion_mode:=ekf`,
`position_mode:=propagated`, `gyro_bias_mode:=fixed`, and
`wheel_yaw_noise_std_rad:=0.20`:

| Asset | Interpretation |
| --- | --- |
| [`baseline_obstacle_ekf_navigation.gif`](baseline_obstacle_ekf_navigation.gif) | Baseline obstacle route after the wheel/state-estimation fix; the physical endpoint is within about `0.04 m` of the goal. |
| [`l_corridor_ekf_navigation.gif`](l_corridor_ekf_navigation.gif) | L-corridor route after the fix; the physical endpoint is within about `0.02 m` of the goal. |

The GIFs are intentionally paired with the CSV traces in the ignored
`results/` directory. The trace and plan filenames identify the scenario,
backend, and seed used to create each replay.

## Summary figure

[`robustness_summary.png`](robustness_summary.png) is a static summary of the
earlier safety robustness experiments. It is separate from the localization
scenario replays above.
