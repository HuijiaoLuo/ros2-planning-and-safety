# Visual assets

These visuals are generated from recorded ROS 2 traces with
[`tools/render_baseline_gif.py`](../../tools/render_baseline_gif.py). On a
Windows environment without the Python plotting packages, the equivalent
fallback is [`tools/render_navigation_gif.ps1`](../../tools/render_navigation_gif.ps1).
They are representative replays, not averages across the full experiment
matrix.

Each navigation GIF uses the same visual language:

- gray cells: occupied cells in the selected map profile;
- blue dashed line: the planned A* path;
- red line: the executed `/odom` trajectory;
- cyan marker: the simulated robot;
- yellow star: the goal.

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
