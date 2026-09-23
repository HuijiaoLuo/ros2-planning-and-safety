# Visual assets

These visuals are generated from recorded ROS 2 traces with
[`tools/render_baseline_gif.py`](../../tools/render_baseline_gif.py). They are
representative replays, not averages across the full experiment matrix.

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

The GIFs are intentionally paired with the CSV traces in the ignored
`results/` directory. The trace and plan filenames identify the scenario,
backend, and seed used to create each replay.

## Summary figure

[`robustness_summary.png`](robustness_summary.png) is a static summary of the
earlier safety robustness experiments. It is separate from the localization
scenario replays above.
