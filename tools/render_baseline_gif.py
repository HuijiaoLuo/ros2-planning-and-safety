#!/usr/bin/env python3
"""Render a closed-loop CSV trace as a compact portfolio GIF."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def read_csv(path: Path) -> list[dict[str, str]]:
    """Read a trace, plan, or map CSV as dictionaries keyed by column name."""
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def optional_float(row: dict[str, str], key: str) -> float | None:
    """Return a nullable numeric field without treating missing data as zero."""
    value = row.get(key, "")
    return None if value in ("", "None", "nan") else float(value)


def frame_indices(count: int, maximum: int) -> list[int]:
    """Choose evenly spaced trace rows while preserving first and last frames.

    Long traces can contain thousands of samples, so the GIF samples the
    trajectory uniformly instead of writing one image per logger sample.
    """
    if count <= maximum:
        return list(range(count))
    return [round(index * (count - 1) / (maximum - 1)) for index in range(maximum)]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--map", dest="map_path", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--max-frames", type=int, default=360)
    parser.add_argument(
        "--title",
        default="Differential-drive A* navigation",
        help="Title rendered inside the GIF for model/scenario identification.",
    )
    parser.add_argument(
        "--map-context-m",
        type=float,
        default=1.2,
        help=(
            "Map context retained around the trajectory and plan. This keeps "
            "relevant corridor walls visible without showing the full world."
        ),
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    trace = read_csv(args.trace)
    plan = read_csv(args.plan)
    occupied = read_csv(args.map_path)
    if not trace:
        raise ValueError(f"Trace is empty: {args.trace}")
    if not plan:
        raise ValueError(f"Plan is empty: {args.plan}")

    trace_x = [float(row["x_m"]) for row in trace]
    trace_y = [float(row["y_m"]) for row in trace]
    plan_x = [float(row["x_m"]) for row in plan]
    plan_y = [float(row["y_m"]) for row in plan]
    map_x = [float(row["x_m"]) for row in occupied]
    map_y = [float(row["y_m"]) for row in occupied]

    goal_x = optional_float(trace[-1], "goal_x_m")
    goal_y = optional_float(trace[-1], "goal_y_m")
    if goal_x is None or goal_y is None:
        goal_x, goal_y = plan_x[-1], plan_y[-1]

    all_x = trace_x + plan_x + [goal_x]
    all_y = trace_y + plan_y + [goal_y]
    base_x_min, base_x_max = min(all_x) - 0.45, max(all_x) + 0.45
    base_y_min, base_y_max = min(all_y) - 0.75, max(all_y) + 0.75
    map_resolution = (
        float(occupied[0].get("resolution_m", 0.1)) if occupied else 0.1
    )
    context_x_min = min(all_x) - args.map_context_m
    context_x_max = max(all_x) + args.map_context_m
    context_y_min = min(all_y) - args.map_context_m
    context_y_max = max(all_y) + args.map_context_m
    visible_map = [
        (x, y)
        for x, y in zip(map_x, map_y)
        if context_x_min <= x <= context_x_max
        and context_y_min <= y <= context_y_max
    ]
    if visible_map:
        x_min = min(
            base_x_min,
            min(x for x, _ in visible_map) - 0.5 * map_resolution - 0.15,
        )
        x_max = max(
            base_x_max,
            max(x for x, _ in visible_map) + 0.5 * map_resolution + 0.15,
        )
        y_min = min(
            base_y_min,
            min(y for _, y in visible_map) - 0.5 * map_resolution - 0.15,
        )
        y_max = max(
            base_y_max,
            max(y for _, y in visible_map) + 0.5 * map_resolution + 0.15,
        )
    else:
        x_min, x_max = base_x_min, base_x_max
        y_min, y_max = base_y_min, base_y_max

    figure, axis = plt.subplots(figsize=(8.0, 5.0), dpi=120)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(x_min, x_max)
    axis.set_ylim(y_min, y_max)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title(args.title)
    axis.grid(True, alpha=0.25)

    from matplotlib.patches import Patch, Rectangle

    for x, y in visible_map:
        axis.add_patch(
            Rectangle(
                (x - 0.5 * map_resolution, y - 0.5 * map_resolution),
                map_resolution,
                map_resolution,
                facecolor="#555555",
                edgecolor="none",
                zorder=1,
            )
        )
    map_legend = Patch(
        facecolor="#555555",
        edgecolor="none",
        label="occupied cells",
    )
    plan_line, = axis.plot(
        plan_x,
        plan_y,
        "--",
        color="#2563eb",
        linewidth=2.0,
        label="A* path",
        zorder=2,
    )
    start_marker = axis.scatter(
        [trace_x[0]],
        [trace_y[0]],
        marker="o",
        s=55,
        color="#16a34a",
        label="start",
        zorder=4,
    )
    goal_marker = axis.scatter(
        [goal_x],
        [goal_y],
        marker="*",
        s=180,
        color="#f59e0b",
        label="goal",
        zorder=4,
    )
    trajectory, = axis.plot(
        [],
        [],
        color="#dc2626",
        linewidth=2.0,
        label="physical /odom trajectory",
        zorder=3,
    )
    robot, = axis.plot([], [], "o", color="#0ea5e9", markersize=9, label="robot", zorder=5)
    heading, = axis.plot([], [], color="#0f172a", linewidth=2.0, zorder=5)
    status = axis.text(
        0.02,
        0.98,
        "",
        transform=axis.transAxes,
        va="top",
        family="monospace",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
        zorder=6,
    )
    axis.legend(
        handles=[
            map_legend,
            plan_line,
            start_marker,
            goal_marker,
            trajectory,
            robot,
        ],
        labels=[
            "occupied cells",
            "A* path",
            "start",
            "goal",
            "physical /odom trajectory",
            "robot",
        ],
        loc="lower right",
        fontsize=8,
    )

    indices = frame_indices(len(trace), max(2, args.max_frames))

    def update(frame_number: int):
        """Update the animated trajectory, robot heading, and status text."""
        row = trace[indices[frame_number]]
        current_index = indices[frame_number]
        x = float(row["x_m"])
        y = float(row["y_m"])
        yaw = float(row["yaw_rad"])
        trajectory.set_data(trace_x[: current_index + 1], trace_y[: current_index + 1])
        robot.set_data([x], [y])
        heading.set_data(
            [x, x + 0.18 * math.cos(yaw)],
            [y, y + 0.18 * math.sin(yaw)],
        )

        clearance = optional_float(row, "front_clearance_m")
        override = row.get("safety_override", "")
        clearance_text = "n/a" if clearance is None else f"{clearance:.3f} m"
        status.set_text(
            f"t = {float(row['time_s']):6.1f} s\n"
            f"front clearance = {clearance_text}\n"
            f"safety override = {override}"
        )
        return trajectory, robot, heading, status

    animation = FuncAnimation(
        figure,
        update,
        frames=len(indices),
        interval=1000 / args.fps,
        blit=False,
        repeat=False,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(args.output, writer=PillowWriter(fps=args.fps))
    plt.close(figure)
    print(f"Wrote GIF to {args.output}")


if __name__ == "__main__":
    main()
