#!/usr/bin/env python3
"""Plot the closed-loop robustness summary for portfolio documentation."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def number(row: dict[str, str], name: str) -> float | None:
    """Convert one CSV field while treating missing metrics as unavailable."""
    value = row.get(name, "").strip()
    return None if not value or value == "--" else float(value)


def matches(row: dict[str, str], **expected: float) -> bool:
    """Select rows whose configuration matches within CSV float precision."""
    for name, target in expected.items():
        value = number(row, name)
        if value is None or abs(value - target) > 1e-9:
            return False
    return True


def read_summary(path: Path) -> list[dict[str, str]]:
    """Load the already-aggregated robustness table."""
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def ordered_points(
    rows: list[dict[str, str]],
    x_field: str,
    y_field: str,
    **filters: float,
) -> tuple[list[float], list[float]]:
    """Extract and sort one x/y series after applying configuration filters."""
    points = []
    for row in rows:
        if not matches(row, **filters):
            continue
        x_value = number(row, x_field)
        y_value = number(row, y_field)
        if x_value is not None and y_value is not None:
            points.append((x_value, y_value))
    points.sort()
    return [point[0] for point in points], [point[1] for point in points]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--summary",
        type=Path,
        default=Path("results/robustness_summary.csv"),
        help="Aggregated robustness CSV.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/robustness_summary.png"),
        help="Output image path.",
    )
    parser.add_argument("--dpi", type=int, default=160)
    return parser


def main() -> None:
    args = build_parser().parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = read_summary(args.summary)
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    figure.suptitle("Closed-loop robustness evaluation", fontsize=16)

    delay_filters = {
        "configured_minimum_clearance_m": 0.50,
        "configured_sensor_latency_s": 0.10,
        "configured_scan_noise_std_m": 0.0,
        "configured_safety_margin_m": 0.15,
        "configured_planning_radius_m": 0.35,
    }
    x, y = ordered_points(
        rows,
        "configured_scan_delay_s",
        "mean_safety_override_ratio",
        **delay_filters,
    )
    axes[0, 0].plot(x, y, "o-", color="#c43c39")
    axes[0, 0].set_title("Stale LiDAR vs safety intervention")
    axes[0, 0].set_xlabel("Scan delay (s)")
    axes[0, 0].set_ylabel("Mean safety override ratio")
    axes[0, 0].grid(True, alpha=0.3)

    x, y = ordered_points(
        rows,
        "configured_scan_delay_s",
        "mean_time_to_goal_s",
        **delay_filters,
    )
    axes[0, 1].plot(x, y, "o-", color="#286090")
    axes[0, 1].set_title("Stale LiDAR vs navigation time")
    axes[0, 1].set_xlabel("Scan delay (s)")
    axes[0, 1].set_ylabel("Mean time to goal (s)")
    axes[0, 1].grid(True, alpha=0.3)

    noise_base = {
        "configured_minimum_clearance_m": 0.50,
        "configured_sensor_latency_s": 0.10,
        "configured_scan_delay_s": 0.0,
        "configured_safety_margin_m": 0.15,
    }
    noise_radii = sorted(
        {
            value
            for row in rows
            if matches(row, **noise_base)
            and (value := number(row, "configured_planning_radius_m")) is not None
        }
    )
    colors = ("#c43c39", "#d28b27", "#2f8f5b", "#286090")
    for index, radius in enumerate(noise_radii):
        color = colors[index % len(colors)]
        x, y = ordered_points(
            rows,
            "configured_scan_noise_std_m",
            "success_rate",
            configured_planning_radius_m=radius,
            **noise_base,
        )
        if x:
            axes[1, 0].plot(
                x,
                y,
                "o-",
                color=color,
                label=f"planning radius = {radius:.2f} m",
            )
    axes[1, 0].set_title("LiDAR noise vs success rate")
    axes[1, 0].set_xlabel("Noise standard deviation (m)")
    axes[1, 0].set_ylabel("Success rate")
    axes[1, 0].set_ylim(-0.05, 1.05)
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    for index, radius in enumerate(noise_radii):
        color = colors[index % len(colors)]
        x, y = ordered_points(
            rows,
            "configured_scan_noise_std_m",
            "mean_safety_override_ratio",
            configured_planning_radius_m=radius,
            **noise_base,
        )
        if x:
            axes[1, 1].plot(
                x,
                y,
                "o-",
                color=color,
                label=f"planning radius = {radius:.2f} m",
            )
    axes[1, 1].set_title("LiDAR noise vs safety intervention")
    axes[1, 1].set_xlabel("Noise standard deviation (m)")
    axes[1, 1].set_ylabel("Mean safety override ratio")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=args.dpi, bbox_inches="tight")
    plt.close(figure)
    print(f"Wrote robustness plot to {args.output}")


if __name__ == "__main__":
    main()
