#!/usr/bin/env python3
"""Aggregate closed-loop robustness CSV files into a compact summary.

The input files are the one-row CSV reports written by ``evaluation_logger``.
Configuration columns are used as grouping keys, while random seeds remain
visible in the summary instead of creating separate configuration groups.
The tool uses only the Python standard library so it can run in the existing
portfolio environment without additional dependencies.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path
from statistics import mean
from typing import Iterable


CONFIG_FIELDS = (
    "configured_minimum_clearance_m",
    "configured_sensor_latency_s",
    "configured_scan_delay_s",
    "configured_scan_noise_std_m",
    "configured_safety_margin_m",
    "configured_planning_radius_m",
)

OUTPUT_FIELDS = (
    *CONFIG_FIELDS,
    "runs",
    "seed_count",
    "seeds",
    "successes",
    "goal_reached_count",
    "experiment_timeout_count",
    "manual_interrupt_count",
    "external_interrupt_count",
    "unknown_termination_count",
    "success_rate",
    "collision_count",
    "collision_rate_known",
    "mean_time_to_goal_s",
    "p95_time_to_goal_s",
    "mean_final_error_m",
    "mean_travelled_distance_m",
    "mean_path_efficiency_successful",
    "mean_minimum_clearance_m",
    "minimum_observed_clearance_m",
    "mean_safety_override_count",
    "total_safety_override_time_s",
    "mean_safety_override_ratio",
    "max_safety_override_ratio",
)


def float_value(row: dict[str, str], name: str, default: float = 0.0) -> float:
    value = row.get(name, "").strip()
    if not value:
        return default
    return float(value)


def optional_float(row: dict[str, str], name: str) -> float | None:
    value = row.get(name, "").strip()
    return None if not value else float(value)


def bool_value(row: dict[str, str], name: str) -> bool:
    return row.get(name, "").strip().lower() == "true"


def termination_reason(row: dict[str, str]) -> str:
    """Return an explicit reason, with a safe fallback for legacy reports."""
    value = row.get("termination_reason", "").strip()
    if value:
        return value
    return "goal_reached" if bool_value(row, "success") else "unknown"


def normalized_config(row: dict[str, str]) -> tuple[float, ...]:
    """Return a stable grouping key, including defaults for older CSV files."""
    return tuple(round(float_value(row, name), 9) for name in CONFIG_FIELDS)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] + weight * (ordered[upper] - ordered[lower])


def summary_for_rows(rows: list[dict[str, str]]) -> dict[str, object]:
    """Aggregate closed-loop outcomes for one complete configuration."""
    if not rows:
        raise ValueError("Cannot summarize an empty group")

    config = normalized_config(rows[0])
    successes = sum(bool_value(row, "success") for row in rows)
    termination_counts = Counter(termination_reason(row) for row in rows)
    collision_values = [
        row.get("collision", "").strip().lower()
        for row in rows
        if row.get("collision", "").strip().lower() in {"true", "false"}
    ]
    collision_count = sum(value == "true" for value in collision_values)
    seeds = sorted(
        {
            row.get("configured_scan_noise_seed", "").strip()
            for row in rows
            if row.get("configured_scan_noise_seed", "").strip()
        },
        key=lambda value: int(float(value)),
    )

    # Incomplete runs have no valid time-to-goal.  Excluding those values keeps
    # the mean interpretable as the time among successful completions.
    time_to_goal = [
        value
        for row in rows
        if (value := optional_float(row, "time_to_goal_s")) is not None
    ]
    final_errors = [
        value
        for row in rows
        if (value := optional_float(row, "final_error_m")) is not None
    ]
    travelled = [
        value
        for row in rows
        if (value := optional_float(row, "travelled_distance_m")) is not None
    ]
    # Path efficiency is deliberately restricted to successful runs because a
    # partial travelled distance cannot represent the full route.
    successful_efficiency = [
        value
        for row in rows
        if bool_value(row, "success")
        and (value := optional_float(row, "path_efficiency")) is not None
    ]
    clearance = [
        value
        for row in rows
        if (value := optional_float(row, "minimum_clearance_m")) is not None
    ]
    override_counts = [float_value(row, "safety_override_count") for row in rows]
    override_times = [
        float_value(row, "safety_override_time_s") for row in rows
    ]
    override_ratios = [
        value
        for row in rows
        if (value := optional_float(row, "safety_override_ratio")) is not None
    ]

    summary: dict[str, object] = dict(zip(CONFIG_FIELDS, config))
    summary.update(
        {
            "runs": len(rows),
            "seed_count": len(seeds),
            "seeds": ",".join(seeds),
            "successes": successes,
            "goal_reached_count": termination_counts["goal_reached"],
            "experiment_timeout_count": termination_counts["experiment_timeout"],
            "manual_interrupt_count": termination_counts["manual_interrupt"],
            "external_interrupt_count": termination_counts["external_interrupt"],
            "unknown_termination_count": termination_counts["unknown"],
            "success_rate": successes / len(rows),
            "collision_count": collision_count,
            "collision_rate_known": (
                None
                if not collision_values
                else collision_count / len(collision_values)
            ),
            "mean_time_to_goal_s": mean(time_to_goal) if time_to_goal else None,
            "p95_time_to_goal_s": percentile(time_to_goal, 0.95),
            "mean_final_error_m": mean(final_errors) if final_errors else None,
            "mean_travelled_distance_m": mean(travelled) if travelled else None,
            "mean_path_efficiency_successful": (
                mean(successful_efficiency) if successful_efficiency else None
            ),
            "mean_minimum_clearance_m": mean(clearance) if clearance else None,
            "minimum_observed_clearance_m": min(clearance) if clearance else None,
            "mean_safety_override_count": mean(override_counts),
            "total_safety_override_time_s": sum(override_times),
            "mean_safety_override_ratio": (
                mean(override_ratios) if override_ratios else None
            ),
            "max_safety_override_ratio": (
                max(override_ratios) if override_ratios else None
            ),
        }
    )
    return summary


def load_rows(paths: Iterable[Path]) -> tuple[list[dict[str, str]], list[Path]]:
    """Read complete schemas while reporting skipped legacy CSV files."""
    rows: list[dict[str, str]] = []
    used_paths: list[Path] = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fieldnames = set(reader.fieldnames or ())
            if not fieldnames.intersection(CONFIG_FIELDS):
                print(f"Skipping unsupported CSV: {path}")
                continue
            missing = [name for name in CONFIG_FIELDS if name not in fieldnames]
            if missing:
                print(
                    f"Skipping legacy CSV without complete configuration "
                    f"({', '.join(missing)}): {path}"
                )
                continue
            file_rows = list(reader)
            rows.extend(file_rows)
            used_paths.append(path)
    return rows, used_paths


def summarize(rows: Iterable[dict[str, str]]) -> list[dict[str, object]]:
    groups: dict[tuple[float, ...], list[dict[str, str]]] = {}
    for row in rows:
        groups.setdefault(normalized_config(row), []).append(row)
    return [summary_for_rows(groups[key]) for key in sorted(groups)]


def format_value(value: object) -> str:
    if value is None:
        return "--"
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def print_summary(summaries: list[dict[str, object]]) -> None:
    headers = [
        "min_clearance",
        "sensor_latency",
        "delay",
        "noise",
        "margin",
        "radius",
        "runs",
        "success",
        "goal",
        "timeout",
        "manual",
        "external",
        "unknown",
        "success_rate",
        "mean_goal_s",
        "mean_clearance_m",
        "override_ratio",
        "collision",
    ]
    print(" ".join(f"{header:>16}" for header in headers))
    for row in summaries:
        values = [
            format_value(row["configured_minimum_clearance_m"]),
            format_value(row["configured_sensor_latency_s"]),
            format_value(row["configured_scan_delay_s"]),
            format_value(row["configured_scan_noise_std_m"]),
            format_value(row["configured_safety_margin_m"]),
            format_value(row["configured_planning_radius_m"]),
            format_value(row["runs"]),
            f"{row['successes']}/{row['runs']}",
            format_value(row["goal_reached_count"]),
            format_value(row["experiment_timeout_count"]),
            format_value(row["manual_interrupt_count"]),
            format_value(row["external_interrupt_count"]),
            format_value(row["unknown_termination_count"]),
            format_value(row["success_rate"]),
            format_value(row["mean_time_to_goal_s"]),
            format_value(row["mean_minimum_clearance_m"]),
            format_value(row["mean_safety_override_ratio"]),
            format_value(row["collision_count"]),
        ]
        print(" ".join(f"{value:>16}" for value in values))


def write_csv(summaries: list[dict[str, object]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(summaries)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--inputs",
        nargs="+",
        type=Path,
        help="Explicit evaluation CSV files. If omitted, use --glob.",
    )
    parser.add_argument(
        "--glob",
        default="results/sweep_*.csv",
        help="Input glob used when --inputs is omitted.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/robustness_summary.csv"),
        help="Summary CSV output path.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    paths = args.inputs or sorted(Path(".").glob(args.glob))
    if not paths:
        raise SystemExit("No evaluation CSV files matched the input selection.")

    rows, used_paths = load_rows(paths)
    if not rows:
        raise SystemExit("No supported evaluation rows were found.")

    summaries = summarize(rows)
    print_summary(summaries)
    write_csv(summaries, args.output)
    print(f"Read {len(rows)} rows from {len(used_paths)} files.")
    print(f"Wrote {len(summaries)} configuration groups to {args.output}")


if __name__ == "__main__":
    main()
