#!/usr/bin/env python3
"""Aggregate V3 estimator diagnostic CSV files into a compact summary.

The input files are the one-row reports written by ``estimation_logger``.
Rows are grouped by the configured gyro model, wheel-slip model, and
wheel-yaw fusion weight; random seeds remain visible in the summary instead
of creating separate groups.
The tool uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from statistics import mean
from typing import Iterable


CONFIG_FIELDS = (
    "configured_imu_gyro_bias_rad_s",
    "configured_imu_gyro_noise_std_rad_s",
    "configured_wheel_weight",
    "configured_wheel_slip_ratio",
    "configured_position_mode",
    "configured_fusion_mode",
    "configured_gyro_rate_noise_std_rad_s",
    "configured_wheel_yaw_noise_std_rad",
    "configured_gyro_bias_random_walk_std_rad_s2",
    "configured_adaptive_wheel_noise",
    "configured_wheel_yaw_noise_min_std_rad",
    "configured_wheel_yaw_noise_max_std_rad",
    "configured_wheel_noise_adaptation_rate",
)

REQUIRED_CONFIG_FIELDS = CONFIG_FIELDS[:2]

OUTPUT_FIELDS = (
    *CONFIG_FIELDS,
    "runs",
    "seed_count",
    "seeds",
    "mean_wheel_position_rmse_m",
    "mean_estimate_position_rmse_m",
    "mean_wheel_heading_rmse_rad",
    "mean_imu_heading_rmse_rad",
    "mean_estimate_heading_rmse_rad",
    "heading_rmse_improvement_pct",
    "mean_wheel_heading_bias_rad",
    "mean_estimate_heading_bias_rad",
    "mean_wheel_final_position_error_m",
    "mean_estimate_final_position_error_m",
    "mean_estimate_final_heading_error_rad",
    "mean_adaptive_wheel_yaw_noise_std_rad",
    "mean_final_wheel_yaw_noise_estimate_std_rad",
)

METRIC_FIELDS = (
    "wheel_position_rmse_m",
    "estimate_position_rmse_m",
    "wheel_heading_rmse_rad",
    "imu_heading_rmse_rad",
    "estimate_heading_rmse_rad",
    "wheel_heading_bias_rad",
    "estimate_heading_bias_rad",
    "wheel_final_position_error_m",
    "estimate_final_position_error_m",
    "estimate_final_heading_error_rad",
)


def optional_float(row: dict[str, str], name: str) -> float | None:
    value = row.get(name, "").strip()
    return None if not value else float(value)


def normalized_config(row: dict[str, str]) -> tuple[object, ...]:
    """Return a stable grouping key for the estimator configuration.

    Defaults preserve comparability with older CSV files, while the rounded
    numeric tuple prevents harmless floating-point formatting differences from
    splitting one experimental condition into multiple groups.
    """
    values: list[object] = []
    for name in CONFIG_FIELDS:
        if name in {
            "configured_position_mode",
            "configured_fusion_mode",
            "configured_adaptive_wheel_noise",
        }:
            if name == "configured_fusion_mode":
                default = "fixed"
            elif name == "configured_position_mode":
                default = "wheel_pose"
            else:
                default = "false"
            value = row.get(name, default).strip().lower() or default
            if name == "configured_adaptive_wheel_noise":
                value = value in {"1", "true", "yes", "on"}
            values.append(value)
            continue
        value = optional_float(row, name)
        # Older estimator reports predate the slip model and therefore imply
        # the default zero-slip configuration.
        if value is None and name == "configured_wheel_slip_ratio":
            value = 0.0
        if value is None and name == "configured_wheel_weight":
            # Older reports used the estimator's historical default.
            value = 0.02
        if value is None and name == "configured_gyro_rate_noise_std_rad_s":
            value = 0.01
        if value is None and name == "configured_wheel_yaw_noise_std_rad":
            value = 0.07
        if value is None and name == "configured_gyro_bias_random_walk_std_rad_s2":
            value = 0.001
        if value is None and name == "configured_wheel_yaw_noise_min_std_rad":
            value = 0.02
        if value is None and name == "configured_wheel_yaw_noise_max_std_rad":
            value = 0.20
        if value is None and name == "configured_wheel_noise_adaptation_rate":
            value = 0.05
        if value is None:
            raise ValueError(f"Missing required configuration field: {name}")
        values.append(round(value, 9))
    return tuple(values)


def values_for(rows: list[dict[str, str]], field: str) -> list[float]:
    return [
        value
        for row in rows
        if (value := optional_float(row, field)) is not None
    ]


def summary_for_rows(rows: list[dict[str, str]]) -> dict[str, object]:
    """Average metrics within one controlled estimator configuration."""
    if not rows:
        raise ValueError("Cannot summarize an empty group")

    config = normalized_config(rows[0])
    seeds = sorted(
        {
            row.get("configured_imu_gyro_noise_seed", "").strip()
            for row in rows
            if row.get("configured_imu_gyro_noise_seed", "").strip()
        },
        key=lambda value: int(float(value)),
    )

    summary: dict[str, object] = dict(zip(CONFIG_FIELDS, config))
    summary.update(
        {
            "runs": len(rows),
            "seed_count": len(seeds),
            "seeds": ",".join(seeds),
        }
    )

    means: dict[str, float | None] = {}
    for field in METRIC_FIELDS:
        values = values_for(rows, field)
        means[f"mean_{field}"] = mean(values) if values else None
    summary.update(means)

    # Positive improvement means the fused estimate has lower heading RMSE than
    # wheel yaw.  A negative value is meaningful: it records a failed tuning or
    # uncertainty model instead of hiding it behind an absolute value.
    wheel_heading = means["mean_wheel_heading_rmse_rad"]
    estimate_heading = means["mean_estimate_heading_rmse_rad"]
    summary["heading_rmse_improvement_pct"] = (
        None
        if wheel_heading in (None, 0.0) or estimate_heading is None
        else 100.0 * (wheel_heading - estimate_heading) / wheel_heading
    )
    noise_values = values_for(
        rows, "wheel_yaw_noise_estimate_mean_std_rad"
    )
    final_noise_values = values_for(
        rows, "final_wheel_yaw_noise_estimate_std_rad"
    )
    summary["mean_adaptive_wheel_yaw_noise_std_rad"] = (
        mean(noise_values) if noise_values else None
    )
    summary["mean_final_wheel_yaw_noise_estimate_std_rad"] = (
        mean(final_noise_values) if final_noise_values else None
    )
    return summary


def load_rows(paths: Iterable[Path]) -> tuple[list[dict[str, str]], list[Path]]:
    """Load supported one-row reports and explicitly skip legacy schemas."""
    rows: list[dict[str, str]] = []
    used_paths: list[Path] = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fieldnames = set(reader.fieldnames or ())
            missing = [
                name for name in REQUIRED_CONFIG_FIELDS if name not in fieldnames
            ]
            if missing:
                print(
                    "Skipping legacy CSV without complete estimator configuration "
                    f"({', '.join(missing)}): {path}"
                )
                continue
            file_rows = list(reader)
            rows.extend(file_rows)
            used_paths.append(path)
    return rows, used_paths


def summarize(rows: Iterable[dict[str, str]]) -> list[dict[str, object]]:
    groups: dict[tuple[object, ...], list[dict[str, str]]] = {}
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
        "bias_rad_s",
        "noise_rad_s",
        "wheel_weight",
        "slip_ratio",
        "position_mode",
        "fusion_mode",
        "gyro_noise",
        "wheel_yaw_noise",
        "bias_rw",
        "adaptive_R",
        "R_min",
        "R_max",
        "R_rate",
        "runs",
        "seeds",
        "wheel_hdg_rmse",
        "imu_hdg_rmse",
        "estimate_hdg_rmse",
        "improvement_pct",
        "wheel_pos_rmse",
        "estimate_pos_rmse",
        "adaptive_R_mean",
        "adaptive_R_final",
    ]
    print(" ".join(f"{header:>18}" for header in headers))
    for row in summaries:
        values = [
            format_value(row["configured_imu_gyro_bias_rad_s"]),
            format_value(row["configured_imu_gyro_noise_std_rad_s"]),
            format_value(row["configured_wheel_weight"]),
            format_value(row["configured_wheel_slip_ratio"]),
            format_value(row["configured_position_mode"]),
            format_value(row["configured_fusion_mode"]),
            format_value(row["configured_gyro_rate_noise_std_rad_s"]),
            format_value(row["configured_wheel_yaw_noise_std_rad"]),
            format_value(row["configured_gyro_bias_random_walk_std_rad_s2"]),
            format_value(row["configured_adaptive_wheel_noise"]),
            format_value(row["configured_wheel_yaw_noise_min_std_rad"]),
            format_value(row["configured_wheel_yaw_noise_max_std_rad"]),
            format_value(row["configured_wheel_noise_adaptation_rate"]),
            format_value(row["runs"]),
            format_value(row["seeds"]),
            format_value(row["mean_wheel_heading_rmse_rad"]),
            format_value(row["mean_imu_heading_rmse_rad"]),
            format_value(row["mean_estimate_heading_rmse_rad"]),
            format_value(row["heading_rmse_improvement_pct"]),
            format_value(row["mean_wheel_position_rmse_m"]),
            format_value(row["mean_estimate_position_rmse_m"]),
            format_value(row["mean_adaptive_wheel_yaw_noise_std_rad"]),
            format_value(row["mean_final_wheel_yaw_noise_estimate_std_rad"]),
        ]
        print(" ".join(f"{value:>18}" for value in values))


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
        help="Explicit estimator CSV files. If omitted, use --glob.",
    )
    parser.add_argument(
        "--glob",
        default="results/v3_*_metrics.csv",
        help="Input glob used when --inputs is omitted.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/estimation_summary.csv"),
        help="Summary CSV output path.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    paths = args.inputs or sorted(Path(".").glob(args.glob))
    if not paths:
        raise SystemExit("No estimator CSV files matched the input selection.")

    rows, used_paths = load_rows(paths)
    if not rows:
        raise SystemExit("No supported estimator rows were found.")

    summaries = summarize(rows)
    print_summary(summaries)
    write_csv(summaries, args.output)
    print(f"Read {len(rows)} rows from {len(used_paths)} files.")
    print(f"Wrote {len(summaries)} estimator groups to {args.output}")


if __name__ == "__main__":
    main()
