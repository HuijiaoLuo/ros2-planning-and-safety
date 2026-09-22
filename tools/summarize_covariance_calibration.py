#!/usr/bin/env python3
"""Summarize per-sample estimator covariance calibration traces.

Each input is a trace CSV written by ``estimation_logger`` with
``estimation_trace_output:=...``.  The tool compares the measured error to
the covariance reported by the estimator, but it never changes estimator
parameters or feeds truth back into navigation.

For a calibrated Gaussian estimate, the expected mean normalized squared
error is approximately the state dimension: 2 for the joint x/y position and
1 for a scalar heading.  The reported coverage values use chi-square 95%
thresholds (5.991 for two dimensions and 3.841 for one dimension).  These are
diagnostic indicators, not guarantees, because trajectory samples are
temporally correlated.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
from pathlib import Path
from statistics import mean, median
from typing import Iterable


POSITION_THRESHOLD_95 = 5.991
HEADING_THRESHOLD_95 = 3.841

TRACE_FIELDS = (
    "error_x_m",
    "error_y_m",
    "error_heading_rad",
    "position_error_m",
    "estimate_covariance_xx_m2",
    "estimate_covariance_yy_m2",
    "estimate_covariance_yaw_rad2",
    "position_normalized_error_sq",
    "heading_normalized_error_sq",
)

OUTPUT_FIELDS = (
    "trace_file",
    "samples",
    "position_error_rmse_m",
    "mean_position_error_m",
    "median_position_error_m",
    "heading_error_rmse_rad",
    "mean_covariance_xx_m2",
    "mean_covariance_yy_m2",
    "mean_covariance_yaw_rad2",
    "position_nees_samples",
    "mean_position_nees",
    "median_position_nees",
    "position_95pct_coverage",
    "heading_nees_samples",
    "mean_heading_nees",
    "median_heading_nees",
    "heading_95pct_coverage",
    "x_normalized_mse",
    "y_normalized_mse",
)


def finite_values(rows: Iterable[dict[str, str]], field: str) -> list[float]:
    """Return finite numeric values from one trace column."""
    values: list[float] = []
    for row in rows:
        value = row.get(field, "").strip()
        if not value:
            continue
        try:
            parsed = float(value)
        except ValueError:
            continue
        if math.isfinite(parsed):
            values.append(parsed)
    return values


def rmse(values: list[float]) -> float | None:
    """Return root mean square, preserving ``None`` for an empty column."""
    if not values:
        return None
    return math.sqrt(mean(value * value for value in values))


def normalized_marginal_values(
    rows: Iterable[dict[str, str]],
    error_field: str,
    variance_field: str,
) -> list[float]:
    """Compute marginal normalized squared errors without losing row pairing."""
    values: list[float] = []
    for row in rows:
        try:
            error = float(row.get(error_field, ""))
            variance = float(row.get(variance_field, ""))
        except (TypeError, ValueError):
            continue
        if math.isfinite(error) and math.isfinite(variance) and variance > 0.0:
            values.append(error * error / variance)
    return values


def summarize_trace(path: Path) -> dict[str, object]:
    """Compute calibration indicators for one run."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = set(reader.fieldnames or ())
        missing = [field for field in TRACE_FIELDS if field not in fieldnames]
        if missing:
            raise ValueError(
                f"{path} is not an estimator trace; missing {', '.join(missing)}"
            )
        rows = list(reader)

    error_x = finite_values(rows, "error_x_m")
    error_y = finite_values(rows, "error_y_m")
    error_heading = finite_values(rows, "error_heading_rad")
    position_error = finite_values(rows, "position_error_m")
    covariance_xx = finite_values(rows, "estimate_covariance_xx_m2")
    covariance_yy = finite_values(rows, "estimate_covariance_yy_m2")
    covariance_yaw = finite_values(rows, "estimate_covariance_yaw_rad2")
    position_nees = finite_values(rows, "position_normalized_error_sq")
    heading_nees = finite_values(rows, "heading_normalized_error_sq")

    def coverage(values: list[float], threshold: float) -> float | None:
        if not values:
            return None
        return sum(value <= threshold for value in values) / len(values)

    x_normalized = normalized_marginal_values(
        rows,
        "error_x_m",
        "estimate_covariance_xx_m2",
    )
    y_normalized = normalized_marginal_values(
        rows,
        "error_y_m",
        "estimate_covariance_yy_m2",
    )

    return {
        "trace_file": str(path),
        "samples": len(rows),
        "position_error_rmse_m": rmse(position_error),
        "mean_position_error_m": mean(position_error) if position_error else None,
        "median_position_error_m": median(position_error) if position_error else None,
        "heading_error_rmse_rad": rmse(error_heading),
        "mean_covariance_xx_m2": mean(covariance_xx) if covariance_xx else None,
        "mean_covariance_yy_m2": mean(covariance_yy) if covariance_yy else None,
        "mean_covariance_yaw_rad2": mean(covariance_yaw) if covariance_yaw else None,
        "position_nees_samples": len(position_nees),
        "mean_position_nees": mean(position_nees) if position_nees else None,
        "median_position_nees": median(position_nees) if position_nees else None,
        "position_95pct_coverage": coverage(position_nees, POSITION_THRESHOLD_95),
        "heading_nees_samples": len(heading_nees),
        "mean_heading_nees": mean(heading_nees) if heading_nees else None,
        "median_heading_nees": median(heading_nees) if heading_nees else None,
        "heading_95pct_coverage": coverage(heading_nees, HEADING_THRESHOLD_95),
        "x_normalized_mse": mean(x_normalized) if x_normalized else None,
        "y_normalized_mse": mean(y_normalized) if y_normalized else None,
    }


def resolve_inputs(inputs: list[str], pattern: str | None) -> list[Path]:
    """Resolve explicit paths or a shell-independent glob pattern."""
    if bool(inputs) == bool(pattern):
        raise ValueError("provide exactly one of --inputs or --glob")
    if pattern:
        paths = [Path(value) for value in sorted(glob.glob(pattern))]
    else:
        paths = [Path(value) for value in inputs]
    if not paths:
        raise ValueError("no trace CSV files matched")
    return paths


def format_value(value: object) -> str:
    """Format a compact terminal table value."""
    if value is None:
        return "--"
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--inputs", nargs="+", help="Trace CSV files to summarize")
    source.add_argument("--glob", dest="pattern", help="Glob pattern for trace CSVs")
    parser.add_argument("--output", required=True, help="Output summary CSV path")
    args = parser.parse_args()

    paths = resolve_inputs(args.inputs or [], args.pattern)
    summaries: list[dict[str, object]] = []
    for path in paths:
        try:
            summaries.append(summarize_trace(path))
        except (OSError, ValueError) as exc:
            print(f"Skipping {path}: {exc}")

    if not summaries:
        raise SystemExit("no valid estimator traces were found")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(summaries)

    print(
        "trace_file samples position_rmse mean_position_nees "
        "position_coverage mean_heading_nees heading_coverage"
    )
    for summary in summaries:
        print(
            f"{summary['trace_file']} {summary['samples']} "
            f"{format_value(summary['position_error_rmse_m'])} "
            f"{format_value(summary['mean_position_nees'])} "
            f"{format_value(summary['position_95pct_coverage'])} "
            f"{format_value(summary['mean_heading_nees'])} "
            f"{format_value(summary['heading_95pct_coverage'])}"
        )
    print(f"Read {len(summaries)} trace files.")
    print(f"Wrote covariance calibration summary to {output}")


if __name__ == "__main__":
    main()
