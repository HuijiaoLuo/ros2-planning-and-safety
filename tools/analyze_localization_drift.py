#!/usr/bin/env python3
"""Compare final /odom, /state_estimate, and /localized_estimate drift.

The evaluation logger stores ``x_m``/``y_m`` from the bridged ``/odom``
reference and the two estimated poses in each trace.  This tool turns those
three streams into explicit final-pose errors so a goal-distance result cannot
hide a common state-estimation drift.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from statistics import mean
from typing import Iterable


def optional_float(row: dict[str, str], key: str) -> float | None:
    value = row.get(key, "").strip()
    if not value or value.lower() in {"none", "nan"}:
        return None
    return float(value)


def optional_bool(row: dict[str, str], key: str) -> bool | None:
    value = row.get(key, "").strip().lower()
    if not value:
        return None
    if value not in {"true", "false"}:
        raise ValueError(f"expected boolean in {key}, got {value!r}")
    return value == "true"


def distance_between(
    row: dict[str, str],
    first_x_key: str,
    first_y_key: str,
    second_x_key: str,
    second_y_key: str,
) -> float | None:
    values = [
        optional_float(row, key)
        for key in (first_x_key, first_y_key, second_x_key, second_y_key)
    ]
    if any(value is None for value in values):
        return None
    first_x, first_y, second_x, second_y = values
    return math.hypot(first_x - second_x, first_y - second_y)


def split_stem(stem: str) -> tuple[str, str, int]:
    """Parse ``scenario__backend__seed_000`` into its matrix identity."""
    prefix, seed_text = stem.rsplit("__seed_", 1)
    scenario, backend = prefix.split("__", 1)
    return scenario, backend, int(seed_text)


@dataclass(frozen=True)
class DriftRecord:
    scenario: str
    backend: str
    seed: int
    success: bool | None
    termination_reason: str
    navigation_pose_topic: str
    navigation_pose_is_independent: bool | None
    navigation_pose_fallback_active: bool | None
    ground_truth_goal_error_m: float | None
    navigation_goal_error_m: float | None
    state_estimate_goal_error_m: float | None
    navigation_truth_error_m: float | None
    state_estimate_truth_error_m: float | None
    navigation_goal_error_gap_m: float | None
    state_estimate_goal_error_gap_m: float | None
    navigation_timestamp_offset_s: float | None
    state_estimate_timestamp_offset_s: float | None
    localization_match_status: str
    localization_candidate_correction_m: float | None
    localization_applied_correction_m: float | None


def record_from_trace(
    stem: str,
    row: dict[str, str],
    evaluation: dict[str, str] | None = None,
) -> DriftRecord:
    scenario, backend, seed = split_stem(stem)
    ground_truth_goal_error = optional_float(row, "ground_truth_goal_error_m")
    navigation_goal_error = optional_float(row, "navigation_goal_error_m")
    state_goal_error = optional_float(row, "state_estimate_goal_error_m")
    return DriftRecord(
        scenario=scenario,
        backend=backend,
        seed=seed,
        success=(
            None
            if evaluation is None or not evaluation.get("success", "").strip()
            else evaluation.get("success", "").strip().lower() == "true"
        ),
        termination_reason=(
            ""
            if evaluation is None
            else evaluation.get("termination_reason", "").strip()
        ),
        navigation_pose_topic=(
            ""
            if evaluation is None
            else evaluation.get("navigation_pose_topic", "").strip()
        ),
        navigation_pose_is_independent=(
            None
            if evaluation is None
            else optional_bool(evaluation, "navigation_pose_is_independent_final")
        ),
        navigation_pose_fallback_active=(
            None
            if evaluation is None
            else optional_bool(evaluation, "navigation_pose_fallback_active_final")
        ),
        ground_truth_goal_error_m=ground_truth_goal_error,
        navigation_goal_error_m=navigation_goal_error,
        state_estimate_goal_error_m=state_goal_error,
        navigation_truth_error_m=distance_between(
            row, "navigation_x_m", "navigation_y_m", "x_m", "y_m"
        ),
        state_estimate_truth_error_m=distance_between(
            row, "state_estimate_x_m", "state_estimate_y_m", "x_m", "y_m"
        ),
        navigation_goal_error_gap_m=(
            None
            if ground_truth_goal_error is None or navigation_goal_error is None
            else ground_truth_goal_error - navigation_goal_error
        ),
        state_estimate_goal_error_gap_m=(
            None
            if ground_truth_goal_error is None or state_goal_error is None
            else ground_truth_goal_error - state_goal_error
        ),
        navigation_timestamp_offset_s=optional_float(
            row, "navigation_timestamp_offset_s"
        ),
        state_estimate_timestamp_offset_s=optional_float(
            row, "state_estimate_timestamp_offset_s"
        ),
        localization_match_status=row.get("localization_match_status", "").strip(),
        localization_candidate_correction_m=optional_float(
            row, "localization_candidate_correction_m"
        ),
        localization_applied_correction_m=optional_float(
            row, "localization_applied_correction_m"
        ),
    )


def read_last_row(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"trace is empty: {path}")
    return rows[-1]


def read_first_row(path: Path) -> dict[str, str] | None:
    if not path.exists():
        return None
    with path.open(newline="", encoding="utf-8") as handle:
        return next(csv.DictReader(handle), None)


def load_records(trace_paths: Iterable[Path]) -> list[DriftRecord]:
    records: list[DriftRecord] = []
    for trace_path in sorted(trace_paths):
        stem = trace_path.name.removesuffix("_trace.csv")
        evaluation_path = trace_path.with_name(f"{stem}_eval.csv")
        records.append(
            record_from_trace(
                stem,
                read_last_row(trace_path),
                read_first_row(evaluation_path),
            )
        )
    return records


def mean_field(records: Iterable[DriftRecord], field: str) -> float | None:
    values = [
        value
        for record in records
        if (value := getattr(record, field)) is not None
    ]
    return mean(values) if values else None


def format_statuses(records: Iterable[DriftRecord]) -> str:
    counts: dict[str, int] = {}
    for record in records:
        status = record.localization_match_status or "unknown"
        counts[status] = counts.get(status, 0) + 1
    return ";".join(
        f"{status}={count}" for status, count in sorted(counts.items())
    )


def format_topics(records: Iterable[DriftRecord]) -> str:
    topics = sorted(
        {record.navigation_pose_topic or "unknown" for record in records}
    )
    return ";".join(topics)


def summarize(records: Iterable[DriftRecord]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str], list[DriftRecord]] = {}
    for record in records:
        grouped.setdefault((record.scenario, record.backend), []).append(record)

    summaries: list[dict[str, object]] = []
    for (scenario, backend), group in sorted(grouped.items()):
        known_success = [record.success for record in group if record.success is not None]
        summaries.append(
            {
                "scenario": scenario,
                "backend": backend,
                "runs": len(group),
                "seeds": ",".join(str(record.seed) for record in sorted(group, key=lambda item: item.seed)),
                "navigation_pose_topic": format_topics(group),
                "navigation_independent_runs": sum(
                    record.navigation_pose_is_independent is True for record in group
                ),
                "navigation_fallback_runs": sum(
                    record.navigation_pose_fallback_active is True for record in group
                ),
                "successes": sum(value is True for value in known_success),
                "success_rate": (
                    sum(value is True for value in known_success) / len(known_success)
                    if known_success
                    else None
                ),
                "mean_ground_truth_goal_error_m": mean_field(
                    group, "ground_truth_goal_error_m"
                ),
                "mean_navigation_goal_error_m": mean_field(
                    group, "navigation_goal_error_m"
                ),
                "mean_state_estimate_goal_error_m": mean_field(
                    group, "state_estimate_goal_error_m"
                ),
                "mean_navigation_truth_error_m": mean_field(
                    group, "navigation_truth_error_m"
                ),
                "mean_state_estimate_truth_error_m": mean_field(
                    group, "state_estimate_truth_error_m"
                ),
                "mean_navigation_goal_error_gap_m": mean_field(
                    group, "navigation_goal_error_gap_m"
                ),
                "mean_state_estimate_goal_error_gap_m": mean_field(
                    group, "state_estimate_goal_error_gap_m"
                ),
                "mean_navigation_timestamp_offset_s": mean_field(
                    group, "navigation_timestamp_offset_s"
                ),
                "mean_state_estimate_timestamp_offset_s": mean_field(
                    group, "state_estimate_timestamp_offset_s"
                ),
                "localization_statuses": format_statuses(group),
            }
        )
    return summaries


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("results/localization_matrix"),
        help="Directory containing matrix trace and evaluation CSV files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("results/localization_matrix"),
        help="Directory for the per-run and grouped drift summaries.",
    )
    args = parser.parse_args()

    trace_paths = sorted(
        path
        for path in args.input_dir.glob("*_trace.csv")
        if not path.name.endswith("_estimation_trace.csv")
    )
    if not trace_paths:
        parser.error(f"no trace files found in {args.input_dir}")

    records = load_records(trace_paths)
    per_run = [record.__dict__ for record in records]
    grouped = summarize(records)
    per_run_path = args.output_dir / "localization_drift_diagnostics.csv"
    summary_path = args.output_dir / "localization_drift_summary.csv"
    write_csv(per_run_path, per_run)
    write_csv(summary_path, grouped)

    print("scenario,backend,runs,success_rate,navigation_pose_topic,"
          "navigation_independent_runs,navigation_fallback_runs,"
          "mean_nav_truth_error_m,mean_state_truth_error_m,localization_statuses")
    for row in grouped:
        print(
            f"{row['scenario']},{row['backend']},{row['runs']},"
            f"{row['success_rate']},{row['navigation_pose_topic']},"
            f"{row['navigation_independent_runs']},{row['navigation_fallback_runs']},"
            f"{row['mean_navigation_truth_error_m']},"
            f"{row['mean_state_estimate_truth_error_m']},"
            f"{row['localization_statuses']}"
        )
    print(f"Wrote per-run drift diagnostics to {per_run_path}")
    print(f"Wrote grouped drift summary to {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
