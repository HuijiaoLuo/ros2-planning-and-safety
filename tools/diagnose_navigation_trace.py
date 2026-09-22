#!/usr/bin/env python3
"""Explain one closed-loop navigation run from its evaluation and trace CSVs.

The evaluation logger deliberately keeps physical ``/odom`` success separate
from the pose used by navigation.  This script turns that separation into a
short, reproducible diagnosis instead of requiring manual inspection of a
long trace.  It is read-only and uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path
from typing import Iterable


def read_one_row(path: Path) -> dict[str, str]:
    """Read the one-row evaluation report and reject malformed input."""
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 1:
        raise ValueError(f"expected one row in {path}, found {len(rows)}")
    return rows[0]


def read_rows(path: Path) -> list[dict[str, str]]:
    """Read a trace while allowing older traces with fewer columns."""
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_bool(row: dict[str, str], name: str) -> bool | None:
    """Parse a CSV Boolean, preserving an empty/missing value as ``None``."""
    value = row.get(name, "").strip().lower()
    if not value:
        return None
    if value in {"true", "1", "yes"}:
        return True
    if value in {"false", "0", "no"}:
        return False
    return None


def as_float(row: dict[str, str], name: str) -> float | None:
    """Parse a numeric CSV field without turning missing diagnostics into zero."""
    value = row.get(name, "").strip()
    return None if not value else float(value)


def nonempty(values: Iterable[float | None]) -> list[float]:
    """Drop unavailable diagnostic values while keeping real zeroes."""
    return [value for value in values if value is not None]


def changed_events(rows: list[dict[str, str]], dx_name: str, dy_name: str) -> int:
    """Count distinct non-zero applied corrections in a latest-value trace.

    The logger samples the latest ROS topic values, so one applied correction
    can appear in many consecutive trace rows.  Counting value changes avoids
    mistaking repeated samples for repeated corrections.
    """
    previous: tuple[float, float] | None = None
    events = 0
    for row in rows:
        dx = as_float(row, dx_name)
        dy = as_float(row, dy_name)
        if dx is None or dy is None:
            continue
        current = (dx, dy)
        if current != previous and (abs(dx) > 1.0e-9 or abs(dy) > 1.0e-9):
            events += 1
        previous = current
    return events


def diagnose(evaluation: dict[str, str], trace: list[dict[str, str]]) -> dict[str, object]:
    """Derive compact evidence and a conservative failure classification."""
    physical_success = as_bool(evaluation, "ground_truth_goal_reached")
    navigation_success = as_bool(evaluation, "navigation_pose_goal_reached")
    state_success = as_bool(evaluation, "state_estimate_goal_reached")

    statuses = Counter(
        row["localization_match_status"].strip()
        for row in trace
        if row.get("localization_match_status", "").strip()
    )
    valid_rows = sum(
        as_bool(row, "localization_match_valid") is True for row in trace
    )
    candidate_values = nonempty(
        as_float(row, "localization_candidate_correction_m") for row in trace
    )
    applied_values = nonempty(
        as_float(row, "localization_applied_correction_m") for row in trace
    )
    score_improvements = nonempty(
        as_float(row, "localization_score_improvement_m") for row in trace
    )

    if physical_success is True:
        classification = "physical_goal_reached"
    elif navigation_success is True or state_success is True:
        classification = "estimated_goal_without_physical_goal"
    else:
        classification = "no_pose_reached_goal"

    if (
        classification == "estimated_goal_without_physical_goal"
        and candidate_values
        and (not applied_values or max(applied_values) <= 1.0e-9)
    ):
        bottleneck = "localization_candidate_not_applied"
    elif (
        classification == "estimated_goal_without_physical_goal"
        and applied_values
        and max(applied_values) < max(candidate_values or [0.0])
    ):
        bottleneck = "localization_correction_was_smoothed_or_limited"
    elif statuses:
        bottleneck = f"latest_localization_status:{statuses.most_common(1)[0][0]}"
    else:
        bottleneck = "insufficient_localization_diagnostics"

    return {
        "classification": classification,
        "bottleneck": bottleneck,
        "physical_success": physical_success,
        "state_estimate_success": state_success,
        "navigation_pose_success": navigation_success,
        "ground_truth_final_error_m": as_float(evaluation, "ground_truth_final_error_m"),
        "state_estimate_final_error_m": as_float(
            evaluation, "state_estimate_final_error_m"
        ),
        "navigation_pose_final_error_m": as_float(
            evaluation, "navigation_pose_final_error_m"
        ),
        "navigation_pose_goal_error_gap_m": as_float(
            evaluation, "navigation_pose_goal_error_gap_m"
        ),
        "localization_valid_samples": valid_rows,
        "localization_applied_events": changed_events(
            trace,
            "localization_applied_dx_m",
            "localization_applied_dy_m",
        ),
        "candidate_correction_max_m": max(candidate_values) if candidate_values else None,
        "applied_correction_max_m": max(applied_values) if applied_values else None,
        "score_improvement_max_m": (
            max(score_improvements) if score_improvements else None
        ),
        "localization_status_counts": dict(statuses),
        "termination_reason": evaluation.get("termination_reason", ""),
    }


def print_report(report: dict[str, object], evaluation_path: Path) -> None:
    """Print a human-readable report suitable for a lab notebook."""
    print(f"Navigation diagnosis: {evaluation_path}")
    print(f"classification: {report['classification']}")
    print(f"bottleneck: {report['bottleneck']}")
    print(f"termination_reason: {report['termination_reason']}")
    print(f"physical_success: {report['physical_success']}")
    print(f"state_estimate_success: {report['state_estimate_success']}")
    print(f"navigation_pose_success: {report['navigation_pose_success']}")
    print(f"ground_truth_final_error_m: {report['ground_truth_final_error_m']}")
    print(f"state_estimate_final_error_m: {report['state_estimate_final_error_m']}")
    print(f"navigation_pose_final_error_m: {report['navigation_pose_final_error_m']}")
    print(f"navigation_pose_goal_error_gap_m: {report['navigation_pose_goal_error_gap_m']}")
    print(f"localization_valid_samples: {report['localization_valid_samples']}")
    print(f"localization_applied_events: {report['localization_applied_events']}")
    print(f"candidate_correction_max_m: {report['candidate_correction_max_m']}")
    print(f"applied_correction_max_m: {report['applied_correction_max_m']}")
    print(f"score_improvement_max_m: {report['score_improvement_max_m']}")
    print("localization_status_counts:")
    for status, count in sorted(report["localization_status_counts"].items()):
        print(f"  {status}: {count}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evaluation", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    args = parser.parse_args()
    report = diagnose(read_one_row(args.evaluation), read_rows(args.trace))
    print_report(report, args.evaluation)


if __name__ == "__main__":
    main()
