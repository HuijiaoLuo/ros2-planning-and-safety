#!/usr/bin/env python3
"""Summarize timestamp and terminal-decision evidence from one run.

This tool is deliberately diagnostic: it reads the evaluation trace and final
metrics, but it never changes controller, estimator, or localization state.
It reports the controller events, the source-pose age embedded in each event,
logger-observed receipt ages, and the interval in which the controller held a
zero command before a possible terminal latch.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def number(value: object) -> float | None:
    """Parse one finite numeric CSV value."""
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def event_fields(payload: str) -> dict[str, str]:
    """Decode the semicolon-delimited controller event payload."""
    fields: dict[str, str] = {}
    for item in payload.split(";"):
        key, separator, value = item.partition("=")
        if separator:
            fields[key] = value
    return fields


def changed_events(rows: list[dict[str, str]]) -> list[tuple[int, dict[str, str]]]:
    """Return event transitions instead of repeating the latched last event."""
    transitions: list[tuple[int, dict[str, str]]] = []
    previous = ""
    for index, row in enumerate(rows):
        payload = row.get("controller_goal_event", "")
        if payload and payload != previous:
            transitions.append((index, event_fields(payload)))
        previous = payload
    return transitions


def finite_column(rows: list[dict[str, str]], field: str) -> list[float]:
    """Collect finite values from one trace column."""
    values: list[float] = []
    for row in rows:
        parsed = number(row.get(field))
        if parsed is not None:
            values.append(parsed)
    return values


def ratio_zero(rows: list[dict[str, str]], field: str) -> float | None:
    """Return the fraction of finite rows whose command component is zero."""
    values = finite_column(rows, field)
    if not values:
        return None
    return sum(abs(value) <= 1.0e-9 for value in values) / len(values)


def load_one_row(path: Path) -> dict[str, str] | None:
    """Read the first row of a one-run metrics CSV."""
    with path.open(newline="", encoding="utf-8") as handle:
        return next(csv.DictReader(handle), None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--metrics", type=Path)
    args = parser.parse_args()

    with args.trace.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    transitions = changed_events(rows)

    print(f"trace_file: {args.trace}")
    print(f"trace_samples: {len(rows)}")
    print(f"controller_event_transitions: {len(transitions)}")
    for index, fields in transitions:
        pose_age = number(fields.get("pose_age_s"))
        pose_age_text = fields.get("pose_age_s", "")
        if pose_age is not None and abs(pose_age) > 1000.0:
            pose_age_text += " [CLOCK_DOMAIN_MISMATCH]"
        print(
            "event[{index}]: {event} node_stamp_s={node} "
            "pose_stamp_s={pose} pose_age_s={age} goal_distance_m={distance}".format(
                index=index,
                event=fields.get("event", ""),
                node=fields.get("node_stamp_s", ""),
                pose=fields.get("pose_stamp_s", ""),
                age=pose_age_text,
                distance=fields.get("goal_distance_m", ""),
            )
        )

    for field in (
        "odom_receipt_age_s",
        "navigation_receipt_age_s",
        "state_estimate_receipt_age_s",
        "scan_receipt_age_s",
        "executed_command_receipt_age_s",
        "localization_status_receipt_age_s",
    ):
        values = finite_column(rows, field)
        if values:
            print(
                f"{field}: mean={sum(values) / len(values):.6f} "
                f"max={max(values):.6f} samples={len(values)}"
            )

    state_stamp_ages = []
    navigation_stamp_ages = []
    for row in rows:
        logger_stamp = number(row.get("logger_ros_timestamp_s"))
        state_stamp = number(row.get("state_estimate_timestamp_s"))
        navigation_stamp = number(row.get("navigation_timestamp_s"))
        if logger_stamp is not None and state_stamp is not None:
            state_stamp_ages.append(logger_stamp - state_stamp)
        if logger_stamp is not None and navigation_stamp is not None:
            navigation_stamp_ages.append(logger_stamp - navigation_stamp)
    for label, values in (
        ("state_estimate_header_age_s", state_stamp_ages),
        ("navigation_header_age_s", navigation_stamp_ages),
    ):
        if values:
            if max(abs(value) for value in values) > 1000.0:
                print(f"{label}: CLOCK_DOMAIN_MISMATCH samples={len(values)}")
            else:
                print(
                    f"{label}: mean={sum(values) / len(values):.6f} "
                    f"max={max(values):.6f} samples={len(values)}"
                )

    enter_index = next(
        (index for index, fields in transitions
         if fields.get("event") == "goal_tolerance_entered"),
        None,
    )
    latch_index = next(
        (index for index, fields in transitions
         if fields.get("event") == "goal_reached_latched"),
        None,
    )
    if enter_index is not None:
        end_index = latch_index if latch_index is not None else len(rows)
        hold_rows = rows[enter_index:end_index]
        print(f"pre_latch_hold_samples: {len(hold_rows)}")
        zero_linear = ratio_zero(hold_rows, "executed_linear_x_mps")
        zero_angular = ratio_zero(hold_rows, "executed_angular_z_radps")
        if zero_linear is not None:
            print(f"pre_latch_zero_linear_ratio: {zero_linear:.6f}")
        if zero_angular is not None:
            print(f"pre_latch_zero_angular_ratio: {zero_angular:.6f}")

    if args.metrics is not None and args.metrics.exists():
        metrics = load_one_row(args.metrics)
        if metrics is not None:
            print(f"metrics_file: {args.metrics}")
            for field in (
                "termination_reason",
                "navigation_pose_goal_reached",
                "state_estimate_goal_reached",
                "evaluation_final_errors_source",
                "evaluation_final_logger_ros_timestamp_s",
                "evaluation_final_navigation_timestamp_s",
                "evaluation_final_state_estimate_timestamp_s",
                "controller_goal_event_count",
            ):
                if field in metrics:
                    print(f"{field}: {metrics[field]}")
            history = metrics.get("controller_goal_event_history", "")
            if history:
                try:
                    print(f"controller_goal_event_history: {json.loads(history)}")
                except json.JSONDecodeError:
                    print(f"controller_goal_event_history: {history}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
