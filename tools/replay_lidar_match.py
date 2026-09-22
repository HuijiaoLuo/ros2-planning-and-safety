#!/usr/bin/env python3
"""Replay LiDAR-map matcher decisions from a localizer JSONL audit file.

The localizer writes immutable map and scan snapshots before matching and the
corresponding optimizer result afterwards.  This tool reruns only the pure
``LidarMapMatcher``; it does not start ROS, move the robot, or apply gates.
Therefore a replay mismatch points to a changing input/configuration or a
problem in the live process, while an exact replay with a rejected status
points to the explicit gate logic.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "robotics_nav"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotics_nav.lidar_localization import LidarMapMatcher  # noqa: E402


def read_records(path: Path) -> list[dict[str, object]]:
    """Load JSONL records while retaining their original order."""
    records: list[dict[str, object]] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON on line {line_number}: {exc}") from exc
            if not isinstance(record, dict):
                raise ValueError(f"line {line_number} is not a JSON object")
            records.append(record)
    return records


def pose_error(first: list[object], second: list[object]) -> float:
    """Return translational/yaw-independent position error between poses."""
    return math.hypot(float(first[0]) - float(second[0]), float(first[1]) - float(second[1]))


def replay(path: Path, selected_id: int | None = None) -> dict[str, object]:
    """Replay all complete input/result pairs in an audit file."""
    records = read_records(path)
    config = next((r for r in records if r.get("record_type") == "config"), None)
    map_record = next((r for r in records if r.get("record_type") == "map"), None)
    if config is None or map_record is None:
        raise ValueError("audit file must contain config and map records")

    matcher_config = config.get("matcher")
    if not isinstance(matcher_config, dict):
        raise ValueError("config record has no matcher configuration")
    matcher = LidarMapMatcher(**matcher_config)
    matcher.update_map(
        width=int(map_record["width"]),
        height=int(map_record["height"]),
        resolution=float(map_record["resolution_m"]),
        origin_x=float(map_record["origin_x_m"]),
        origin_y=float(map_record["origin_y_m"]),
        data=[int(value) for value in map_record["data"]],  # type: ignore[arg-type]
    )

    inputs = {
        int(record["match_id"]): record
        for record in records
        if record.get("record_type") == "match_input"
    }
    results = {
        int(record["match_id"]): record
        for record in records
        if record.get("record_type") == "match_result"
    }
    ids = sorted(set(inputs) & set(results))
    if selected_id is not None:
        ids = [match_id for match_id in ids if match_id == selected_id]

    analyzed_results = [results[match_id] for match_id in ids]
    match_ages = [
        float(record["match_age_s"])
        for record in analyzed_results
        if record.get("match_age_s") is not None
    ]
    worker_times = [
        float(record["worker_compute_time_s"])
        for record in analyzed_results
        if record.get("worker_compute_time_s") is not None
    ]
    odom_motion_during_match = [
        float(record["odom_motion_during_match_m"])
        for record in analyzed_results
        if record.get("odom_motion_during_match_m") is not None
    ]
    odom_yaw_change_during_match = [
        abs(float(record["odom_yaw_change_during_match_rad"]))
        for record in analyzed_results
        if record.get("odom_yaw_change_during_match_rad") is not None
    ]
    status_counts = Counter(
        str(record.get("status", "unknown")) for record in analyzed_results
    )
    rejection_reason_counts = Counter(
        reason
        for record in analyzed_results
        for reason in record.get("rejection_reasons", [])  # type: ignore[union-attr]
    )
    accepted_records = sum(
        bool(record.get("match_valid", False)) for record in analyzed_results
    )
    quality_valid_records = sum(
        bool(record.get("quality_valid", False)) for record in analyzed_results
    )
    candidate_not_applied_records = sum(
        float(record.get("candidate_correction_m", 0.0)) > 1.0e-9
        and math.hypot(
            float(record.get("correction_x_m", 0.0)),
            float(record.get("correction_y_m", 0.0)),
        )
        <= 1.0e-9
        for record in analyzed_results
    )
    candidate_values = [
        float(record.get("candidate_correction_m", 0.0))
        for record in analyzed_results
    ]
    applied_values = [
        math.hypot(
            float(record.get("correction_x_m", 0.0)),
            float(record.get("correction_y_m", 0.0)),
        )
        for record in analyzed_results
    ]

    pose_tolerance = 1.0e-9
    score_tolerance = 1.0e-9
    mismatches: list[dict[str, object]] = []
    max_pose_error = 0.0
    max_score_error = 0.0
    point_count_mismatches = 0
    for match_id in ids:
        scan = inputs[match_id]
        recorded = results[match_id]
        ranges = [
            float(value) if value is not None else float("inf")
            for value in scan["ranges"]  # type: ignore[index]
        ]
        replayed = matcher.match_pose(
            float(scan["map_pose"][0]),  # type: ignore[index]
            float(scan["map_pose"][1]),  # type: ignore[index]
            float(scan["map_pose"][2]),  # type: ignore[index]
            ranges,
            angle_min=float(scan["angle_min"]),
            angle_increment=float(scan["angle_increment"]),
            range_min=float(scan["range_min"]),
            range_max=float(scan["range_max"]),
        )
        replayed_x, replayed_y, replayed_yaw, replayed_score, replayed_points = replayed
        recorded_pose = recorded["matched_pose"]  # type: ignore[assignment]
        replayed_pose = [replayed_x, replayed_y, replayed_yaw]
        current_pose_error = pose_error(recorded_pose, replayed_pose)
        recorded_score = float(recorded["score_m"])
        current_score_error = abs(recorded_score - replayed_score)
        max_pose_error = max(max_pose_error, current_pose_error)
        max_score_error = max(max_score_error, current_score_error)
        points_differ = int(recorded["point_count"]) != replayed_points
        point_count_mismatches += int(points_differ)
        if (
            current_pose_error > pose_tolerance
            or current_score_error > score_tolerance
            or points_differ
        ):
            mismatches.append(
                {
                    "match_id": match_id,
                    "status": recorded.get("status"),
                    "recorded_pose": recorded_pose,
                    "replayed_pose": replayed_pose,
                    "recorded_score_m": recorded_score,
                    "replayed_score_m": replayed_score,
                    "recorded_points": recorded.get("point_count"),
                    "replayed_points": replayed_points,
                }
            )

    return {
        "input_records": len(inputs),
        "result_records": len(results),
        "replayed_records": len(ids),
        "status_counts": dict(status_counts),
        "rejection_reason_counts": dict(rejection_reason_counts),
        "quality_valid_records": quality_valid_records,
        "accepted_records": accepted_records,
        "candidate_not_applied_records": candidate_not_applied_records,
        "candidate_correction_max_m": max(candidate_values, default=0.0),
        "applied_correction_max_m": max(applied_values, default=0.0),
        "max_pose_error_m": max_pose_error,
        "max_score_error_m": max_score_error,
        "point_count_mismatches": point_count_mismatches,
        "mismatch_count": len(mismatches),
        "match_age_mean_s": statistics.mean(match_ages) if match_ages else None,
        "match_age_max_s": max(match_ages, default=None),
        "worker_compute_time_mean_s": (
            statistics.mean(worker_times) if worker_times else None
        ),
        "worker_compute_time_max_s": max(worker_times, default=None),
        "odom_motion_mean_m": (
            statistics.mean(odom_motion_during_match)
            if odom_motion_during_match
            else None
        ),
        "odom_motion_max_m": max(odom_motion_during_match, default=None),
        "odom_yaw_change_mean_rad": (
            statistics.mean(odom_yaw_change_during_match)
            if odom_yaw_change_during_match
            else None
        ),
        "odom_yaw_change_max_rad": max(
            odom_yaw_change_during_match,
            default=None,
        ),
        "unfinished_input_ids": sorted(set(inputs) - set(results)),
        "mismatches": mismatches,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--match-id", type=int, default=None)
    args = parser.parse_args()
    report = replay(args.audit, args.match_id)
    print(f"audit: {args.audit}")
    for name in (
        "input_records",
        "result_records",
        "replayed_records",
        "quality_valid_records",
        "accepted_records",
        "candidate_not_applied_records",
        "candidate_correction_max_m",
        "applied_correction_max_m",
        "max_pose_error_m",
        "max_score_error_m",
        "point_count_mismatches",
        "mismatch_count",
        "match_age_mean_s",
        "match_age_max_s",
        "worker_compute_time_mean_s",
        "worker_compute_time_max_s",
        "odom_motion_mean_m",
        "odom_motion_max_m",
        "odom_yaw_change_mean_rad",
        "odom_yaw_change_max_rad",
        "unfinished_input_ids",
    ):
        print(f"{name}: {report[name]}")
    print("status_counts:")
    for status, count in sorted(report["status_counts"].items()):
        print(f"  {status}: {count}")
    print("rejection_reason_counts:")
    for reason, count in sorted(report["rejection_reason_counts"].items()):
        print(f"  {reason}: {count}")
    for mismatch in report["mismatches"][:10]:
        print(json.dumps(mismatch, separators=(",", ":")))


if __name__ == "__main__":
    main()
