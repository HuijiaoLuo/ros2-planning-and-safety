#!/usr/bin/env python3
"""Test LiDAR-map recovery from synthetic known x/y pose offsets.

This is a stationary observability experiment built from a recorded map and
scan.  The scan is kept fixed while the matcher prior is deliberately moved
by a known offset.  A successful local correction should point back toward
the original prior: an injected ``(+dx, +dy)`` should produce a candidate
near ``(-dx, -dy)``.  The experiment does not start ROS or change any gate.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "robotics_nav"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotics_nav.lidar_localization import LidarMapMatcher  # noqa: E402


def read_records(path: Path) -> list[dict[str, object]]:
    """Read JSONL audit records."""
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def build_matcher(
    records: list[dict[str, object]],
    score_mode: str | None = None,
) -> LidarMapMatcher:
    """Reconstruct the exact static map and matcher configuration."""
    config = next(r for r in records if r.get("record_type") == "config")
    map_record = next(r for r in records if r.get("record_type") == "map")
    matcher_config = config["matcher"]
    if not isinstance(matcher_config, dict):
        raise ValueError("audit has no matcher configuration")
    matcher_config = dict(matcher_config)
    if score_mode is not None:
        matcher_config["score_mode"] = score_mode
    matcher = LidarMapMatcher(**matcher_config)
    matcher.update_map(
        width=int(map_record["width"]),
        height=int(map_record["height"]),
        resolution=float(map_record["resolution_m"]),
        origin_x=float(map_record["origin_x_m"]),
        origin_y=float(map_record["origin_y_m"]),
        data=[int(value) for value in map_record["data"]],  # type: ignore[arg-type]
    )
    return matcher


def run_case(
    matcher: LidarMapMatcher,
    scan: dict[str, object],
    offset_x: float,
    offset_y: float,
) -> dict[str, object]:
    """Run one synthetic prior offset with heading held fixed."""
    base_x, base_y, base_yaw = (
        float(scan["map_pose"][0]),  # type: ignore[index]
        float(scan["map_pose"][1]),  # type: ignore[index]
        float(scan["map_pose"][2]),  # type: ignore[index]
    )
    prior_x = base_x + offset_x
    prior_y = base_y + offset_y
    ranges = [
        float(value) if value is not None else float("inf")
        for value in scan["ranges"]  # type: ignore[index]
    ]
    matched_x, matched_y, _, score, point_count = matcher.match_pose(
        prior_x,
        prior_y,
        base_yaw,
        ranges,
        angle_min=float(scan["angle_min"]),
        angle_increment=float(scan["angle_increment"]),
        range_min=float(scan["range_min"]),
        range_max=float(scan["range_max"]),
        yaw_search_radius_rad=0.0,
    )
    correction_x = matched_x - prior_x
    correction_y = matched_y - prior_y
    residual_x = matched_x - base_x
    residual_y = matched_y - base_y
    return {
        "offset_x_m": offset_x,
        "offset_y_m": offset_y,
        "candidate_dx_m": correction_x,
        "candidate_dy_m": correction_y,
        "remaining_error_m": math.hypot(residual_x, residual_y),
        "remaining_dx_m": residual_x,
        "remaining_dy_m": residual_y,
        "score_m": score,
        "point_count": point_count,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--match-id", type=int, required=True)
    parser.add_argument(
        "--score-mode",
        choices=("range", "endpoint", "boundary"),
        default="range",
        help="Score model to diagnose; range is the production baseline.",
    )
    args = parser.parse_args()

    records = read_records(args.audit)
    matcher = build_matcher(records, args.score_mode)
    scan = next(
        record
        for record in records
        if record.get("record_type") == "match_input"
        and int(record["match_id"]) == args.match_id
    )
    offsets = (
        (0.05, 0.0),
        (-0.05, 0.0),
        (0.0, 0.05),
        (0.0, -0.05),
        (0.10, 0.0),
        (0.0, 0.10),
    )
    print(f"audit: {args.audit}")
    print(f"match_id: {args.match_id}")
    print(f"score_mode: {args.score_mode}")
    print("offset_dx offset_dy candidate_dx candidate_dy remaining_error score points")
    for offset_x, offset_y in offsets:
        result = run_case(matcher, scan, offset_x, offset_y)
        print(
            f"{offset_x:+.3f} {offset_y:+.3f} "
            f"{float(result['candidate_dx_m']):+.3f} "
            f"{float(result['candidate_dy_m']):+.3f} "
            f"{float(result['remaining_error_m']):.3f} "
            f"{float(result['score_m']):.4f} "
            f"{int(result['point_count'])}"
        )


if __name__ == "__main__":
    main()
