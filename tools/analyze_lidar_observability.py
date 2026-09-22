#!/usr/bin/env python3
"""Inspect local x/y observability of recorded LiDAR-map scan matches.

The live localizer searches a small x/y window while keeping the fused heading
as the orientation prior.  This diagnostic reuses an audit file and evaluates
the score at the prior pose and at its four immediate x/y neighbours.  It does
not start ROS, change any gate, or apply a correction.  A clear score drop on
one axis is evidence that the scan contains local translation information; a
flat or nearly tied neighbourhood is an observability limitation rather than
an acceptance-threshold problem.
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
    """Read JSONL audit records and reject malformed non-object lines."""
    records: list[dict[str, object]] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            record = json.loads(line)
            if not isinstance(record, dict):
                raise ValueError(f"line {line_number} is not a JSON object")
            records.append(record)
    return records


def make_matcher(
    records: list[dict[str, object]],
    score_mode: str | None = None,
) -> LidarMapMatcher:
    """Reconstruct the exact matcher and static map from the audit header."""
    config = next(
        record for record in records if record.get("record_type") == "config"
    )
    map_record = next(
        record for record in records if record.get("record_type") == "map"
    )
    matcher_config = config["matcher"]
    if not isinstance(matcher_config, dict):
        raise ValueError("config record has no matcher configuration")
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


def score_at(
    matcher: LidarMapMatcher,
    scan: dict[str, object],
    x: float,
    y: float,
) -> float:
    """Score one fixed-heading candidate using the recorded scan."""
    ranges = [
        float(value) if value is not None else float("inf")
        for value in scan["ranges"]  # type: ignore[index]
    ]
    score, _ = matcher.score_pose(
        x,
        y,
        float(scan["map_pose"][2]),  # type: ignore[index]
        ranges,
        angle_min=float(scan["angle_min"]),
        angle_increment=float(scan["angle_increment"]),
        range_min=float(scan["range_min"]),
        range_max=float(scan["range_max"]),
    )
    return score


def analyze(
    path: Path,
    selected_ids: set[int] | None = None,
    score_mode: str | None = None,
) -> list[dict[str, object]]:
    """Return local score-surface diagnostics for complete audit records."""
    records = read_records(path)
    matcher = make_matcher(records, score_mode=score_mode)
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
    output: list[dict[str, object]] = []
    step = matcher.search_step_m
    for match_id in sorted(set(inputs) & set(results)):
        if selected_ids is not None and match_id not in selected_ids:
            continue
        scan = inputs[match_id]
        result = results[match_id]
        prior_x, prior_y = (
            float(scan["map_pose"][0]),  # type: ignore[index]
            float(scan["map_pose"][1]),  # type: ignore[index]
        )
        scores = {
            "prior": score_at(matcher, scan, prior_x, prior_y),
            "x_minus": score_at(matcher, scan, prior_x - step, prior_y),
            "x_plus": score_at(matcher, scan, prior_x + step, prior_y),
            "y_minus": score_at(matcher, scan, prior_x, prior_y - step),
            "y_plus": score_at(matcher, scan, prior_x, prior_y + step),
        }
        finite_neighbours = [
            value for name, value in scores.items() if name != "prior" and math.isfinite(value)
        ]
        best_neighbour = min(finite_neighbours, default=float("inf"))
        x_best = min(scores["x_minus"], scores["x_plus"])
        y_best = min(scores["y_minus"], scores["y_plus"])
        prior_score = scores["prior"]
        output.append(
            {
                "match_id": match_id,
                "status": result.get("status"),
                "point_count": result.get("point_count"),
                "prior_score_m": prior_score,
                "best_neighbour_score_m": best_neighbour,
                "x_gain_m": prior_score - x_best,
                "y_gain_m": prior_score - y_best,
                "neighbour_spread_m": (
                    max(finite_neighbours) - min(finite_neighbours)
                    if finite_neighbours
                    else float("inf")
                ),
                "candidate_dx_m": result.get("candidate_dx_m"),
                "candidate_dy_m": result.get("candidate_dy_m"),
                "candidate_correction_m": result.get("candidate_correction_m"),
                "score_improvement_m": result.get("score_improvement_m"),
                "scores": scores,
            }
        )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--match-id", type=int, action="append")
    parser.add_argument(
        "--score-mode",
        choices=("range", "endpoint", "boundary"),
        default="range",
        help="Score model to diagnose; range is the production baseline.",
    )
    args = parser.parse_args()
    selected_ids = set(args.match_id) if args.match_id else None
    rows = analyze(args.audit, selected_ids, score_mode=args.score_mode)
    print(f"audit: {args.audit}")
    print(f"score_mode: {args.score_mode}")
    print("id status points prior x_gain y_gain neighbour_spread candidate_dx candidate_dy")

    def optional_float(value: object, digits: int = 3) -> str:
        """Format fields that are absent in older audit files."""
        if value is None:
            return "n/a"
        return f"{float(value):+.{digits}f}"

    for row in rows:
        print(
            f"{row['match_id']:>2} {str(row['status']):<30} "
            f"{int(row['point_count']):>2} "
            f"{float(row['prior_score_m']):.4f} "
            f"{float(row['x_gain_m']):+.4f} "
            f"{float(row['y_gain_m']):+.4f} "
            f"{float(row['neighbour_spread_m']):.4f} "
            f"{optional_float(row['candidate_dx_m'])} "
            f"{optional_float(row['candidate_dy_m'])}"
        )


if __name__ == "__main__":
    main()
