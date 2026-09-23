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
import csv
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
            "xy_minus_minus": score_at(
                matcher, scan, prior_x - step, prior_y - step
            ),
            "xy_minus_plus": score_at(
                matcher, scan, prior_x - step, prior_y + step
            ),
            "xy_plus_minus": score_at(
                matcher, scan, prior_x + step, prior_y - step
            ),
            "xy_plus_plus": score_at(
                matcher, scan, prior_x + step, prior_y + step
            ),
        }
        finite_neighbours = [
            value for name, value in scores.items() if name != "prior" and math.isfinite(value)
        ]
        best_neighbour = min(finite_neighbours, default=float("inf"))
        x_best = min(scores["x_minus"], scores["x_plus"])
        y_best = min(scores["y_minus"], scores["y_plus"])
        prior_score = scores["prior"]
        search_diagnostics = result.get("search_diagnostics", {})
        if not isinstance(search_diagnostics, dict):
            search_diagnostics = {}
        top_candidates = search_diagnostics.get("top_candidates", [])
        if not isinstance(top_candidates, list):
            top_candidates = []
        top_positions = [
            (float(candidate["x_m"]), float(candidate["y_m"]))
            for candidate in top_candidates
            if isinstance(candidate, dict)
            and isinstance(candidate.get("x_m"), (int, float))
            and isinstance(candidate.get("y_m"), (int, float))
        ]
        pairwise_distances = [
            math.hypot(first[0] - second[0], first[1] - second[1])
            for index, first in enumerate(top_positions)
            for second in top_positions[index + 1 :]
        ]
        candidate_mahalanobis_sq = result.get(
            "candidate_position_mahalanobis_sq"
        )
        max_candidate_mahalanobis_sq = result.get(
            "max_candidate_mahalanobis_sq"
        )
        mahalanobis_gate_ratio = None
        if (
            isinstance(candidate_mahalanobis_sq, (int, float))
            and isinstance(max_candidate_mahalanobis_sq, (int, float))
            and float(max_candidate_mahalanobis_sq) > 0.0
        ):
            mahalanobis_gate_ratio = (
                float(candidate_mahalanobis_sq)
                / float(max_candidate_mahalanobis_sq)
            )

        def local_curvature(minus: float, centre: float, plus: float) -> float | None:
            """Approximate score-surface curvature along one axis."""
            if not all(math.isfinite(value) for value in (minus, centre, plus)):
                return None
            return (minus + plus - 2.0 * centre) / (step * step)

        x_curvature = local_curvature(
            scores["x_minus"], prior_score, scores["x_plus"]
        )
        y_curvature = local_curvature(
            scores["y_minus"], prior_score, scores["y_plus"]
        )
        xy_curvature = None
        if all(
            math.isfinite(scores[name])
            for name in (
                "xy_minus_minus",
                "xy_minus_plus",
                "xy_plus_minus",
                "xy_plus_plus",
            )
        ):
            xy_curvature = (
                scores["xy_plus_plus"]
                - scores["xy_plus_minus"]
                - scores["xy_minus_plus"]
                + scores["xy_minus_minus"]
            ) / (4.0 * step * step)
        hessian_eigenvalues = (None, None)
        hessian_condition = None
        if x_curvature is not None and y_curvature is not None and xy_curvature is not None:
            hessian_trace = x_curvature + y_curvature
            hessian_discriminant = math.sqrt(
                max(
                    0.0,
                    (x_curvature - y_curvature) ** 2
                    + 4.0 * xy_curvature * xy_curvature,
                )
            )
            eigen_min = 0.5 * (hessian_trace - hessian_discriminant)
            eigen_max = 0.5 * (hessian_trace + hessian_discriminant)
            hessian_eigenvalues = (eigen_min, eigen_max)
            if abs(eigen_min) > 1.0e-12:
                hessian_condition = abs(eigen_max / eigen_min)

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
                "score_margin_m": result.get("score_margin_m"),
                "candidate_position_mahalanobis_sq": candidate_mahalanobis_sq,
                "max_candidate_mahalanobis_sq": max_candidate_mahalanobis_sq,
                "mahalanobis_gate_ratio": mahalanobis_gate_ratio,
                "match_age_s": result.get("match_age_s"),
                "odom_motion_during_match_m": result.get(
                    "odom_motion_during_match_m"
                ),
                "odom_yaw_change_during_match_rad": result.get(
                    "odom_yaw_change_during_match_rad"
                ),
                "x_curvature": x_curvature,
                "y_curvature": y_curvature,
                "xy_curvature": xy_curvature,
                "hessian_eigen_min": hessian_eigenvalues[0],
                "hessian_eigen_max": hessian_eigenvalues[1],
                "hessian_condition_abs": hessian_condition,
                "top_candidate_count": len(top_positions),
                "top_candidate_span_m": max(pairwise_distances, default=None),
                "top_candidate_min_spacing_m": min(
                    pairwise_distances, default=None
                ),
                "scores": scores,
            }
        )
    return output


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    """Write scalar observability fields without nested score dictionaries."""
    fieldnames = [
        "match_id",
        "status",
        "point_count",
        "prior_score_m",
        "best_neighbour_score_m",
        "x_gain_m",
        "y_gain_m",
        "x_curvature",
        "y_curvature",
        "xy_curvature",
        "hessian_eigen_min",
        "hessian_eigen_max",
        "hessian_condition_abs",
        "neighbour_spread_m",
        "candidate_dx_m",
        "candidate_dy_m",
        "candidate_correction_m",
        "score_improvement_m",
        "score_margin_m",
        "candidate_position_mahalanobis_sq",
        "max_candidate_mahalanobis_sq",
        "mahalanobis_gate_ratio",
        "match_age_s",
        "odom_motion_during_match_m",
        "odom_yaw_change_during_match_rad",
        "top_candidate_count",
        "top_candidate_span_m",
        "top_candidate_min_spacing_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def print_summary(rows: list[dict[str, object]]) -> None:
    """Print gate and local-observability aggregates before any tuning."""
    status_counts = Counter(str(row.get("status", "unknown")) for row in rows)
    gate_ratios = [
        float(row["mahalanobis_gate_ratio"])
        for row in rows
        if isinstance(row.get("mahalanobis_gate_ratio"), (int, float))
    ]
    x_gains = [
        float(row["x_gain_m"])
        for row in rows
        if isinstance(row.get("x_gain_m"), (int, float))
        and math.isfinite(float(row["x_gain_m"]))
    ]
    y_gains = [
        float(row["y_gain_m"])
        for row in rows
        if isinstance(row.get("y_gain_m"), (int, float))
        and math.isfinite(float(row["y_gain_m"]))
    ]
    hessian_conditions = [
        float(row["hessian_condition_abs"])
        for row in rows
        if isinstance(row.get("hessian_condition_abs"), (int, float))
        and math.isfinite(float(row["hessian_condition_abs"]))
    ]
    print(f"records: {len(rows)}")
    print(f"status_counts: {dict(status_counts)}")
    if gate_ratios:
        print(
            "mahalanobis_gate_ratio: "
            f"mean={statistics.mean(gate_ratios):.3f}, "
            f"median={statistics.median(gate_ratios):.3f}, "
            f">1={sum(value > 1.0 for value in gate_ratios)}"
        )
    if x_gains and y_gains:
        print(
            "local_axis_gain_m: "
            f"x_mean={statistics.mean(x_gains):+.5f}, "
            f"y_mean={statistics.mean(y_gains):+.5f}, "
            f"x_nonzero={sum(value > 1.0e-6 for value in x_gains)}, "
            f"y_nonzero={sum(value > 1.0e-6 for value in y_gains)}"
        )
    if hessian_conditions:
        print(
            "local_hessian_condition_abs: "
            f"median={statistics.median(hessian_conditions):.3f}, "
            f">10={sum(value > 10.0 for value in hessian_conditions)}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--match-id", type=int, action="append")
    parser.add_argument(
        "--score-mode",
        choices=("range", "endpoint", "boundary", "point_to_line"),
        default=None,
        help="Override the score model; by default use the audit configuration.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional CSV output for scalar per-match observability fields.",
    )
    args = parser.parse_args()
    selected_ids = set(args.match_id) if args.match_id else None
    rows = analyze(args.audit, selected_ids, score_mode=args.score_mode)
    print(f"audit: {args.audit}")
    print(f"score_mode: {args.score_mode or 'audit-config'}")
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
    print_summary(rows)
    if args.output:
        write_csv(args.output, rows)
        print(f"wrote: {args.output}")


if __name__ == "__main__":
    main()
