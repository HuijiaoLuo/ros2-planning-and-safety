#!/usr/bin/env python3
"""Check whether a LiDAR score model is self-consistent on its own map.

For each selected recorded pose, this diagnostic synthesizes a scan with the
same occupancy-grid raycaster used by the matcher.  A self-consistent score
should recover the generating pose, up to the configured search resolution.
The recorded scan is also replayed for comparison.  This separates a scoring
or map-raycast defect from a Gazebo sensor/map-frame mismatch without starting
ROS or changing any acceptance gate.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


TOOLS_ROOT = Path(__file__).parent
sys.path.insert(0, str(TOOLS_ROOT))

from analyze_lidar_observability import make_matcher, read_records  # noqa: E402


SCORE_MODES = ("range", "endpoint", "boundary", "point_to_line")


def scan_arguments(scan: dict[str, object]) -> dict[str, float]:
    """Extract scalar LaserScan metadata from one audit input."""
    return {
        "angle_min": float(scan["angle_min"]),
        "angle_increment": float(scan["angle_increment"]),
        "range_min": float(scan["range_min"]),
        "range_max": float(scan["range_max"]),
    }


def recorded_ranges(scan: dict[str, object]) -> list[float]:
    """Convert JSON null returns back to the matcher convention."""
    return [
        float(value) if value is not None else float("inf")
        for value in scan["ranges"]  # type: ignore[index]
    ]


def synthetic_ranges(matcher: object, scan: dict[str, object]) -> list[float]:
    """Generate a scan using the exact map raycaster inside the matcher."""
    metadata = scan_arguments(scan)
    pose = scan["map_pose"]  # type: ignore[index]
    raycast = getattr(matcher, "_raycast_range")
    output: list[float] = []
    for index, _ in enumerate(scan["ranges"]):  # type: ignore[index]
        angle = metadata["angle_min"] + index * metadata["angle_increment"]
        distance = raycast(
            float(pose[0]),
            float(pose[1]),
            float(pose[2]) + angle,
            range_max=metadata["range_max"],
        )
        output.append(distance if distance is not None else float("inf"))
    return output


def run_match(matcher: object, scan: dict[str, object], ranges: list[float]) -> dict[str, float | int]:
    """Run the pure matcher and report displacement from the generating pose."""
    metadata = scan_arguments(scan)
    pose = scan["map_pose"]  # type: ignore[index]
    result = getattr(matcher, "match_pose")(
        float(pose[0]),
        float(pose[1]),
        float(pose[2]),
        ranges,
        **metadata,
        yaw_search_radius_rad=0.0,
    )
    matched_x, matched_y, _matched_yaw, score, point_count = result
    dx = float(matched_x) - float(pose[0])
    dy = float(matched_y) - float(pose[1])
    diagnostics = getattr(matcher, "last_match_diagnostics", {})
    return {
        "matched_dx_m": dx,
        "matched_dy_m": dy,
        "matched_correction_m": math.hypot(dx, dy),
        "score_m": float(score),
        "point_count": int(point_count),
        "score_margin_m": diagnostics.get("score_margin_m"),
    }


def analyze(path: Path, match_ids: set[int] | None = None) -> list[dict[str, object]]:
    """Run synthetic and recorded self-consistency checks."""
    records = read_records(path)
    inputs = {
        int(record["match_id"]): record
        for record in records
        if record.get("record_type") == "match_input"
    }
    results: list[dict[str, object]] = []
    ids = sorted(inputs)
    if match_ids is not None:
        ids = [match_id for match_id in ids if match_id in match_ids]
    for match_id in ids:
        scan = inputs[match_id]
        for score_mode in SCORE_MODES:
            matcher = make_matcher(records, score_mode=score_mode)
            synthetic = run_match(matcher, scan, synthetic_ranges(matcher, scan))
            recorded = run_match(matcher, scan, recorded_ranges(scan))
            results.append(
                {
                    "match_id": match_id,
                    "score_mode": score_mode,
                    "synthetic_dx_m": synthetic["matched_dx_m"],
                    "synthetic_dy_m": synthetic["matched_dy_m"],
                    "synthetic_correction_m": synthetic["matched_correction_m"],
                    "synthetic_score_m": synthetic["score_m"],
                    "synthetic_points": synthetic["point_count"],
                    "recorded_dx_m": recorded["matched_dx_m"],
                    "recorded_dy_m": recorded["matched_dy_m"],
                    "recorded_correction_m": recorded["matched_correction_m"],
                    "recorded_score_m": recorded["score_m"],
                    "recorded_points": recorded["point_count"],
                }
            )
    return results


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    """Write consistency results for later model comparison."""
    fields = list(rows[0]) if rows else ["match_id", "score_mode"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--match-id", type=int, action="append")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rows = analyze(args.audit, set(args.match_id) if args.match_id else None)
    print(f"audit: {args.audit}")
    print("mode match synthetic_error recorded_error synthetic_score recorded_score")
    for row in rows:
        print(
            f"{row['score_mode']:<12} {int(row['match_id']):>5} "
            f"{float(row['synthetic_correction_m']):.4f} "
            f"{float(row['recorded_correction_m']):.4f} "
            f"{float(row['synthetic_score_m']):.4f} "
            f"{float(row['recorded_score_m']):.4f}"
        )
    if args.output:
        write_csv(args.output, rows)
        print(f"wrote: {args.output}")


if __name__ == "__main__":
    main()
