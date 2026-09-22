#!/usr/bin/env python3
"""Compare temporal correction formulations on a LiDAR audit replay.

The localizer receives a scan paired with an older odometry pose and applies
the result after a CPU-bound worker delay.  This tool compares two equivalent
ways of expressing that update:

1. compute the desired ``map -> odom`` transform from the matched old pose;
2. compute the map-frame pose innovation at the old pose and apply that
   innovation to the transform that was active when the scan was submitted.

For the current estimator, the scan job is immutable and no second match can
update the transform while the worker is running.  The two formulations should
therefore agree.  A large difference would indicate a transform bookkeeping
bug; agreement means that the remaining temporal issue is time-varying drift
or weak scan observability, not a missing algebraic correction term.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "robotics_nav"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotics_nav.lidar_localization import (  # noqa: E402
    Pose2D,
    compose_pose,
    inverse_pose,
    interpolate_pose,
    map_odom_from_poses,
    transform_pose,
)


def read_records(path: Path) -> list[dict[str, object]]:
    """Read JSONL objects in their recorded order."""
    with path.open(encoding="utf-8") as handle:
        return [
            json.loads(line)
            for line in handle
            if line.strip()
        ]


def pose_distance(first: Pose2D, second: Pose2D) -> float:
    """Return Euclidean translation difference between two planar poses."""
    return math.hypot(first[0] - second[0], first[1] - second[1])


def pose_from(values: object) -> Pose2D:
    """Convert a JSON pose array into the project pose type."""
    sequence = values  # type: ignore[assignment]
    return (
        float(sequence[0]),  # type: ignore[index]
        float(sequence[1]),  # type: ignore[index]
        float(sequence[2]),  # type: ignore[index]
    )


def analyze(path: Path) -> list[dict[str, object]]:
    """Reconstruct accepted updates and compare direct/innovation forms."""
    records = read_records(path)
    results = sorted(
        (
            record
            for record in records
            if record.get("record_type") == "match_result"
        ),
        key=lambda record: int(record["match_id"]),
    )
    config = next(
        record for record in records if record.get("record_type") == "config"
    )
    gates = config.get("gates")
    if not isinstance(gates, dict):
        raise ValueError("audit has no gate configuration")
    smoothing = float(gates["correction_smoothing"])

    current_transform: Pose2D = (0.0, 0.0, 0.0)
    rows: list[dict[str, object]] = []
    for result in results:
        before = current_transform
        old_odom = pose_from(result["odom_pose"])
        prior_map_pose = pose_from(result["map_pose"])
        matched_pose = pose_from(result["matched_pose"])
        # The production localizer searches yaw only as a diagnostic
        # innovation.  Its persistent map->odom update deliberately keeps
        # the prior fused heading and applies translation only.
        matched_map_pose: Pose2D = (
            matched_pose[0],
            matched_pose[1],
            prior_map_pose[2],
        )

        # The direct method mirrors the production implementation: a scan
        # result is converted from its paired old odometry pose into a desired
        # map->odom transform, then smoothed toward the current transform.
        desired_direct = map_odom_from_poses(matched_map_pose, old_odom)
        direct_after = interpolate_pose(before, desired_direct, smoothing)

        # The innovation method expresses the same information as a local
        # map-frame correction around the pose that was active at submission.
        # Applying that correction to `before` avoids confusing a local scan
        # innovation with a new absolute frame origin.
        predicted_prior_map_pose = transform_pose(before, old_odom)
        map_innovation = map_odom_from_poses(
            matched_map_pose,
            predicted_prior_map_pose,
        )
        innovation_target = compose_pose(map_innovation, before)
        innovation_after = interpolate_pose(
            before,
            innovation_target,
            smoothing,
        )

        recorded_after = (
            pose_from(result["map_odom_after"])
            if result.get("map_odom_after") is not None
            else None
        )
        accepted = bool(result.get("match_valid", False))
        if accepted:
            current_transform = direct_after

        rows.append(
            {
                "match_id": int(result["match_id"]),
                "status": result.get("status"),
                "accepted": accepted,
                "direct_innovation_difference_m": pose_distance(
                    direct_after,
                    innovation_after,
                ),
                "direct_vs_recorded_m": (
                    pose_distance(direct_after, recorded_after)
                    if recorded_after is not None and accepted
                    else None
                ),
                "old_to_apply_motion_m": result.get(
                    "odom_motion_during_match_m"
                ),
                "candidate_correction_m": result.get(
                    "candidate_correction_m"
                ),
                "map_innovation_m": pose_distance(
                    map_innovation,
                    (0.0, 0.0, 0.0),
                ),
            }
        )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    args = parser.parse_args()
    rows = analyze(args.audit)
    print(f"audit: {args.audit}")
    print("id status accepted direct_vs_innovation_m direct_vs_recorded_m odom_motion_m")
    for row in rows:
        print(
            f"{row['match_id']:>2} {str(row['status']):<30} "
            f"{str(row['accepted']):<8} "
            f"{float(row['direct_innovation_difference_m']):.9f} "
            f"{row['direct_vs_recorded_m']} "
            f"{row['old_to_apply_motion_m']}"
        )
    accepted_rows = [row for row in rows if row["accepted"]]
    if accepted_rows:
        max_difference = max(
            float(row["direct_innovation_difference_m"])
            for row in accepted_rows
        )
        print(f"accepted_records: {len(accepted_rows)}")
        print(f"max_accepted_direct_vs_innovation_m: {max_difference:.9f}")


if __name__ == "__main__":
    main()
