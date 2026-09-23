#!/usr/bin/env python3
"""Build an explicit /odom, /state_estimate, and /localized_estimate table."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


OUTPUT_FIELDS = [
    "configuration",
    "scenario",
    "backend",
    "runs",
    "successes",
    "success_rate",
    "mean_odom_goal_error_m",
    "mean_localized_estimate_goal_error_m",
    "mean_state_estimate_goal_error_m",
    "mean_localized_estimate_truth_error_m",
    "mean_state_estimate_truth_error_m",
    "localized_estimate_independent_runs",
    "localized_estimate_fallback_runs",
    "localization_statuses",
]


def read_summary(path: Path, configuration: str, backend: str) -> list[dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(f"summary not found: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    result: list[dict[str, str]] = []
    for row in rows:
        if row.get("backend", "") != backend:
            continue
        result.append(
            {
                "configuration": configuration,
                "scenario": row.get("scenario", ""),
                "backend": row.get("backend", ""),
                "runs": row.get("runs", ""),
                "successes": row.get("successes", ""),
                "success_rate": row.get("success_rate", ""),
                "mean_odom_goal_error_m": row.get(
                    "mean_ground_truth_goal_error_m", ""
                ),
                "mean_localized_estimate_goal_error_m": row.get(
                    "mean_navigation_goal_error_m", ""
                ),
                "mean_state_estimate_goal_error_m": row.get(
                    "mean_state_estimate_goal_error_m", ""
                ),
                "mean_localized_estimate_truth_error_m": row.get(
                    "mean_navigation_truth_error_m", ""
                ),
                "mean_state_estimate_truth_error_m": row.get(
                    "mean_state_estimate_truth_error_m", ""
                ),
                "localized_estimate_independent_runs": row.get(
                    "navigation_independent_runs", ""
                ),
                "localized_estimate_fallback_runs": row.get(
                    "navigation_fallback_runs", ""
                ),
                "localization_statuses": row.get("localization_statuses", ""),
            }
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline-dir",
        type=Path,
        default=Path("results/localization_matrix"),
    )
    parser.add_argument(
        "--validated-dir",
        type=Path,
        default=Path("results/estimator_validation_ekf_fixedgyro"),
    )
    parser.add_argument("--backend", default="v4")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/estimator_comparison_summary.csv"),
    )
    args = parser.parse_args()

    rows = []
    rows.extend(
        read_summary(
            args.baseline_dir / "localization_drift_summary.csv",
            "original_fixed_fusion",
            args.backend,
        )
    )
    rows.extend(
        read_summary(
            args.validated_dir / "localization_drift_summary.csv",
            "validated_ekf",
            args.backend,
        )
    )
    if not rows:
        raise SystemExit(f"no backend={args.backend!r} rows found")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} estimator comparison rows to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
