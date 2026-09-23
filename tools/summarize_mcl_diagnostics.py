#!/usr/bin/env python3
"""Summarize particle-filter JSONL diagnostics separately from local matches."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


def finite_values(rows: list[dict[str, object]], key: str) -> list[float]:
    values = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return values


def summarize(path: Path) -> dict[str, object]:
    rows: list[dict[str, object]] = []
    belief_rows: list[dict[str, object]] = []
    config: dict[str, object] = {}
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("record_type") == "config":
                config = record
            elif record.get("record_type") == "mcl_update":
                rows.append(record)
            elif record.get("record_type") == "mcl_belief":
                belief_rows.append(record)

    valid = [row for row in rows if row.get("status") == "accepted"]
    status_counts = Counter(str(row.get("status", "unknown")) for row in rows)

    def mean(key: str) -> float | None:
        values = finite_values(rows, key)
        return sum(values) / len(values) if values else None

    def minimum(key: str) -> float | None:
        values = finite_values(rows, key)
        return min(values) if values else None

    def maximum(key: str) -> float | None:
        values = finite_values(rows, key)
        return max(values) if values else None

    def accepted_mean(key: str) -> float | None:
        values = finite_values(valid, key)
        return sum(values) / len(values) if values else None

    result: dict[str, object] = {
        "diagnostic_file": str(path),
        "update_count": len(rows),
        "belief_event_count": len(belief_rows),
        "belief_measurement_applied_count": sum(
            bool(row.get("measurement_applied")) for row in belief_rows
        ),
        "belief_status_counts": json.dumps(
            dict(Counter(str(row.get("status", "unknown")) for row in belief_rows)),
            sort_keys=True,
        ),
        "accepted_update_count": len(valid),
        "accepted_update_ratio": len(valid) / len(rows) if rows else None,
        "status_counts": json.dumps(dict(status_counts), sort_keys=True),
        "stale_scan_count": status_counts.get("stale_scan", 0),
        "scan_ahead_of_pose_count": status_counts.get("scan_ahead_of_pose", 0),
        "insufficient_points_count": status_counts.get("insufficient_points", 0),
        "uncertainty_rejection_count": status_counts.get(
            "candidate_uncertainty_too_large", 0
        ),
        "not_better_than_prior_count": status_counts.get(
            "candidate_not_better_than_prior", 0
        ),
        "inconsistent_with_prior_count": status_counts.get(
            "candidate_inconsistent_with_prior", 0
        ),
        "measurement_applied_count": sum(
            bool(row.get("measurement_applied")) for row in rows
        ),
        "mean_scan_pose_age_s": mean("scan_pose_age_s"),
        "max_scan_pose_age_s": maximum("scan_pose_age_s"),
        "mean_observation_count": mean("observation_count"),
        "max_observation_count": maximum("observation_count"),
        "mean_candidate_position_std_m": mean("candidate_position_std_m"),
        "max_candidate_position_std_m": maximum("candidate_position_std_m"),
        "mean_update_latency_s": mean("update_latency_s"),
        "max_update_latency_s": maximum("update_latency_s"),
        "mean_belief_update_latency_s": (
            sum(finite_values(belief_rows, "update_latency_s"))
            / len(finite_values(belief_rows, "update_latency_s"))
            if finite_values(belief_rows, "update_latency_s")
            else None
        ),
        "max_belief_update_latency_s": (
            max(finite_values(belief_rows, "update_latency_s"))
            if finite_values(belief_rows, "update_latency_s")
            else None
        ),
        "mean_effective_sample_size": mean("effective_sample_size"),
        "min_effective_sample_size": minimum("effective_sample_size"),
        "mean_normalized_entropy": mean("normalized_entropy"),
        "min_normalized_entropy": minimum("normalized_entropy"),
        "mean_valid_beam_count": mean("valid_beam_count"),
        "max_correction_m": maximum("correction_m"),
        # A positive gain means that the LiDAR candidate has a higher
        # likelihood than the continuously propagated state estimate.  These
        # fields are intentionally diagnostic: a compact particle cloud can
        # still represent a wrong map alias, so this comparison must be read
        # together with correction size and temporal consistency.
        "mean_log_likelihood_gain": mean("log_likelihood_gain"),
        "accepted_mean_log_likelihood_gain": accepted_mean("log_likelihood_gain"),
        "accepted_min_log_likelihood_gain": (
            min(finite_values(valid, "log_likelihood_gain"))
            if finite_values(valid, "log_likelihood_gain")
            else None
        ),
        "accepted_max_log_likelihood_gain": (
            max(finite_values(valid, "log_likelihood_gain"))
            if finite_values(valid, "log_likelihood_gain")
            else None
        ),
        # The Bayesian terms are diagnostic-only in the current adapter.  A
        # large Mahalanobis value means the candidate moved beyond the
        # uncertainty carried by the propagated state; a negative Bayesian
        # gain means that the LiDAR likelihood improvement does not compensate
        # for that physically implausible displacement.
        "mean_correction_mahalanobis_sq": mean("correction_mahalanobis_sq"),
        "accepted_mean_correction_mahalanobis_sq": accepted_mean(
            "correction_mahalanobis_sq"
        ),
        "accepted_max_correction_mahalanobis_sq": (
            max(finite_values(valid, "correction_mahalanobis_sq"))
            if finite_values(valid, "correction_mahalanobis_sq")
            else None
        ),
        "mean_bayesian_log_gain": mean("bayesian_log_gain"),
        "accepted_mean_bayesian_log_gain": accepted_mean("bayesian_log_gain"),
        "accepted_min_bayesian_log_gain": (
            min(finite_values(valid, "bayesian_log_gain"))
            if finite_values(valid, "bayesian_log_gain")
            else None
        ),
        "accepted_max_bayesian_log_gain": (
            max(finite_values(valid, "bayesian_log_gain"))
            if finite_values(valid, "bayesian_log_gain")
            else None
        ),
        "resample_count": sum(bool(row.get("resampled")) for row in rows),
        "config": json.dumps(config.get("config", {}), sort_keys=True),
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnostic", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = summarize(args.diagnostic)
    print("diagnostic_file:", result["diagnostic_file"])
    for key, value in result.items():
        if key != "diagnostic_file":
            print(f"{key}: {value}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(result))
            writer.writeheader()
            writer.writerow(result)
        print(f"Wrote MCL summary to {args.output}")


if __name__ == "__main__":
    main()
