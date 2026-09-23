#!/usr/bin/env python3
"""Summarize point-to-point ICP diagnostics without hiding failed matches."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


def finite_values(rows: list[dict[str, object]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return values


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def correction_magnitudes(rows: list[dict[str, object]], prefix: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        x = row.get(f"{prefix}dx_m")
        y = row.get(f"{prefix}dy_m")
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            value = math.hypot(float(x), float(y))
            if math.isfinite(value):
                values.append(value)
    return values


def summarize(path: Path) -> dict[str, object]:
    records = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    rows = [row for row in records if row.get("record_type") == "icp_result"]
    statuses = Counter(str(row.get("status", "unknown")) for row in rows)
    accepted = [row for row in rows if row.get("measurement_applied") is True]
    result = {
        "diagnostic_file": str(path),
        "result_count": len(rows),
        "accepted_count": len(accepted),
        "acceptance_rate": len(accepted) / len(rows) if rows else 0.0,
        "status_counts": dict(sorted(statuses.items())),
        "mean_residual_m": mean(finite_values(rows, "mean_residual_m")),
        "mean_rms_residual_m": mean(finite_values(rows, "rms_residual_m")),
        "mean_correspondences": mean(finite_values(rows, "correspondence_count")),
        "mean_iterations": mean(finite_values(rows, "iterations")),
        "mean_candidate_correction_m": mean(
            finite_values(rows, "candidate_correction_m")
        ),
        "mean_applied_correction_m": mean(
            correction_magnitudes(rows, "applied_")
        ),
        "mean_match_age_s": mean(finite_values(rows, "match_age_s")),
        "mean_worker_compute_time_s": mean(
            finite_values(rows, "worker_compute_time_s")
        ),
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnostic", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.diagnostic)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        key for key, value in result.items() if not isinstance(value, dict)
    ]
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow({key: result[key] for key in fields})
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"Wrote ICP summary to {args.output}")


if __name__ == "__main__":
    main()
