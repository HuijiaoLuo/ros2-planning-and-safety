#!/usr/bin/env python3
"""Summarize fixed localization-matrix evaluation CSV files."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path
from statistics import mean
from typing import Iterable


def bool_value(row: dict[str, str], name: str) -> bool:
    return row.get(name, "").strip().lower() == "true"


def optional_float(row: dict[str, str], name: str) -> float | None:
    value = row.get(name, "").strip()
    return None if not value else float(value)


def values(rows: Iterable[dict[str, str]], field: str) -> list[float]:
    result: list[float] = []
    for row in rows:
        value = optional_float(row, field)
        if value is not None:
            result.append(value)
    return result


def summarize(rows: Iterable[dict[str, str]]) -> list[dict[str, object]]:
    groups: dict[tuple[str, str], list[dict[str, str]]] = {}
    for row in rows:
        key = (
            row.get("scenario", "unknown").strip() or "unknown",
            row.get("localization_backend", "unknown").strip() or "unknown",
        )
        groups.setdefault(key, []).append(row)

    summaries: list[dict[str, object]] = []
    for (scenario, backend), group in sorted(groups.items()):
        termination = Counter(
            row.get("termination_reason", "unknown").strip() or "unknown"
            for row in group
        )
        collision_values = [
            row.get("collision", "").strip().lower()
            for row in group
            if row.get("collision", "").strip().lower() in {"true", "false"}
        ]
        seeds = sorted(
            {
                row.get("configured_scan_noise_seed", "").strip()
                for row in group
                if row.get("configured_scan_noise_seed", "").strip()
            },
            key=lambda value: int(float(value)),
        )
        final_errors = values(group, "ground_truth_final_error_m")
        navigation_errors = values(group, "navigation_pose_final_error_m")
        state_errors = values(group, "state_estimate_final_error_m")
        time_to_goal = values(group, "time_to_goal_s")
        override_ratios = values(group, "safety_override_ratio")

        summaries.append(
            {
                "scenario": scenario,
                "localization_backend": backend,
                "runs": len(group),
                "seeds": ",".join(seeds),
                "successes": sum(bool_value(row, "success") for row in group),
                "success_rate": sum(
                    bool_value(row, "success") for row in group
                )
                / len(group),
                "goal_reached_count": termination["goal_reached"],
                "experiment_timeout_count": termination["experiment_timeout"],
                "mean_final_error_m": mean(final_errors) if final_errors else None,
                "mean_navigation_final_error_m": (
                    mean(navigation_errors) if navigation_errors else None
                ),
                "mean_state_estimate_final_error_m": (
                    mean(state_errors) if state_errors else None
                ),
                "mean_time_to_goal_s": mean(time_to_goal) if time_to_goal else None,
                "collision_count": sum(value == "true" for value in collision_values),
                "collision_rate_known": (
                    None
                    if not collision_values
                    else sum(value == "true" for value in collision_values)
                    / len(collision_values)
                ),
                "mean_safety_override_ratio": (
                    mean(override_ratios) if override_ratios else None
                ),
                "termination_reasons": ";".join(
                    f"{name}={count}" for name, count in sorted(termination.items())
                ),
            }
        )
    return summaries


def load_rows(paths: Iterable[Path]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            rows.extend(csv.DictReader(handle))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--glob",
        default="results/localization_matrix/*_eval.csv",
        help="Glob selecting one-row evaluation CSV files.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/localization_matrix_summary.csv"),
    )
    args = parser.parse_args()

    paths = sorted(Path().glob(args.glob))
    if not paths:
        raise SystemExit(f"No evaluation files matched: {args.glob}")

    summaries = summarize(load_rows(paths))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fields = list(summaries[0]) if summaries else []
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)

    print(f"Summarized {len(paths)} evaluation files into {len(summaries)} groups.")
    print(f"Wrote localization matrix summary to {args.output}")


if __name__ == "__main__":
    main()
