#!/usr/bin/env python3
"""Run a fixed known-map localization comparison matrix.

The script is intended for a ROS 2 Jazzy shell, such as the WSL2 environment
described in the README.  It keeps the controller, safety policy, and motion
prior fixed while varying only the scenario, localization backend, and seed.
Each run writes a complete set of evaluation and diagnostic artifacts under a
unique stem and appends its process result to ``matrix_manifest.csv``.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import subprocess
import sys
import time
from typing import Iterable


DEFAULT_SCENARIOS = ("baseline_obstacle", "l_corridor", "symmetric_corridor")
DEFAULT_BACKENDS = ("v4", "mcl", "icp")


@dataclass(frozen=True)
class RunSpec:
    """One deterministic row of the experiment matrix."""

    scenario: str
    backend: str
    seed: int

    @property
    def stem(self) -> str:
        return f"{self.scenario}__{self.backend}__seed_{self.seed:03d}"


def comma_values(raw: str) -> tuple[str, ...]:
    values = tuple(value.strip() for value in raw.split(",") if value.strip())
    if not values:
        raise ValueError("expected at least one comma-separated value")
    return values


def integer_values(raw: str) -> tuple[int, ...]:
    values = tuple(int(value) for value in comma_values(raw))
    if len(set(values)) != len(values):
        raise ValueError("seed values must be unique")
    return values


def build_specs(
    scenarios: Iterable[str],
    backends: Iterable[str],
    seeds: Iterable[int],
) -> list[RunSpec]:
    return [
        RunSpec(scenario=scenario, backend=backend, seed=seed)
        for scenario in scenarios
        for backend in backends
        for seed in seeds
    ]


def artifact_paths(output_dir: Path, spec: RunSpec) -> dict[str, Path]:
    stem = output_dir / spec.stem
    return {
        "evaluation": stem.with_name(stem.name + "_eval.csv"),
        "trace": stem.with_name(stem.name + "_trace.csv"),
        "plan": stem.with_name(stem.name + "_plan.csv"),
        "map": stem.with_name(stem.name + "_map.csv"),
        "metrics": stem.with_name(stem.name + "_metrics.csv"),
        "estimation_trace": stem.with_name(stem.name + "_estimation_trace.csv"),
        "diagnostic": stem.with_name(stem.name + "_diagnostic.jsonl"),
        "launch_log": stem.with_name(stem.name + "_launch.log"),
    }


def _fresh_file(path: Path, started_at: float | None) -> bool:
    """Return whether a non-empty file was produced by the current run."""
    try:
        stat = path.stat()
    except FileNotFoundError:
        return False
    if stat.st_size == 0:
        return False
    return started_at is None or stat.st_mtime >= started_at


def validate_run_artifacts(
    paths: dict[str, Path], *, started_at: float | None = None
) -> tuple[bool, str]:
    """Validate that ROS produced a real experiment, not only an exit code.

    A clean ROS launch shutdown is not sufficient: the evaluation logger must
    have received at least one sample, and the estimator must have produced a
    non-empty diagnostic row. ``started_at`` prevents stale artifacts from a
    previous attempt being counted as the current run.
    """
    for name in ("evaluation", "trace", "metrics", "estimation_trace", "diagnostic"):
        if not _fresh_file(paths[name], started_at):
            return False, f"missing_or_stale_{name}"

    try:
        with paths["evaluation"].open(newline="", encoding="utf-8") as handle:
            evaluation_rows = list(csv.DictReader(handle))
    except (OSError, csv.Error) as error:
        return False, f"invalid_evaluation_csv:{type(error).__name__}"
    if not evaluation_rows:
        return False, "evaluation_has_no_rows"
    try:
        evaluation_samples = int(float(evaluation_rows[0].get("samples", "0")))
    except (TypeError, ValueError):
        return False, "evaluation_samples_not_numeric"
    if evaluation_samples <= 0:
        return False, "evaluation_samples_zero"

    try:
        with paths["metrics"].open(newline="", encoding="utf-8") as handle:
            metric_rows = list(csv.DictReader(handle))
    except (OSError, csv.Error) as error:
        return False, f"invalid_metrics_csv:{type(error).__name__}"
    if not metric_rows:
        return False, "metrics_has_no_rows"
    try:
        metric_samples = int(float(metric_rows[0].get("samples", "0")))
    except (TypeError, ValueError):
        return False, "metrics_samples_not_numeric"
    if metric_samples <= 0:
        return False, "metrics_samples_zero"

    return True, "ok"


def build_command(
    spec: RunSpec,
    *,
    output_dir: Path,
    experiment_timeout_s: float,
    mcl_initialization_mode: str,
    fusion_mode: str | None = None,
    position_mode: str | None = None,
    gyro_bias_mode: str | None = None,
    wheel_yaw_noise_std_rad: float | None = None,
    localization_max_correction_m: float | None = None,
    localization_max_total_correction_m: float | None = None,
) -> list[str]:
    paths = artifact_paths(output_dir, spec)
    command = [
        "ros2",
        "launch",
        "robotics_sim",
        "sim.launch.py",
        f"scenario:={spec.scenario}",
        f"localization_backend:={spec.backend}",
        "navigation_pose_topic:=/localized_estimate",
        "control_pose_topic:=/state_prediction",
        "motion_prior_topic:=/state_prediction",
        f"mcl_random_seed:={spec.seed}",
        f"scan_noise_seed:={spec.seed}",
        f"mcl_initialization_mode:={mcl_initialization_mode}",
        # Keep a decimal point so ROS 2 declares this launch parameter as a
        # DOUBLE rather than an INTEGER (the evaluation logger requires a
        # floating-point timeout).
        f"experiment_timeout_s:={experiment_timeout_s:.6f}",
        f"evaluation_output:={paths['evaluation']}",
        f"trace_output:={paths['trace']}",
        f"plan_output:={paths['plan']}",
        f"map_output:={paths['map']}",
        f"estimation_output:={paths['metrics']}",
        f"estimation_trace_output:={paths['estimation_trace']}",
        f"localization_diagnostic_output:={paths['diagnostic']}",
    ]
    if localization_max_correction_m is not None:
        command.append(
            f"localization_max_correction_m:={localization_max_correction_m:.6f}"
        )
    if localization_max_total_correction_m is not None:
        command.append(
            "localization_max_total_correction_m:="
            f"{localization_max_total_correction_m:.6f}"
        )
    if fusion_mode is not None:
        command.append(f"fusion_mode:={fusion_mode}")
    if position_mode is not None:
        command.append(f"position_mode:={position_mode}")
    if gyro_bias_mode is not None:
        command.append(f"gyro_bias_mode:={gyro_bias_mode}")
    if wheel_yaw_noise_std_rad is not None:
        command.append(
            f"wheel_yaw_noise_std_rad:={wheel_yaw_noise_std_rad:.6f}"
        )
    return command


def write_manifest(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "scenario",
        "backend",
        "seed",
        "stem",
        "return_code",
        "completed",
        "failure_reason",
        "duration_s",
        "evaluation_file",
        "metrics_file",
        "diagnostic_file",
        "launch_log",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def load_manifest(path: Path) -> list[dict[str, object]]:
    """Load an existing manifest so a partial matrix can be resumed safely."""
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a fixed scenario/backend/seed localization matrix."
    )
    parser.add_argument(
        "--scenarios",
        default=",".join(DEFAULT_SCENARIOS),
        help="Comma-separated scenario profiles.",
    )
    parser.add_argument(
        "--backends",
        default=",".join(DEFAULT_BACKENDS),
        help="Comma-separated localization backends.",
    )
    parser.add_argument(
        "--seeds",
        default="0,1,2",
        help="Comma-separated integer seeds.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("results/localization_matrix"),
        help="Directory for per-run artifacts and the manifest.",
    )
    parser.add_argument(
        "--experiment-timeout-s",
        type=float,
        default=120.0,
        help="Per-run ROS evaluation timeout.",
    )
    parser.add_argument(
        "--mcl-initialization-mode",
        choices=("local", "global"),
        default="local",
        help="MCL prior mode used for the matrix.",
    )
    parser.add_argument(
        "--fusion-mode",
        choices=("fixed", "adaptive", "ekf"),
        default=None,
        help="Optional estimator fusion mode for a controlled comparison.",
    )
    parser.add_argument(
        "--position-mode",
        choices=("wheel_pose", "propagated"),
        default=None,
        help="Optional estimator position model for a controlled comparison.",
    )
    parser.add_argument(
        "--gyro-bias-mode",
        choices=("estimated", "fixed"),
        default=None,
        help="Optional pose-EKF gyro-bias mode for a controlled comparison.",
    )
    parser.add_argument(
        "--wheel-yaw-noise-std-rad",
        type=float,
        default=None,
        help=(
            "Optional wheel-yaw measurement noise for adaptive fusion or the "
            "pose EKF."
        ),
    )
    parser.add_argument(
        "--localization-max-correction-m",
        type=float,
        default=None,
        help=(
            "Optional localization displacement gate override for a controlled "
            "A/B experiment."
        ),
    )
    parser.add_argument(
        "--localization-max-total-correction-m",
        type=float,
        default=None,
        help=(
            "Optional accumulated localization displacement gate override for "
            "a controlled A/B experiment."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands without starting ROS 2.",
    )
    parser.add_argument(
        "--stop-on-error",
        action="store_true",
        help="Stop after the first ROS error or incomplete artifact set.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        scenarios = comma_values(args.scenarios)
        backends = comma_values(args.backends)
        seeds = integer_values(args.seeds)
        specs = build_specs(scenarios, backends, seeds)
    except ValueError as error:
        print(f"Invalid matrix configuration: {error}", file=sys.stderr)
        return 2

    if args.experiment_timeout_s <= 0.0:
        print("--experiment-timeout-s must be positive", file=sys.stderr)
        return 2
    for option_name in (
        "localization_max_correction_m",
        "localization_max_total_correction_m",
    ):
        value = getattr(args, option_name)
        if value is not None and value <= 0.0:
            print(f"--{option_name.replace('_', '-')} must be positive", file=sys.stderr)
            return 2
    if (
        args.wheel_yaw_noise_std_rad is not None
        and args.wheel_yaw_noise_std_rad < 0.0
    ):
        print("--wheel-yaw-noise-std-rad must be nonnegative", file=sys.stderr)
        return 2

    if not args.dry_run:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output_dir / "matrix_manifest.csv"
    manifest_rows: list[dict[str, object]] = (
        [] if args.dry_run else load_manifest(manifest_path)
    )
    manifest_indices = {
        str(row.get("stem", "")): index
        for index, row in enumerate(manifest_rows)
        if row.get("stem")
    }

    print(f"Prepared {len(specs)} runs in {args.output_dir}")
    for index, spec in enumerate(specs, start=1):
        command = build_command(
            spec,
            output_dir=args.output_dir,
            experiment_timeout_s=args.experiment_timeout_s,
            mcl_initialization_mode=args.mcl_initialization_mode,
            fusion_mode=args.fusion_mode,
            position_mode=args.position_mode,
            gyro_bias_mode=args.gyro_bias_mode,
            wheel_yaw_noise_std_rad=args.wheel_yaw_noise_std_rad,
            localization_max_correction_m=args.localization_max_correction_m,
            localization_max_total_correction_m=(
                args.localization_max_total_correction_m
            ),
        )
        print(f"[{index}/{len(specs)}] {spec.stem}")
        print(" ".join(command))
        paths = artifact_paths(args.output_dir, spec)

        if args.dry_run:
            return_code = 0
            completed = True
            failure_reason = "dry_run"
            duration_s = 0.0
        else:
            started_at = time.time()
            started_clock = time.perf_counter()
            with paths["launch_log"].open("w", encoding="utf-8") as log_handle:
                process = subprocess.run(
                    command,
                    check=False,
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
            return_code = int(process.returncode)
            duration_s = time.perf_counter() - started_clock
            artifacts_ok, failure_reason = validate_run_artifacts(
                paths, started_at=started_at
            )
            completed = return_code == 0 and artifacts_ok
            if not completed:
                if return_code != 0:
                    failure_reason = f"ros_exit_{return_code};{failure_reason}"
                print(
                    f"Run incomplete: {failure_reason}. "
                    f"See {paths['launch_log']}",
                    file=sys.stderr,
                )

        manifest_row = {
            "scenario": spec.scenario,
            "backend": spec.backend,
            "seed": spec.seed,
            "stem": spec.stem,
            "return_code": return_code,
            "completed": completed,
            "failure_reason": failure_reason,
            "duration_s": f"{duration_s:.3f}",
            "evaluation_file": str(paths["evaluation"]),
            "metrics_file": str(paths["metrics"]),
            "diagnostic_file": str(paths["diagnostic"]),
            "launch_log": str(paths["launch_log"]),
        }
        existing_index = manifest_indices.get(spec.stem)
        if existing_index is None:
            manifest_indices[spec.stem] = len(manifest_rows)
            manifest_rows.append(manifest_row)
        else:
            manifest_rows[existing_index] = manifest_row
        if not args.dry_run:
            write_manifest(manifest_path, manifest_rows)

        if not args.dry_run and not completed and args.stop_on_error:
            print(f"Stopping after incomplete run {spec.stem}.", file=sys.stderr)
            return return_code if return_code != 0 else 1

    if args.dry_run:
        print("Dry run complete; no ROS 2 processes or artifacts were created.")
        return 0

    print(f"Wrote matrix manifest to {manifest_path}")
    return 0 if all(row["completed"] for row in manifest_rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
