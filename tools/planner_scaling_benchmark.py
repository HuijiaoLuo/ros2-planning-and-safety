#!/usr/bin/env python3
"""Benchmark classical grid planners across map sizes and obstacle densities.

The benchmark keeps the map, start, goal, obstacle density, and random seed
identical for every planner in a case. Results are written as one CSV row per
planner and case so they can be plotted or aggregated later.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import random
from statistics import mean
import sys
from typing import Callable, Iterable

# Allow ``python tools/planner_scaling_benchmark.py`` to run directly from the
# repository root without requiring an editable package installation.
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from robotics_planning.benchmark import benchmark
from robotics_planning.grid import Cell, GridMap
from robotics_planning.planners import (
    AStarPlanner,
    BFSPlanner,
    DFSBacktrackingPlanner,
    DijkstraPlanner,
    GreedyBestFirstPlanner,
    euclidean,
    manhattan,
)


PlannerFactory = Callable[[], object]


@dataclass(frozen=True)
class PlannerSpec:
    label: str
    heuristic: str
    factory: PlannerFactory


def parse_csv_values(value: str, converter: Callable[[str], object]) -> tuple[object, ...]:
    values = tuple(converter(item.strip()) for item in value.split(",") if item.strip())
    if not values:
        raise argparse.ArgumentTypeError("Expected at least one comma-separated value")
    return values


def make_grid(size: int, density: float, seed: int) -> tuple[GridMap, Cell, Cell]:
    if size <= 1:
        raise ValueError("Grid size must be greater than one")
    if not 0.0 <= density < 1.0:
        raise ValueError("Obstacle density must satisfy 0 <= density < 1")

    start = (0, 0)
    goal = (size - 1, size - 1)
    rng = random.Random(seed)
    obstacles = frozenset(
        (x, y)
        for y in range(size)
        for x in range(size)
        if (x, y) not in (start, goal) and rng.random() < density
    )
    return GridMap(size, size, obstacles=obstacles), start, goal


def planner_specs() -> tuple[PlannerSpec, ...]:
    return (
        PlannerSpec("DFS backtracking", "none", DFSBacktrackingPlanner),
        PlannerSpec("BFS", "none", BFSPlanner),
        PlannerSpec("Dijkstra", "h=0", DijkstraPlanner),
        PlannerSpec(
            "Greedy (Manhattan)",
            "Manhattan",
            lambda: GreedyBestFirstPlanner(heuristic=manhattan),
        ),
        PlannerSpec(
            "A* (h=0)",
            "h=0",
            lambda: AStarPlanner(heuristic=lambda _a, _b: 0.0),
        ),
        PlannerSpec(
            "A* (Manhattan)",
            "Manhattan",
            lambda: AStarPlanner(heuristic=manhattan),
        ),
        PlannerSpec(
            "A* (Manhattan, tie-break)",
            "Manhattan + goal tie-break",
            lambda: AStarPlanner(
                heuristic=manhattan,
                prefer_goal_on_ties=True,
            ),
        ),
        PlannerSpec(
            "A* (Euclidean)",
            "Euclidean",
            lambda: AStarPlanner(heuristic=euclidean),
        ),
    )


def run_cases(
    sizes: Iterable[int],
    densities: Iterable[float],
    seed_count: int,
    base_seed: int,
    repetitions: int,
) -> list[dict[str, object]]:
    if seed_count <= 0:
        raise ValueError("seed_count must be positive")

    rows: list[dict[str, object]] = []
    specs = planner_specs()
    for size in sizes:
        for density in densities:
            for trial in range(seed_count):
                seed = base_seed + trial
                grid, start, goal = make_grid(size, density, seed)
                for spec in specs:
                    record = benchmark(
                        [spec.factory()],
                        grid,
                        start,
                        goal,
                        repetitions=repetitions,
                    )[0]
                    rows.append(
                        {
                            "grid_size": size,
                            "obstacle_density": density,
                            "seed": seed,
                            "algorithm": spec.label,
                            "heuristic": spec.heuristic,
                            "found": record.found,
                            "path_cost": record.path_cost,
                            "path_length": record.path_length,
                            "expanded_nodes": record.expanded_nodes,
                            "runtime_ms": record.runtime_seconds * 1e3,
                        }
                    )
    return rows


def write_csv(rows: list[dict[str, object]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "grid_size",
        "obstacle_density",
        "seed",
        "algorithm",
        "heuristic",
        "found",
        "path_cost",
        "path_length",
        "expanded_nodes",
        "runtime_ms",
    ]
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows: list[dict[str, object]]) -> None:
    groups: dict[tuple[int, float, str], list[dict[str, object]]] = {}
    for row in rows:
        key = (int(row["grid_size"]), float(row["obstacle_density"]), str(row["algorithm"]))
        groups.setdefault(key, []).append(row)

    print("grid  density  algorithm            success  mean_ms  mean_expanded  mean_path")
    print("-" * 82)
    for (size, density, algorithm), group in groups.items():
        successful = [row for row in group if row["found"]]
        mean_ms = mean(float(row["runtime_ms"]) for row in group)
        mean_expanded = mean(int(row["expanded_nodes"]) for row in group)
        path_values = [int(row["path_length"]) for row in successful]
        mean_path = "-" if not path_values else f"{mean(path_values):.1f}"
        print(
            f"{size:4d}  {density:7.2f}  {algorithm:20s} "
            f"{len(successful)}/{len(group):7d}  {mean_ms:7.3f}  "
            f"{mean_expanded:13.1f}  {mean_path:9s}"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sizes",
        type=lambda value: parse_csv_values(value, int),
        default=(20, 50, 100, 200),
        help="Comma-separated square grid sizes (default: 20,50,100,200)",
    )
    parser.add_argument(
        "--densities",
        type=lambda value: parse_csv_values(value, float),
        default=(0.0, 0.1, 0.2, 0.3),
        help="Comma-separated obstacle densities (default: 0,0.1,0.2,0.3)",
    )
    parser.add_argument("--seed-count", type=int, default=3)
    parser.add_argument("--base-seed", type=int, default=0)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/planner_scaling.csv"),
        help="CSV output path",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rows = run_cases(
        sizes=args.sizes,
        densities=args.densities,
        seed_count=args.seed_count,
        base_seed=args.base_seed,
        repetitions=args.repetitions,
    )
    write_csv(rows, args.output)
    print_summary(rows)
    print(f"\nWrote {len(rows)} records to {args.output}")


if __name__ == "__main__":
    main()
