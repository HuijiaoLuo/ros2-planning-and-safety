"""Benchmark utilities for comparing grid-search planners."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable

from .grid import Cell, GridMap
from .planners import SearchResult


@dataclass(frozen=True)
class BenchmarkRecord:
    algorithm: str
    found: bool
    path_cost: float | None
    path_length: int | None
    expanded_nodes: int
    runtime_seconds: float
    result: SearchResult


def benchmark(
    planners: Iterable[object],
    grid: GridMap,
    start: Cell,
    goal: Cell,
    repetitions: int = 1,
) -> tuple[BenchmarkRecord, ...]:
    """Run each planner and return one record per planner.

    ``repetitions`` is useful for reducing timer noise on larger maps. The
    reported runtime is the mean over repetitions; the result is from the
    final run.

    Every planner receives the same map and endpoints. This makes
    ``expanded_nodes`` a comparable measure of search effort, while the
    returned path and cost describe solution quality.
    """

    if repetitions <= 0:
        raise ValueError("repetitions must be positive")

    records: list[BenchmarkRecord] = []
    for planner in planners:
        result: SearchResult | None = None
        elapsed_total = 0.0
        for _ in range(repetitions):
            started = time.perf_counter()
            result = planner.plan(grid, start, goal)
            elapsed_total += time.perf_counter() - started
        assert result is not None
        records.append(
            BenchmarkRecord(
                algorithm=result.algorithm,
                found=result.found,
                path_cost=result.cost,
                path_length=result.path_length,
                expanded_nodes=len(result.expanded),
                runtime_seconds=elapsed_total / repetitions,
                result=result,
            )
        )
    return tuple(records)


def format_records(records: Iterable[BenchmarkRecord]) -> str:
    """Format records as a compact plain-text table.

    Missing paths are shown as ``-`` rather than being converted to zero, so
    an unreachable case cannot be mistaken for a free or instantaneous one.
    """

    rows = [
        [
            record.algorithm,
            "yes" if record.found else "no",
            "-" if record.path_cost is None else f"{record.path_cost:.1f}",
            "-" if record.path_length is None else str(record.path_length),
            str(record.expanded_nodes),
            f"{record.runtime_seconds * 1e3:.3f}",
        ]
        for record in records
    ]
    headers = ["algorithm", "found", "cost", "steps", "expanded", "time_ms"]
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]
    lines = [
        "  ".join(header.ljust(widths[index]) for index, header in enumerate(headers)),
        "  ".join("-" * width for width in widths),
    ]
    lines.extend(
        "  ".join(value.ljust(widths[index]) for index, value in enumerate(row))
        for row in rows
    )
    return "\n".join(lines)

