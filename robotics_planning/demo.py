"""Run a small reproducible comparison of the planning algorithms."""

from .benchmark import benchmark, format_records
from .grid import parse_ascii_map
from .planners import (
    AStarPlanner,
    BFSPlanner,
    DFSBacktrackingPlanner,
    DijkstraPlanner,
    GreedyBestFirstPlanner,
)


MAP = """
S...#.........
....#.........
....#..#####..
....#.........
....#####.....
..............
..#########...
...........G..
"""


def main() -> None:
    """Run all baseline planners on one hand-written obstacle map.

    Keeping the map and endpoints fixed makes the printed path and expansion
    visualizations a small, reproducible sanity check before larger
    benchmarks are launched.
    """
    grid, start, goal = parse_ascii_map(MAP)
    planners = [
        DFSBacktrackingPlanner(),
        BFSPlanner(),
        DijkstraPlanner(),
        GreedyBestFirstPlanner(),
        AStarPlanner(),
    ]
    records = benchmark(planners, grid, start, goal, repetitions=3)

    print("Map legend: # obstacle, + explored, * final path")
    for record in records:
        print(f"\n--- {record.algorithm} ---")
        print(
            grid.render(
                path=record.result.path or (),
                explored=record.result.expanded,
                start=start,
                goal=goal,
            )
        )

    print("\nBenchmark")
    print(format_records(records))


if __name__ == "__main__":
    main()

