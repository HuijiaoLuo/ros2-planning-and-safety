"""Classical graph-search planners for occupancy grids."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
from collections import deque
from math import hypot
from typing import Callable

from .grid import Cell, GridMap

Heuristic = Callable[[Cell, Cell], float]


@dataclass(frozen=True)
class SearchResult:
    """Planner output and metrics for one search run."""

    algorithm: str
    path: tuple[Cell, ...] | None
    expanded: tuple[Cell, ...]
    cost: float | None

    @property
    def found(self) -> bool:
        return self.path is not None

    @property
    def path_length(self) -> int | None:
        return None if self.path is None else max(0, len(self.path) - 1)


def manhattan(a: Cell, b: Cell) -> float:
    return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))


def euclidean(a: Cell, b: Cell) -> float:
    return hypot(a[0] - b[0], a[1] - b[1])


def reconstruct_path(
    parent: dict[Cell, Cell], start: Cell, goal: Cell
) -> tuple[Cell, ...] | None:
    """Follow parent pointers from goal back to start, then reverse."""

    if start == goal:
        return (start,)
    if goal not in parent:
        return None

    reverse_path = [goal]
    current = goal
    while current != start:
        current = parent[current]
        reverse_path.append(current)
    reverse_path.reverse()
    return tuple(reverse_path)


def _validate_endpoints(grid: GridMap, start: Cell, goal: Cell) -> None:
    if not grid.is_free(start):
        raise ValueError(f"Start is not a free cell: {start}")
    if not grid.is_free(goal):
        raise ValueError(f"Goal is not a free cell: {goal}")


def _result(
    algorithm: str,
    parent: dict[Cell, Cell],
    start: Cell,
    goal: Cell,
    expanded: list[Cell],
    grid: GridMap,
) -> SearchResult:
    path = reconstruct_path(parent, start, goal)
    if path is None:
        return SearchResult(algorithm, None, tuple(expanded), None)
    cost = sum(grid.cost_to_enter(cell) for cell in path[1:])
    return SearchResult(algorithm, path, tuple(expanded), cost)


class BFSPlanner:
    name = "BFS"

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> SearchResult:
        _validate_endpoints(grid, start, goal)
        frontier = deque([start])
        discovered = {start}
        parent: dict[Cell, Cell] = {}
        expanded: list[Cell] = []

        while frontier:
            current = frontier.popleft()
            expanded.append(current)
            if current == goal:
                break
            for neighbor in grid.neighbors(current):
                if neighbor not in discovered:
                    discovered.add(neighbor)
                    parent[neighbor] = current
                    frontier.append(neighbor)

        return _result(self.name, parent, start, goal, expanded, grid)


class DijkstraPlanner:
    name = "Dijkstra"

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> SearchResult:
        _validate_endpoints(grid, start, goal)
        counter = 0
        frontier: list[tuple[float, int, Cell]] = [(0.0, counter, start)]
        distances = {start: 0.0}
        parent: dict[Cell, Cell] = {}
        expanded: list[Cell] = []

        while frontier:
            distance, _, current = heapq.heappop(frontier)
            if distance != distances.get(current):
                continue
            expanded.append(current)
            if current == goal:
                break
            for neighbor in grid.neighbors(current):
                new_distance = distance + grid.cost_to_enter(neighbor)
                if new_distance < distances.get(neighbor, float("inf")):
                    distances[neighbor] = new_distance
                    parent[neighbor] = current
                    counter += 1
                    heapq.heappush(frontier, (new_distance, counter, neighbor))

        return _result(self.name, parent, start, goal, expanded, grid)


class GreedyBestFirstPlanner:
    name = "Greedy"

    def __init__(self, heuristic: Heuristic = manhattan) -> None:
        self.heuristic = heuristic

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> SearchResult:
        _validate_endpoints(grid, start, goal)
        counter = 0
        frontier: list[tuple[float, int, Cell]] = [
            (self.heuristic(start, goal), counter, start)
        ]
        discovered = {start}
        parent: dict[Cell, Cell] = {}
        expanded: list[Cell] = []

        while frontier:
            _, _, current = heapq.heappop(frontier)
            expanded.append(current)
            if current == goal:
                break
            for neighbor in grid.neighbors(current):
                if neighbor not in discovered:
                    discovered.add(neighbor)
                    parent[neighbor] = current
                    counter += 1
                    priority = self.heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, counter, neighbor))

        return _result(self.name, parent, start, goal, expanded, grid)


class AStarPlanner:
    name = "A*"

    def __init__(
        self,
        heuristic: Heuristic = manhattan,
        prefer_goal_on_ties: bool = False,
    ) -> None:
        self.heuristic = heuristic
        self.prefer_goal_on_ties = prefer_goal_on_ties

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> SearchResult:
        _validate_endpoints(grid, start, goal)
        counter = 0
        start_heuristic = self.heuristic(start, goal)
        start_tie_break = start_heuristic if self.prefer_goal_on_ties else 0.0
        frontier: list[tuple[float, float, int, Cell, float]] = [
            (start_heuristic, start_tie_break, counter, start, 0.0)
        ]
        distances = {start: 0.0}
        parent: dict[Cell, Cell] = {}
        expanded: list[Cell] = []

        while frontier:
            _, _, _, current, queued_cost = heapq.heappop(frontier)
            if queued_cost != distances.get(current):
                continue
            current_cost = distances[current]
            expanded.append(current)
            if current == goal:
                break
            for neighbor in grid.neighbors(current):
                new_cost = current_cost + grid.cost_to_enter(neighbor)
                if new_cost < distances.get(neighbor, float("inf")):
                    distances[neighbor] = new_cost
                    parent[neighbor] = current
                    counter += 1
                    heuristic_value = self.heuristic(neighbor, goal)
                    priority = new_cost + heuristic_value
                    tie_break = (
                        heuristic_value if self.prefer_goal_on_ties else 0.0
                    )
                    heapq.heappush(
                        frontier,
                        (priority, tie_break, counter, neighbor, new_cost),
                    )

        return _result(self.name, parent, start, goal, expanded, grid)


class DFSBacktrackingPlanner:
    """Depth-first maze solver for comparison and educational purposes.

    The implementation uses an explicit stack rather than Python recursion so
    that scaling experiments can include larger grids without hitting the
    interpreter recursion limit.
    """

    name = "DFS backtracking"

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> SearchResult:
        _validate_endpoints(grid, start, goal)
        visited: set[Cell] = set()
        expanded: list[Cell] = []
        path: list[Cell] = []

        visited.add(start)
        expanded.append(start)
        path.append(start)
        stack: list[tuple[Cell, tuple[Cell, ...], int]] = [
            (start, grid.neighbors(start), 0)
        ]

        while stack:
            current, neighbors, next_index = stack[-1]
            if current == goal:
                break
            if next_index >= len(neighbors):
                stack.pop()
                path.pop()
                continue

            stack[-1] = (current, neighbors, next_index + 1)
            neighbor = neighbors[next_index]
            if neighbor in visited:
                continue

            visited.add(neighbor)
            expanded.append(neighbor)
            path.append(neighbor)
            stack.append((neighbor, grid.neighbors(neighbor), 0))

        if not stack or not path or path[-1] != goal:
            return SearchResult(self.name, None, tuple(expanded), None)
        cost = sum(grid.cost_to_enter(cell) for cell in path[1:])
        return SearchResult(self.name, tuple(path), tuple(expanded), cost)
