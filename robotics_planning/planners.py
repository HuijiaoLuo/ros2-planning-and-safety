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
    """Follow predecessor links from goal back to start, then reverse.

    Search algorithms store one predecessor per discovered cell rather than
    copying a full path into every queue entry. Reversing the recovered chain
    produces the start-to-goal path expected by the benchmark.
    """

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
    """Convert common search bookkeeping into the public result structure.

    The expanded sequence is kept for algorithm comparisons. Path cost is
    recomputed from the returned path so weighted cell-entry costs are
    included consistently for every planner.
    """
    path = reconstruct_path(parent, start, goal)
    if path is None:
        return SearchResult(algorithm, None, tuple(expanded), None)
    cost = sum(grid.cost_to_enter(cell) for cell in path[1:])
    return SearchResult(algorithm, path, tuple(expanded), cost)


class BFSPlanner:
    name = "BFS"

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> SearchResult:
        """Search in layers using a FIFO queue.

        With unit edge costs, the first visit to the goal has minimum step
        count. This baseline does not account for weighted cell costs.
        """
        _validate_endpoints(grid, start, goal)
        # ``frontier`` is the wavefront of cells whose neighbors have not yet
        # been inspected. ``discovered`` is marked at enqueue time, so the
        # same cell is never inserted twice through two equal-length routes.
        frontier = deque([start])
        discovered = {start}
        parent: dict[Cell, Cell] = {}
        expanded: list[Cell] = []

        while frontier:
            current = frontier.popleft()
            expanded.append(current)
            if current == goal:
                break
            # FIFO order visits all cells at distance d before distance d+1;
            # ``parent`` records the first shortest-step predecessor.
            for neighbor in grid.neighbors(current):
                if neighbor not in discovered:
                    discovered.add(neighbor)
                    parent[neighbor] = current
                    frontier.append(neighbor)

        return _result(self.name, parent, start, goal, expanded, grid)


class DijkstraPlanner:
    name = "Dijkstra"

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> SearchResult:
        """Expand the currently cheapest known cost-to-reach cell.

        Heap entries can become stale after a shorter route is found. The
        distance check discards those entries without requiring decrease-key.
        """
        _validate_endpoints(grid, start, goal)
        counter = 0
        # Heap entries are (known cost, insertion order, cell). The counter
        # makes equal-cost ordering deterministic without comparing cells.
        frontier: list[tuple[float, int, Cell]] = [(0.0, counter, start)]
        distances = {start: 0.0}
        parent: dict[Cell, Cell] = {}
        expanded: list[Cell] = []

        while frontier:
            distance, _, current = heapq.heappop(frontier)
            # A better route may have been found after this entry was pushed;
            # ignore the obsolete copy instead of expanding it again.
            if distance != distances.get(current):
                continue
            expanded.append(current)
            if current == goal:
                break
            for neighbor in grid.neighbors(current):
                # Relax the edge: replace the predecessor only when entering
                # this neighbor through the current cell is cheaper.
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
        """Prioritize cells that appear closest to the goal.

        Greedy search is a speed-oriented comparison, not an optimal planner:
        it ignores the cost already spent reaching the current cell.
        """
        _validate_endpoints(grid, start, goal)
        counter = 0
        # Greedy priority is h(n) only. ``discovered`` prevents cycles, but
        # unlike Dijkstra/A*, this search does not reopen a cell on a cheaper
        # route because path optimality is not its objective.
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
                    # Only the estimated remaining distance determines the
                    # queue order; the traveled cost is intentionally ignored.
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
        """Minimize ``cost_so_far + heuristic`` with optional tie-breaking.

        The heuristic changes queue order, while ``distances`` controls route
        replacement. With an admissible heuristic, A* preserves shortest-path
        cost; the optional goal tie-break only selects among equal priorities.
        """
        _validate_endpoints(grid, start, goal)
        counter = 0
        start_heuristic = self.heuristic(start, goal)
        start_tie_break = start_heuristic if self.prefer_goal_on_ties else 0.0
        # A* entries are (f, tie_break, insertion_order, cell, g), where
        # f=g+h is the estimated total path cost and g is retained to reject
        # stale heap entries. The extra fields only make ties reproducible.
        frontier: list[tuple[float, float, int, Cell, float]] = [
            (start_heuristic, start_tie_break, counter, start, 0.0)
        ]
        distances = {start: 0.0}
        parent: dict[Cell, Cell] = {}
        expanded: list[Cell] = []

        while frontier:
            _, _, _, current, queued_cost = heapq.heappop(frontier)
            # ``current`` may have an old, larger g value in the heap after a
            # shorter path updated ``distances``. It must not be expanded.
            if queued_cost != distances.get(current):
                continue
            current_cost = distances[current]
            expanded.append(current)
            if current == goal:
                break
            for neighbor in grid.neighbors(current):
                # Relax this edge using the true accumulated cost. The
                # heuristic is used only to prioritize the resulting entry.
                new_cost = current_cost + grid.cost_to_enter(neighbor)
                if new_cost < distances.get(neighbor, float("inf")):
                    distances[neighbor] = new_cost
                    parent[neighbor] = current
                    counter += 1
                    heuristic_value = self.heuristic(neighbor, goal)
                    priority = new_cost + heuristic_value
                    # With an admissible h, f preserves A* optimality. The
                    # optional tie-break prefers smaller h among equal f and
                    # therefore tends to expand cells closer to the goal.
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
        """Explore one branch at a time with an explicit backtracking stack.

        Each stack entry stores the next neighbor index to inspect. This is
        recursive DFS expressed iteratively, avoiding Python recursion limits
        in scaling experiments.
        """
        _validate_endpoints(grid, start, goal)
        visited: set[Cell] = set()
        expanded: list[Cell] = []
        path: list[Cell] = []

        visited.add(start)
        expanded.append(start)
        path.append(start)
        # Each stack tuple is (cell, fixed neighbor list, next index). Keeping
        # the index is the iterative equivalent of returning from a recursive
        # DFS call and continuing with the next sibling.
        stack: list[tuple[Cell, tuple[Cell, ...], int]] = [
            (start, grid.neighbors(start), 0)
        ]

        while stack:
            current, neighbors, next_index = stack[-1]
            if current == goal:
                break
            if next_index >= len(neighbors):
                # Every branch below this cell was exhausted: backtrack one
                # level and remove it from the current candidate path.
                stack.pop()
                path.pop()
                continue

            stack[-1] = (current, neighbors, next_index + 1)
            neighbor = neighbors[next_index]
            if neighbor in visited:
                continue

            # DFS marks on discovery so cycles cannot re-enter an old branch.
            visited.add(neighbor)
            expanded.append(neighbor)
            path.append(neighbor)
            stack.append((neighbor, grid.neighbors(neighbor), 0))

        if not stack or not path or path[-1] != goal:
            return SearchResult(self.name, None, tuple(expanded), None)
        cost = sum(grid.cost_to_enter(cell) for cell in path[1:])
        return SearchResult(self.name, tuple(path), tuple(expanded), cost)
