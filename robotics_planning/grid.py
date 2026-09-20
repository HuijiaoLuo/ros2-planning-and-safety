"""Occupancy-grid primitives used by the planning experiments."""

from __future__ import annotations

from dataclasses import dataclass, field
import textwrap
from typing import Iterable, Mapping

Cell = tuple[int, int]


@dataclass(frozen=True)
class GridMap:
    """A small 2-D occupancy grid.

    Coordinates are ``(x, y)``. ``x`` increases to the right and ``y``
    increases downward when the grid is rendered as ASCII text.

    ``cell_costs`` stores positive costs for entering selected free cells.
    Cells not present in the mapping have cost 1.0.
    """

    width: int
    height: int
    obstacles: frozenset[Cell] = field(default_factory=frozenset)
    cell_costs: Mapping[Cell, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Grid dimensions must be positive")
        for cell in self.obstacles:
            if not self.in_bounds(cell):
                raise ValueError(f"Obstacle outside grid: {cell}")
        for cell, cost in self.cell_costs.items():
            if not self.in_bounds(cell):
                raise ValueError(f"Cost cell outside grid: {cell}")
            if cost <= 0:
                raise ValueError("Cell costs must be positive")

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and cell not in self.obstacles

    def cost_to_enter(self, cell: Cell) -> float:
        if not self.is_free(cell):
            raise ValueError(f"Cannot enter occupied or out-of-bounds cell: {cell}")
        return float(self.cell_costs.get(cell, 1.0))

    def neighbors(self, cell: Cell, diagonal: bool = False) -> tuple[Cell, ...]:
        """Return free neighboring cells in deterministic order."""

        x, y = cell
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        if diagonal:
            directions += [(1, 1), (-1, 1), (-1, -1), (1, -1)]
        return tuple(
            candidate
            for dx, dy in directions
            if self.is_free(candidate := (x + dx, y + dy))
        )

    def inflated(self, radius_cells: int) -> "GridMap":
        """Return a copy with obstacles expanded by a square cell radius."""

        if radius_cells < 0:
            raise ValueError("Inflation radius cannot be negative")
        inflated_obstacles = set(self.obstacles)
        for obstacle in self.obstacles:
            ox, oy = obstacle
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    candidate = (ox + dx, oy + dy)
                    if self.in_bounds(candidate):
                        inflated_obstacles.add(candidate)
        return GridMap(
            width=self.width,
            height=self.height,
            obstacles=frozenset(inflated_obstacles),
            cell_costs=self.cell_costs,
        )

    def render(
        self,
        path: Iterable[Cell] = (),
        explored: Iterable[Cell] = (),
        start: Cell | None = None,
        goal: Cell | None = None,
    ) -> str:
        """Render the grid, explored cells, and final path as ASCII."""

        path_set = set(path)
        explored_set = set(explored)
        rows: list[str] = []
        for y in range(self.height):
            row: list[str] = []
            for x in range(self.width):
                cell = (x, y)
                if cell in self.obstacles:
                    symbol = "#"
                elif cell == start:
                    symbol = "S"
                elif cell == goal:
                    symbol = "G"
                elif cell in path_set:
                    symbol = "*"
                elif cell in explored_set:
                    symbol = "+"
                else:
                    symbol = "."
                row.append(symbol)
            rows.append("".join(row))
        return "\n".join(rows)


def parse_ascii_map(lines: str | Iterable[str]) -> tuple[GridMap, Cell, Cell]:
    """Parse an ASCII map and return ``(grid, start, goal)``."""

    raw_lines = (
        textwrap.dedent(lines).splitlines()
        if isinstance(lines, str)
        else list(lines)
    )
    cleaned = [line.rstrip("\n") for line in raw_lines if line.strip()]
    if not cleaned:
        raise ValueError("Map cannot be empty")

    width = len(cleaned[0])
    if width == 0 or any(len(line) != width for line in cleaned):
        raise ValueError("Map must be rectangular")

    obstacles: set[Cell] = set()
    start: Cell | None = None
    goal: Cell | None = None
    for y, line in enumerate(cleaned):
        for x, symbol in enumerate(line):
            cell = (x, y)
            if symbol == "#":
                obstacles.add(cell)
            elif symbol == "S":
                if start is not None:
                    raise ValueError("Map must contain exactly one start")
                start = cell
            elif symbol == "G":
                if goal is not None:
                    raise ValueError("Map must contain exactly one goal")
                goal = cell
            elif symbol != ".":
                raise ValueError(f"Unsupported map symbol: {symbol!r}")

    if start is None or goal is None:
        raise ValueError("Map must contain one start and one goal")
    return GridMap(width, len(cleaned), frozenset(obstacles)), start, goal
