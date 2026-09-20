import unittest

from robotics_planning.benchmark import benchmark
from robotics_planning.grid import GridMap, parse_ascii_map
from robotics_planning.planners import (
    AStarPlanner,
    BFSPlanner,
    DFSBacktrackingPlanner,
    DijkstraPlanner,
    GreedyBestFirstPlanner,
    reconstruct_path,
)


class PlannerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.grid, self.start, self.goal = parse_ascii_map(
            """
            S...#..
            ....#..
            .......
            ..###..
            ......G
            """
        )

    def test_all_planners_find_a_path(self) -> None:
        planners = [
            DFSBacktrackingPlanner(),
            BFSPlanner(),
            DijkstraPlanner(),
            GreedyBestFirstPlanner(),
            AStarPlanner(),
        ]
        records = benchmark(planners, self.grid, self.start, self.goal)
        self.assertTrue(all(record.found for record in records))

    def test_unit_cost_planners_have_same_optimal_cost(self) -> None:
        bfs = BFSPlanner().plan(self.grid, self.start, self.goal)
        dijkstra = DijkstraPlanner().plan(self.grid, self.start, self.goal)
        astar = AStarPlanner().plan(self.grid, self.start, self.goal)
        self.assertEqual(bfs.cost, dijkstra.cost)
        self.assertEqual(dijkstra.cost, astar.cost)

    def test_astar_expands_no_more_than_dijkstra_on_this_map(self) -> None:
        dijkstra = DijkstraPlanner().plan(self.grid, self.start, self.goal)
        astar = AStarPlanner().plan(self.grid, self.start, self.goal)
        self.assertLessEqual(len(astar.expanded), len(dijkstra.expanded))

    def test_unreachable_goal_returns_no_path(self) -> None:
        grid, start, goal = parse_ascii_map(
            """
            S#G
            ###
            ...
            """
        )
        result = AStarPlanner().plan(grid, start, goal)
        self.assertFalse(result.found)
        self.assertIsNone(result.cost)

    def test_dijkstra_uses_weighted_costs(self) -> None:
        grid = GridMap(
            width=3,
            height=2,
            cell_costs={(1, 0): 10.0},
        )
        start = (0, 0)
        goal = (2, 0)
        result = DijkstraPlanner().plan(grid, start, goal)
        self.assertEqual(result.path, ((0, 0), (0, 1), (1, 1), (2, 1), (2, 0)))
        self.assertEqual(result.cost, 4.0)

    def test_reconstruct_path_handles_start_and_missing_goal(self) -> None:
        self.assertEqual(reconstruct_path({}, (0, 0), (0, 0)), ((0, 0),))
        self.assertIsNone(reconstruct_path({}, (0, 0), (1, 0)))

    def test_obstacle_inflation(self) -> None:
        grid = GridMap(width=3, height=3, obstacles=frozenset({(1, 1)}))
        inflated = grid.inflated(1)
        self.assertEqual(len(inflated.obstacles), 9)


if __name__ == "__main__":
    unittest.main()

