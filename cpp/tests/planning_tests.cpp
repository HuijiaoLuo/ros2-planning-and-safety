#include "robotics_planning/planners.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace robotics_planning;

void test_all_planners_find_a_path() {
    const auto parsed = parse_ascii_map({
        "S...#..",
        "....#..",
        ".......",
        "..###..",
        "......G",
    });

    BFSPlanner bfs;
    DijkstraPlanner dijkstra;
    GreedyBestFirstPlanner greedy;
    AStarPlanner astar;
    DFSBacktrackingPlanner dfs;

    assert(bfs.plan(parsed.grid, parsed.start, parsed.goal).found);
    assert(dijkstra.plan(parsed.grid, parsed.start, parsed.goal).found);
    assert(greedy.plan(parsed.grid, parsed.start, parsed.goal).found);
    assert(astar.plan(parsed.grid, parsed.start, parsed.goal).found);
    assert(dfs.plan(parsed.grid, parsed.start, parsed.goal).found);
}

void test_unit_cost_optimality() {
    const auto parsed = parse_ascii_map({
        "S...#..",
        "....#..",
        ".......",
        "..###..",
        "......G",
    });

    const auto bfs = BFSPlanner().plan(parsed.grid, parsed.start, parsed.goal);
    const auto dijkstra = DijkstraPlanner().plan(parsed.grid, parsed.start, parsed.goal);
    const auto astar = AStarPlanner().plan(parsed.grid, parsed.start, parsed.goal);

    assert(bfs.cost == dijkstra.cost);
    assert(dijkstra.cost == astar.cost);
}

void test_weighted_dijkstra() {
    std::vector<double> costs(6, 1.0);
    costs[1] = 10.0;
    const GridMap grid(3, 2, std::vector<std::uint8_t>{}, costs);

    const auto result = DijkstraPlanner().plan(grid, 0, 2);
    const std::vector<Cell> expected_path = {0, 3, 4, 5, 2};

    assert(result.found);
    assert(result.path == expected_path);
    assert(std::abs(result.cost - 4.0) < 1e-12);
}

void test_unreachable_goal() {
    const auto parsed = parse_ascii_map({
        "S#G",
        "###",
        "...",
    });

    const auto result = AStarPlanner().plan(parsed.grid, parsed.start, parsed.goal);
    assert(!result.found);
    assert(result.cost < 0.0);
}

void test_grid_occupancy() {
    const auto parsed = parse_ascii_map({
        "S..",
        ".#.",
        "..G",
    });
    assert(parsed.grid.occupancy()[4] == 1U);
}

int main() {
    test_all_planners_find_a_path();
    test_unit_cost_optimality();
    test_weighted_dijkstra();
    test_unreachable_goal();
    test_grid_occupancy();
    std::cout << "All C++ planner tests passed.\n";
    return 0;
}
