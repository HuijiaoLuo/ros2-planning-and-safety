#include "robotics_planning/benchmark.hpp"

#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using namespace robotics_planning;

int main() {
    const auto parsed = parse_ascii_map({
        "S...#.........",
        "....#.........",
        "....#..#####..",
        "....#.........",
        "....#####.....",
        "..............",
        "..#########...",
        "...........G..",
    });

    BFSPlanner bfs;
    DijkstraPlanner dijkstra;
    GreedyBestFirstPlanner greedy;
    AStarPlanner astar;
    DFSBacktrackingPlanner dfs;

    struct PlannerEntry {
        std::string name;
        PlannerFunction function;
    };

    const std::vector<PlannerEntry> planners = {
        {"DFS backtracking", [&dfs](const GridMap& grid, Cell start, Cell goal) {
             return dfs.plan(grid, start, goal);
         }},
        {"BFS", [&bfs](const GridMap& grid, Cell start, Cell goal) {
             return bfs.plan(grid, start, goal);
         }},
        {"Dijkstra", [&dijkstra](const GridMap& grid, Cell start, Cell goal) {
             return dijkstra.plan(grid, start, goal);
         }},
        {"Greedy", [&greedy](const GridMap& grid, Cell start, Cell goal) {
             return greedy.plan(grid, start, goal);
         }},
        {"A*", [&astar](const GridMap& grid, Cell start, Cell goal) {
             return astar.plan(grid, start, goal);
         }},
    };

    std::vector<BenchmarkRecord> records;
    for (const auto& planner : planners) {
        records.push_back(benchmark_planner(
            planner.name,
            planner.function,
            parsed.grid,
            parsed.start,
            parsed.goal,
            3
        ));

        const auto& record = records.back();
        std::cout << "\n--- " << record.algorithm << " ---\n";
        std::cout << parsed.grid.render(
            record.result.path,
            record.result.expanded,
            parsed.start,
            parsed.goal
        ) << "\n";
    }

    std::cout << "\nBenchmark\n";
    std::cout << format_benchmark_table(records);
    return 0;
}

