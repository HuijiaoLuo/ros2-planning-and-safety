#pragma once

#include "robotics_planning/grid.hpp"

#include <string>
#include <vector>

namespace robotics_planning {

enum class HeuristicKind {
    Manhattan,
    Euclidean,
};

struct SearchResult {
    // ``expanded`` is retained for visualizations and search-effort
    // comparisons; ``path`` contains only the final start-to-goal chain.
    std::string algorithm;
    std::vector<Cell> path;
    std::vector<Cell> expanded;
    double cost = -1.0;
    bool found = false;

    int path_length() const noexcept {
        return found ? static_cast<int>(path.size()) - 1 : -1;
    }
};

double heuristic(
    const GridMap& grid,
    Cell current,
    Cell goal,
    HeuristicKind kind
);

std::vector<Cell> reconstruct_path(
    const std::vector<Cell>& parent,
    Cell start,
    Cell goal
);

class BFSPlanner {
public:
    // FIFO wavefront search. With unit edge costs this minimizes step count.
    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;
};

class DijkstraPlanner {
public:
    // Uniform-cost search. Unlike BFS, this honors per-cell entry costs.
    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;
};

class GreedyBestFirstPlanner {
public:
    explicit GreedyBestFirstPlanner(
        HeuristicKind heuristic_kind = HeuristicKind::Manhattan
    ) : heuristic_kind_(heuristic_kind) {}

    // Priority is h(n) only; this is a fast but non-optimal baseline.
    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;

private:
    HeuristicKind heuristic_kind_;
};

class AStarPlanner {
public:
    explicit AStarPlanner(
        HeuristicKind heuristic_kind = HeuristicKind::Manhattan
    ) : heuristic_kind_(heuristic_kind) {}

    // Priority is f(n) = g(n) + h(n); admissible h preserves optimality.
    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;

private:
    HeuristicKind heuristic_kind_;
};

class DFSBacktrackingPlanner {
public:
    // Recursive-style depth-first exploration with explicit parent links.
    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;
};

}  // namespace robotics_planning

