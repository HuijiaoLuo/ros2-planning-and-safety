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
    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;
};

class DijkstraPlanner {
public:
    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;
};

class GreedyBestFirstPlanner {
public:
    explicit GreedyBestFirstPlanner(
        HeuristicKind heuristic_kind = HeuristicKind::Manhattan
    ) : heuristic_kind_(heuristic_kind) {}

    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;

private:
    HeuristicKind heuristic_kind_;
};

class AStarPlanner {
public:
    explicit AStarPlanner(
        HeuristicKind heuristic_kind = HeuristicKind::Manhattan
    ) : heuristic_kind_(heuristic_kind) {}

    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;

private:
    HeuristicKind heuristic_kind_;
};

class DFSBacktrackingPlanner {
public:
    SearchResult plan(const GridMap& grid, Cell start, Cell goal) const;
};

}  // namespace robotics_planning

