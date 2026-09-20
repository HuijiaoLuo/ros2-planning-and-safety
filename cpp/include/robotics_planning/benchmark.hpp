#pragma once

#include "robotics_planning/planners.hpp"

#include <functional>
#include <string>
#include <vector>

namespace robotics_planning {

using PlannerFunction = std::function<SearchResult(const GridMap&, Cell, Cell)>;

struct BenchmarkRecord {
    std::string algorithm;
    bool found = false;
    double path_cost = -1.0;
    int path_length = -1;
    std::size_t expanded_nodes = 0;
    double runtime_ms = 0.0;
    SearchResult result;
};

BenchmarkRecord benchmark_planner(
    const std::string& name,
    const PlannerFunction& planner,
    const GridMap& grid,
    Cell start,
    Cell goal,
    int repetitions = 1
);

std::string format_benchmark_table(const std::vector<BenchmarkRecord>& records);

}  // namespace robotics_planning

