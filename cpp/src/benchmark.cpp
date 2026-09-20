#include "robotics_planning/benchmark.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace robotics_planning {

BenchmarkRecord benchmark_planner(
    const std::string& name,
    const PlannerFunction& planner,
    const GridMap& grid,
    Cell start,
    Cell goal,
    int repetitions
) {
    if (repetitions <= 0) {
        throw std::invalid_argument("Benchmark repetitions must be positive");
    }

    SearchResult result;
    double total_ms = 0.0;
    for (int repetition = 0; repetition < repetitions; ++repetition) {
        const auto begin = std::chrono::steady_clock::now();
        result = planner(grid, start, goal);
        const auto end = std::chrono::steady_clock::now();
        total_ms += std::chrono::duration<double, std::milli>(end - begin).count();
    }

    BenchmarkRecord record;
    record.algorithm = name;
    record.found = result.found;
    record.path_cost = result.cost;
    record.path_length = result.path_length();
    record.expanded_nodes = result.expanded.size();
    record.runtime_ms = total_ms / static_cast<double>(repetitions);
    record.result = std::move(result);
    return record;
}

std::string format_benchmark_table(const std::vector<BenchmarkRecord>& records) {
    std::ostringstream output;
    output << std::left
           << std::setw(20) << "algorithm"
           << std::setw(8) << "found"
           << std::setw(10) << "cost"
           << std::setw(10) << "steps"
           << std::setw(12) << "expanded"
           << "time_ms\n";
    output << std::string(70, '-') << '\n';

    output << std::fixed << std::setprecision(3);
    for (const auto& record : records) {
        output << std::left
               << std::setw(20) << record.algorithm
               << std::setw(8) << (record.found ? "yes" : "no");
        if (record.found) {
            output << std::setw(10) << std::setprecision(1) << record.path_cost
                   << std::setw(10) << record.path_length;
        } else {
            output << std::setw(10) << "-"
                   << std::setw(10) << "-";
        }
        output << std::setprecision(0) << std::setw(12) << record.expanded_nodes
               << std::setprecision(3) << record.runtime_ms << '\n';
    }
    return output.str();
}

}  // namespace robotics_planning
