#include "robotics_planning/planners.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>

namespace robotics_planning {
namespace {

struct QueueEntry {
    double priority;
    std::size_t sequence;
    Cell cell;
    double g_cost;
};

struct MinQueue {
    bool operator()(const QueueEntry& left, const QueueEntry& right) const {
        if (left.priority != right.priority) {
            return left.priority > right.priority;
        }
        return left.sequence > right.sequence;
    }
};

void validate_endpoints(const GridMap& grid, Cell start, Cell goal) {
    if (!grid.is_free(start)) {
        throw std::invalid_argument("Start is not a free cell");
    }
    if (!grid.is_free(goal)) {
        throw std::invalid_argument("Goal is not a free cell");
    }
}

double path_cost(const GridMap& grid, const std::vector<Cell>& path) {
    double total = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
        total += grid.cost_to_enter(path[index]);
    }
    return total;
}

SearchResult make_result(
    const std::string& algorithm,
    const GridMap& grid,
    const std::vector<Cell>& parent,
    Cell start,
    Cell goal,
    std::vector<Cell> expanded
) {
    SearchResult result;
    result.algorithm = algorithm;
    result.expanded = std::move(expanded);
    result.path = reconstruct_path(parent, start, goal);
    result.found = !result.path.empty();
    if (result.found) {
        result.cost = path_cost(grid, result.path);
    }
    return result;
}

}  // namespace

double heuristic(
    const GridMap& grid,
    Cell current,
    Cell goal,
    HeuristicKind kind
) {
    const double dx = static_cast<double>(grid.x_of(current) - grid.x_of(goal));
    const double dy = static_cast<double>(grid.y_of(current) - grid.y_of(goal));
    if (kind == HeuristicKind::Euclidean) {
        return std::sqrt(dx * dx + dy * dy);
    }
    return std::abs(dx) + std::abs(dy);
}

std::vector<Cell> reconstruct_path(
    const std::vector<Cell>& parent,
    Cell start,
    Cell goal
) {
    if (start == goal) {
        return {start};
    }
    if (goal < 0 || goal >= static_cast<Cell>(parent.size()) || parent[goal] < 0) {
        return {};
    }

    std::vector<Cell> reverse_path;
    reverse_path.push_back(goal);
    Cell current = goal;
    while (current != start) {
        if (current < 0 || current >= static_cast<Cell>(parent.size())) {
            return {};
        }
        current = parent[current];
        if (current < 0) {
            return {};
        }
        reverse_path.push_back(current);
    }

    std::reverse(reverse_path.begin(), reverse_path.end());
    return reverse_path;
}

SearchResult BFSPlanner::plan(const GridMap& grid, Cell start, Cell goal) const {
    validate_endpoints(grid, start, goal);
    std::queue<Cell> frontier;
    std::vector<std::uint8_t> discovered(static_cast<std::size_t>(grid.cell_count()), 0U);
    std::vector<Cell> parent(static_cast<std::size_t>(grid.cell_count()), -1);
    std::vector<Cell> expanded;

    frontier.push(start);
    discovered[static_cast<std::size_t>(start)] = 1U;

    while (!frontier.empty()) {
        const Cell current = frontier.front();
        frontier.pop();
        expanded.push_back(current);
        if (current == goal) {
            break;
        }

        for (const Cell neighbor : grid.neighbors4(current)) {
            if (neighbor >= 0 && discovered[static_cast<std::size_t>(neighbor)] == 0U) {
                discovered[static_cast<std::size_t>(neighbor)] = 1U;
                parent[static_cast<std::size_t>(neighbor)] = current;
                frontier.push(neighbor);
            }
        }
    }

    return make_result("BFS", grid, parent, start, goal, std::move(expanded));
}

SearchResult DijkstraPlanner::plan(const GridMap& grid, Cell start, Cell goal) const {
    validate_endpoints(grid, start, goal);
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> distance(static_cast<std::size_t>(grid.cell_count()), infinity);
    std::vector<Cell> parent(static_cast<std::size_t>(grid.cell_count()), -1);
    std::vector<Cell> expanded;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, MinQueue> frontier;
    std::size_t sequence = 0;

    distance[static_cast<std::size_t>(start)] = 0.0;
    frontier.push({0.0, sequence++, start, 0.0});

    while (!frontier.empty()) {
        const QueueEntry entry = frontier.top();
        frontier.pop();
        if (entry.g_cost > distance[static_cast<std::size_t>(entry.cell)]) {
            continue;
        }

        expanded.push_back(entry.cell);
        if (entry.cell == goal) {
            break;
        }

        for (const Cell neighbor : grid.neighbors4(entry.cell)) {
            if (neighbor < 0) {
                continue;
            }
            const double new_cost = entry.g_cost + grid.cost_to_enter(neighbor);
            auto& known_cost = distance[static_cast<std::size_t>(neighbor)];
            if (new_cost < known_cost) {
                known_cost = new_cost;
                parent[static_cast<std::size_t>(neighbor)] = entry.cell;
                frontier.push({new_cost, sequence++, neighbor, new_cost});
            }
        }
    }

    return make_result("Dijkstra", grid, parent, start, goal, std::move(expanded));
}

SearchResult GreedyBestFirstPlanner::plan(
    const GridMap& grid,
    Cell start,
    Cell goal
) const {
    validate_endpoints(grid, start, goal);
    std::vector<std::uint8_t> discovered(static_cast<std::size_t>(grid.cell_count()), 0U);
    std::vector<Cell> parent(static_cast<std::size_t>(grid.cell_count()), -1);
    std::vector<Cell> expanded;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, MinQueue> frontier;
    std::size_t sequence = 0;

    discovered[static_cast<std::size_t>(start)] = 1U;
    frontier.push({heuristic(grid, start, goal, heuristic_kind_), sequence++, start, 0.0});

    while (!frontier.empty()) {
        const QueueEntry entry = frontier.top();
        frontier.pop();
        expanded.push_back(entry.cell);
        if (entry.cell == goal) {
            break;
        }

        for (const Cell neighbor : grid.neighbors4(entry.cell)) {
            if (neighbor >= 0 && discovered[static_cast<std::size_t>(neighbor)] == 0U) {
                discovered[static_cast<std::size_t>(neighbor)] = 1U;
                parent[static_cast<std::size_t>(neighbor)] = entry.cell;
                frontier.push({
                    heuristic(grid, neighbor, goal, heuristic_kind_),
                    sequence++,
                    neighbor,
                    0.0,
                });
            }
        }
    }

    return make_result("Greedy", grid, parent, start, goal, std::move(expanded));
}

SearchResult AStarPlanner::plan(const GridMap& grid, Cell start, Cell goal) const {
    validate_endpoints(grid, start, goal);
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> distance(static_cast<std::size_t>(grid.cell_count()), infinity);
    std::vector<Cell> parent(static_cast<std::size_t>(grid.cell_count()), -1);
    std::vector<Cell> expanded;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, MinQueue> frontier;
    std::size_t sequence = 0;

    distance[static_cast<std::size_t>(start)] = 0.0;
    frontier.push({
        heuristic(grid, start, goal, heuristic_kind_),
        sequence++,
        start,
        0.0,
    });

    while (!frontier.empty()) {
        const QueueEntry entry = frontier.top();
        frontier.pop();
        if (entry.g_cost > distance[static_cast<std::size_t>(entry.cell)]) {
            continue;
        }

        expanded.push_back(entry.cell);
        if (entry.cell == goal) {
            break;
        }

        for (const Cell neighbor : grid.neighbors4(entry.cell)) {
            if (neighbor < 0) {
                continue;
            }
            const double new_cost = entry.g_cost + grid.cost_to_enter(neighbor);
            auto& known_cost = distance[static_cast<std::size_t>(neighbor)];
            if (new_cost < known_cost) {
                known_cost = new_cost;
                parent[static_cast<std::size_t>(neighbor)] = entry.cell;
                frontier.push({
                    new_cost + heuristic(grid, neighbor, goal, heuristic_kind_),
                    sequence++,
                    neighbor,
                    new_cost,
                });
            }
        }
    }

    return make_result("A*", grid, parent, start, goal, std::move(expanded));
}

SearchResult DFSBacktrackingPlanner::plan(
    const GridMap& grid,
    Cell start,
    Cell goal
) const {
    validate_endpoints(grid, start, goal);
    std::vector<std::uint8_t> visited(static_cast<std::size_t>(grid.cell_count()), 0U);
    std::vector<Cell> parent(static_cast<std::size_t>(grid.cell_count()), -1);
    std::vector<Cell> expanded;

    std::function<bool(Cell)> visit = [&](Cell current) {
        visited[static_cast<std::size_t>(current)] = 1U;
        expanded.push_back(current);
        if (current == goal) {
            return true;
        }

        for (const Cell neighbor : grid.neighbors4(current)) {
            if (neighbor >= 0 && visited[static_cast<std::size_t>(neighbor)] == 0U) {
                parent[static_cast<std::size_t>(neighbor)] = current;
                if (visit(neighbor)) {
                    return true;
                }
            }
        }
        return false;
    };

    visit(start);
    return make_result(
        "DFS backtracking",
        grid,
        parent,
        start,
        goal,
        std::move(expanded)
    );
}

}  // namespace robotics_planning

