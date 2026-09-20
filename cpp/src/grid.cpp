#include "robotics_planning/grid.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace robotics_planning {

GridMap::GridMap(
    int width,
    int height,
    std::vector<std::uint8_t> occupied,
    std::vector<double> cell_costs
) : width_(width), height_(height) {
    if (width_ <= 0 || height_ <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive");
    }

    const auto expected_size = static_cast<std::size_t>(width_ * height_);
    if (occupied.empty()) {
        occupied.assign(expected_size, 0U);
    } else if (occupied.size() != expected_size) {
        throw std::invalid_argument("Occupancy array has the wrong size");
    }

    if (cell_costs.empty()) {
        cell_costs.assign(expected_size, 1.0);
    } else if (cell_costs.size() != expected_size) {
        throw std::invalid_argument("Cell-cost array has the wrong size");
    }

    for (const double cost : cell_costs) {
        if (cost <= 0.0 || !std::isfinite(cost)) {
            throw std::invalid_argument("Cell costs must be finite and positive");
        }
    }

    occupied_ = std::move(occupied);
    cell_costs_ = std::move(cell_costs);
}

bool GridMap::in_bounds(int x, int y) const noexcept {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool GridMap::in_bounds(Cell cell) const noexcept {
    return cell >= 0 && cell < cell_count();
}

bool GridMap::is_free(Cell cell) const noexcept {
    return in_bounds(cell) && occupied_[static_cast<std::size_t>(cell)] == 0U;
}

double GridMap::cost_to_enter(Cell cell) const {
    if (!is_free(cell)) {
        throw std::invalid_argument("Cannot enter an occupied or invalid cell");
    }
    return cell_costs_[static_cast<std::size_t>(cell)];
}

std::array<Cell, 4> GridMap::neighbors4(Cell current) const noexcept {
    if (!in_bounds(current)) {
        return {-1, -1, -1, -1};
    }

    const int x = x_of(current);
    const int y = y_of(current);
    const std::array<std::pair<int, int>, 4> directions = {
        std::pair<int, int>{1, 0},
        std::pair<int, int>{0, 1},
        std::pair<int, int>{-1, 0},
        std::pair<int, int>{0, -1},
    };

    std::array<Cell, 4> result{};
    for (std::size_t index = 0; index < directions.size(); ++index) {
        const int neighbor_x = x + directions[index].first;
        const int neighbor_y = y + directions[index].second;
        const Cell candidate = in_bounds(neighbor_x, neighbor_y)
            ? cell(neighbor_x, neighbor_y)
            : -1;
        result[index] = candidate >= 0 && is_free(candidate) ? candidate : -1;
    }
    return result;
}

std::string GridMap::render(
    const std::vector<Cell>& path,
    const std::vector<Cell>& expanded,
    Cell start,
    Cell goal
) const {
    std::vector<char> symbols(static_cast<std::size_t>(cell_count()), '.');
    for (Cell cell_index = 0; cell_index < cell_count(); ++cell_index) {
        if (!is_free(cell_index)) {
            symbols[static_cast<std::size_t>(cell_index)] = '#';
        }
    }

    for (const Cell cell_index : expanded) {
        if (is_free(cell_index)) {
            symbols[static_cast<std::size_t>(cell_index)] = '+';
        }
    }
    for (const Cell cell_index : path) {
        if (is_free(cell_index)) {
            symbols[static_cast<std::size_t>(cell_index)] = '*';
        }
    }
    if (is_free(start)) {
        symbols[static_cast<std::size_t>(start)] = 'S';
    }
    if (is_free(goal)) {
        symbols[static_cast<std::size_t>(goal)] = 'G';
    }

    std::ostringstream output;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            output << symbols[static_cast<std::size_t>(cell(x, y))];
        }
        if (y + 1 < height_) {
            output << '\n';
        }
    }
    return output.str();
}

ParsedAsciiMap parse_ascii_map(const std::vector<std::string>& lines) {
    if (lines.empty()) {
        throw std::invalid_argument("Map cannot be empty");
    }

    const int height = static_cast<int>(lines.size());
    const int width = static_cast<int>(lines.front().size());
    if (width == 0) {
        throw std::invalid_argument("Map rows cannot be empty");
    }

    std::vector<std::uint8_t> occupied(
        static_cast<std::size_t>(width * height), 0U
    );
    Cell start = -1;
    Cell goal = -1;

    for (int y = 0; y < height; ++y) {
        if (static_cast<int>(lines[static_cast<std::size_t>(y)].size()) != width) {
            throw std::invalid_argument("Map must be rectangular");
        }

        for (int x = 0; x < width; ++x) {
            const char symbol = lines[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
            const Cell current = y * width + x;
            if (symbol == '#') {
                occupied[static_cast<std::size_t>(current)] = 1U;
            } else if (symbol == 'S') {
                if (start != -1) {
                    throw std::invalid_argument("Map must contain one start");
                }
                start = current;
            } else if (symbol == 'G') {
                if (goal != -1) {
                    throw std::invalid_argument("Map must contain one goal");
                }
                goal = current;
            } else if (symbol != '.') {
                throw std::invalid_argument("Unsupported map symbol");
            }
        }
    }

    if (start == -1 || goal == -1) {
        throw std::invalid_argument("Map must contain one start and one goal");
    }

    return {GridMap(width, height, std::move(occupied)), start, goal};
}

}  // namespace robotics_planning
