#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace robotics_planning {

// Cells use a flat row-major index: index = y * grid_width + x.
using Cell = int;

struct ParsedAsciiMap;

class GridMap {
public:
    GridMap(
        int width,
        int height,
        std::vector<std::uint8_t> occupied = {},
        std::vector<double> cell_costs = {}
    );

    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }
    int cell_count() const noexcept { return width_ * height_; }

    bool in_bounds(int x, int y) const noexcept;
    bool in_bounds(Cell cell) const noexcept;
    bool is_free(Cell cell) const noexcept;
    double cost_to_enter(Cell cell) const;

    // Flat row-major cell index: cell = y * width + x.
    int cell(int x, int y) const noexcept { return y * width_ + x; }
    int x_of(Cell cell) const noexcept { return cell % width_; }
    int y_of(Cell cell) const noexcept { return cell / width_; }

    // Right, down, left, up. Invalid or occupied neighbors are returned as -1
    // so planners can keep a fixed-size neighbor loop without exceptions.
    std::array<Cell, 4> neighbors4(Cell cell) const noexcept;

    const std::vector<std::uint8_t>& occupancy() const noexcept { return occupied_; }
    const std::vector<double>& cell_costs() const noexcept { return cell_costs_; }

    std::string render(
        const std::vector<Cell>& path,
        const std::vector<Cell>& expanded,
        Cell start,
        Cell goal
    ) const;

private:
    int width_;
    int height_;
    std::vector<std::uint8_t> occupied_;
    std::vector<double> cell_costs_;
};

struct ParsedAsciiMap {
    GridMap grid;
    Cell start;
    Cell goal;
};

ParsedAsciiMap parse_ascii_map(const std::vector<std::string>& lines);

}  // namespace robotics_planning

