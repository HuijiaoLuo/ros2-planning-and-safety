#!/usr/bin/env python3
"""A ROS2 adapter around the project's grid-based A* planning idea."""

from __future__ import annotations

import heapq
import math
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


Cell = tuple[int, int]


class GlobalPlanner(Node):
    """Plan a collision-free grid path from odometry to a fixed goal."""

    def __init__(self) -> None:
        super().__init__("global_planner")

        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 0.0)
        # Conservative circumscribed radius for the rectangular base and
        # caster. The base diagonal is about 0.31 m, so 0.35 m leaves margin
        # for grid discretization and tracking error.
        self.declare_parameter("robot_radius_m", 0.35)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("odom_topic", "/odom")

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.robot_radius_m = float(self.get_parameter("robot_radius_m").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(Path, "/plan", path_qos)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_odom: Optional[Odometry] = None
        self.last_reported_signature: Optional[tuple[Cell, Cell]] = None

    def map_callback(self, message: OccupancyGrid) -> None:
        self.latest_map = message
        self.try_plan()

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        self.try_plan()

    def try_plan(self) -> None:
        if self.latest_map is None or self.latest_odom is None:
            return

        grid = self.latest_map
        position = self.latest_odom.pose.pose.position
        start = self.world_to_cell(grid, position.x, position.y)
        goal = self.world_to_cell(grid, self.goal_x, self.goal_y)
        if start is None or goal is None:
            self.publish_empty_path(grid)
            return

        signature = (start, goal)
        if signature == self.last_reported_signature:
            return

        inflated = self.inflate_obstacles(grid)
        path = self.astar(grid, inflated, start, goal)
        if path is None:
            self.get_logger().warn(
                f"No path found from {start} to {goal}; publishing an empty path."
            )
            self.publish_empty_path(grid)
        else:
            self.get_logger().info(
                f"A* path found: {len(path) - 1} grid steps from {start} to {goal}."
            )
            self.publish_path(grid, path, (self.goal_x, self.goal_y))
        self.last_reported_signature = signature

    def world_to_cell(
        self,
        grid: OccupancyGrid,
        x: float,
        y: float,
    ) -> Optional[Cell]:
        if grid.info.resolution <= 0.0:
            return None
        # OccupancyGrid stores resolution as float32. For example, 0.1 may
        # arrive as 0.10000000149, so an exact boundary such as x=2.0 can
        # otherwise be mapped to the previous cell by floor().
        def stable_floor(value: float) -> int:
            nearest = round(value)
            if abs(value - nearest) < 1e-6:
                return int(nearest)
            return math.floor(value)

        column = stable_floor(
            (x - grid.info.origin.position.x) / grid.info.resolution
        )
        row = stable_floor(
            (y - grid.info.origin.position.y) / grid.info.resolution
        )
        if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
            return None
        return (column, row)

    def cell_is_occupied(self, grid: OccupancyGrid, cell: Cell) -> bool:
        column, row = cell
        value = grid.data[row * grid.info.width + column]
        return value < 0 or value >= self.occupied_threshold

    def inflate_obstacles(self, grid: OccupancyGrid) -> set[Cell]:
        radius_cells = math.ceil(self.robot_radius_m / grid.info.resolution)
        occupied: set[Cell] = set()

        for row in range(grid.info.height):
            for column in range(grid.info.width):
                cell = (column, row)
                if not self.cell_is_occupied(grid, cell):
                    continue
                for dx in range(-radius_cells, radius_cells + 1):
                    for dy in range(-radius_cells, radius_cells + 1):
                        inflated = (column + dx, row + dy)
                        if (
                            0 <= inflated[0] < grid.info.width
                            and 0 <= inflated[1] < grid.info.height
                        ):
                            occupied.add(inflated)
        return occupied

    def astar(
        self,
        grid: OccupancyGrid,
        occupied: set[Cell],
        start: Cell,
        goal: Cell,
    ) -> Optional[tuple[Cell, ...]]:
        if start in occupied or goal in occupied:
            return None

        frontier: list[tuple[float, int, Cell]] = []
        counter = 0
        heapq.heappush(frontier, (0.0, counter, start))
        parent: dict[Cell, Cell] = {}
        cost_so_far = {start: 0.0}

        while frontier:
            _, _, current = heapq.heappop(frontier)
            if current == goal:
                return self.reconstruct_path(parent, start, goal)

            for neighbor in self.neighbors(grid, occupied, current):
                new_cost = cost_so_far[current] + 1.0
                if new_cost >= cost_so_far.get(neighbor, float("inf")):
                    continue
                cost_so_far[neighbor] = new_cost
                parent[neighbor] = current
                counter += 1
                priority = new_cost + self.manhattan(neighbor, goal)
                heapq.heappush(frontier, (priority, counter, neighbor))

        return None

    @staticmethod
    def manhattan(a: Cell, b: Cell) -> float:
        return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))

    @staticmethod
    def neighbors(
        grid: OccupancyGrid,
        occupied: set[Cell],
        cell: Cell,
    ) -> tuple[Cell, ...]:
        column, row = cell
        candidates = (
            (column + 1, row),
            (column, row + 1),
            (column - 1, row),
            (column, row - 1),
        )
        return tuple(
            candidate
            for candidate in candidates
            if (
                0 <= candidate[0] < grid.info.width
                and 0 <= candidate[1] < grid.info.height
                and candidate not in occupied
            )
        )

    @staticmethod
    def reconstruct_path(
        parent: dict[Cell, Cell],
        start: Cell,
        goal: Cell,
    ) -> tuple[Cell, ...]:
        reverse_path = [goal]
        current = goal
        while current != start:
            current = parent[current]
            reverse_path.append(current)
        reverse_path.reverse()
        return tuple(reverse_path)

    def publish_path(
        self,
        grid: OccupancyGrid,
        cells: tuple[Cell, ...],
        exact_goal: Optional[tuple[float, float]] = None,
    ) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = grid.header.frame_id or "odom"
        for index, (column, row) in enumerate(cells):
            pose = PoseStamped()
            pose.header = message.header
            if exact_goal is not None and index == len(cells) - 1:
                pose.pose.position.x, pose.pose.position.y = exact_goal
            else:
                pose.pose.position.x = grid.info.origin.position.x + (
                    column + 0.5
                ) * grid.info.resolution
                pose.pose.position.y = grid.info.origin.position.y + (
                    row + 0.5
                ) * grid.info.resolution
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def publish_empty_path(self, grid: OccupancyGrid) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = grid.header.frame_id or "odom"
        self.path_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
