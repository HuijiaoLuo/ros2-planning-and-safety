#!/usr/bin/env python3
"""Publish a small deterministic occupancy grid for the first ROS2 experiment."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class StaticMapPublisher(Node):
    """Publish a map aligned with the obstacle in the Gazebo world."""

    def __init__(self) -> None:
        super().__init__("static_map_publisher")

        self.declare_parameter("width", 60)
        self.declare_parameter("height", 60)
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("origin_x", -1.0)
        self.declare_parameter("origin_y", -3.0)
        self.declare_parameter("publish_rate_hz", 1.0)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.message = self.build_map()
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)
        self.publish_map()

    def build_map(self) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.frame_id = "odom"
        message.info.resolution = self.resolution
        message.info.width = self.width
        message.info.height = self.height
        message.info.origin.position.x = self.origin_x
        message.info.origin.position.y = self.origin_y
        message.info.origin.orientation.w = 1.0

        data = [0] * (self.width * self.height)

        # Match the obstacle box in robotics_sim/worlds/differential_drive.sdf.
        self.mark_rectangle(
            data,
            x_min=0.85,
            x_max=1.15,
            y_min=-0.50,
            y_max=0.50,
            value=100,
        )

        # Keep the outer boundary occupied so future planners cannot leave the map.
        for x in range(self.width):
            data[x] = 100
            data[(self.height - 1) * self.width + x] = 100
        for y in range(self.height):
            data[y * self.width] = 100
            data[y * self.width + self.width - 1] = 100

        message.data = data
        return message

    def mark_rectangle(
        self,
        data: list[int],
        *,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        value: int,
    ) -> None:
        for row in range(self.height):
            y = self.origin_y + (row + 0.5) * self.resolution
            for column in range(self.width):
                x = self.origin_x + (column + 0.5) * self.resolution
                if x_min <= x <= x_max and y_min <= y <= y_max:
                    data[row * self.width + column] = value

    def publish_map(self) -> None:
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
