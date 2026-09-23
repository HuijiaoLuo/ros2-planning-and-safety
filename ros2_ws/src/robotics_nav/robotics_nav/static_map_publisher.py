#!/usr/bin/env python3
"""Publish a small deterministic occupancy grid for the first ROS2 experiment."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from robotics_nav.scenario_profiles import (
    build_occupancy_data,
    get_scenario_profile,
)


class StaticMapPublisher(Node):
    """Publish a map aligned with the obstacle in the Gazebo world."""

    def __init__(self) -> None:
        super().__init__("static_map_publisher")

        self.declare_parameter("width", 60)
        self.declare_parameter("height", 60)
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("origin_x", -1.0)
        self.declare_parameter("origin_y", -3.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("scenario", "baseline_obstacle")

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.scenario_name = str(self.get_parameter("scenario").value)
        self.scenario = get_scenario_profile(self.scenario_name)

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
        self.get_logger().info(
            f"Publishing scenario '{self.scenario.name}': {self.scenario.description}"
        )

    def build_map(self) -> OccupancyGrid:
        """Build the deterministic map used by the simulation experiments.

        The obstacle geometry comes from the shared scenario profile used by
        the Gazebo world renderer. The message is cached because only its
        timestamp changes between publications.
        """
        message = OccupancyGrid()
        message.header.frame_id = self.frame_id
        message.info.resolution = self.resolution
        message.info.width = self.width
        message.info.height = self.height
        message.info.origin.position.x = self.origin_x
        message.info.origin.position.y = self.origin_y
        message.info.origin.orientation.w = 1.0

        message.data = build_occupancy_data(
            self.scenario,
            width=self.width,
            height=self.height,
            resolution_m=self.resolution,
            origin_x_m=self.origin_x,
            origin_y_m=self.origin_y,
        )
        return message

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
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
