#!/usr/bin/env python3
"""Print LiDAR sector minima from a running ROS2 simulation."""

from __future__ import annotations

import math

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def sector_minimum(scan: LaserScan, lower: float, upper: float) -> float:
    """Return the minimum finite range in an angular sector."""
    values = []
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if lower <= angle <= upper:
            if math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                values.append(value)
    return min(values) if values else float("inf")


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("scan_debug")
    received: list[LaserScan] = []

    def callback(message: LaserScan) -> None:
        received.append(message)

    node.create_subscription(
        LaserScan,
        "/scan",
        callback,
        qos_profile_sensor_data,
    )

    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if received:
            break

    if not received:
        print("No LaserScan received from /scan.")
    else:
        scan = received[0]
        degree = math.pi / 180.0
        front = sector_minimum(scan, -60.0 * degree, 60.0 * degree)
        left = sector_minimum(scan, 30.0 * degree, 90.0 * degree)
        right = sector_minimum(scan, -90.0 * degree, -30.0 * degree)
        print(f"front +/-60 deg : {front:.3f} m")
        print(f"left  30..90 deg: {left:.3f} m")
        print(f"right -90..-30  : {right:.3f} m")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
