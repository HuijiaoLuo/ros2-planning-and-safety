#!/usr/bin/env python3
"""Velocity safety layer based on a LiDAR stopping-distance envelope."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SafetySupervisor(Node):
    """Gate raw velocity commands using the closest forward LiDAR return."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")

        self.declare_parameter("max_deceleration", 0.8)
        self.declare_parameter("sensor_latency", 0.10)
        self.declare_parameter("safety_margin", 0.15)
        self.declare_parameter("minimum_clearance", 0.35)
        # Prevent command chattering when the measured range oscillates around
        # the intervention threshold, e.g. 0.348 m <-> 0.352 m.
        self.declare_parameter("clearance_hysteresis", 0.10)
        self.declare_parameter("front_angle_deg", 60.0)
        self.declare_parameter("recovery_turn_speed", 0.60)
        # A tiny angular command is not enough to escape a blocked state.
        # Below this threshold, use the latched recovery turn instead.
        self.declare_parameter("minimum_planned_turn_speed", 0.10)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.max_deceleration = float(self.get_parameter("max_deceleration").value)
        self.sensor_latency = float(self.get_parameter("sensor_latency").value)
        self.safety_margin = float(self.get_parameter("safety_margin").value)
        self.minimum_clearance = float(
            self.get_parameter("minimum_clearance").value
        )
        self.clearance_hysteresis = float(
            self.get_parameter("clearance_hysteresis").value
        )
        self.front_angle = math.radians(
            float(self.get_parameter("front_angle_deg").value)
        )
        self.recovery_turn_speed = float(
            self.get_parameter("recovery_turn_speed").value
        )
        self.minimum_planned_turn_speed = float(
            self.get_parameter("minimum_planned_turn_speed").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.raw_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self.raw_callback,
            10,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_safe_command)

        self.latest_raw_command: Optional[Twist] = None
        self.latest_scan: Optional[LaserScan] = None
        self.intervention_count = 0
        self.was_blocked = False
        self.recovery_turn = 0.0

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def raw_callback(self, message: Twist) -> None:
        self.latest_raw_command = message

    def scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message

    def front_distance(self, scan: LaserScan) -> Optional[float]:
        """Return the closest valid range inside the configured front sector."""
        if scan.angle_increment == 0.0:
            return None

        closest = None
        saw_clear_ray = False
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > self.front_angle:
                continue
            if math.isnan(value):
                continue
            if math.isinf(value):
                saw_clear_ray = True
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            closest = value if closest is None else min(closest, value)

        # A valid LaserScan uses +inf for rays that saw no obstacle within the
        # sensor range. That means the front sector is clear, not unknown.
        # Treat it as the maximum measurable distance. NaN-only scans remain
        # invalid and are handled conservatively by the caller.
        if closest is not None:
            return closest
        return scan.range_max if saw_clear_ray else None

    def stop_distance(self, speed: float) -> float:
        speed = max(0.0, speed)
        braking_distance = speed * speed / (2.0 * self.max_deceleration)
        return braking_distance + speed * self.sensor_latency + self.safety_margin

    def side_clearance(self, scan: LaserScan, *, left: bool) -> float:
        """Return the closest valid return in a 90-degree side sector."""
        closest = None
        saw_clear_ray = False
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            in_sector = 0.0 <= angle <= math.pi / 2.0
            if not left:
                in_sector = -math.pi / 2.0 <= angle < 0.0
            if not in_sector or math.isnan(value):
                continue
            if math.isinf(value):
                saw_clear_ray = True
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            closest = value if closest is None else min(closest, value)

        if closest is not None:
            return closest
        return scan.range_max if saw_clear_ray else 0.0

    def choose_recovery_turn(self, scan: LaserScan) -> float:
        """Turn toward the side with more measured clearance."""
        left_clearance = self.side_clearance(scan, left=True)
        right_clearance = self.side_clearance(scan, left=False)
        direction = 1.0 if left_clearance >= right_clearance else -1.0
        return direction * self.recovery_turn_speed

    def publish_safe_command(self) -> None:
        if self.latest_raw_command is None:
            return

        if self.latest_scan is None:
            self.publisher.publish(Twist())
            if not self.was_blocked:
                self.get_logger().warn("No LiDAR scan received; holding robot stopped.")
            self.was_blocked = True
            return

        raw = self.latest_raw_command
        distance = self.front_distance(self.latest_scan)
        dynamic_stop_distance = self.stop_distance(abs(raw.linear.x))
        required_distance = max(dynamic_stop_distance, self.minimum_clearance)

        # Use separate thresholds for entering and leaving the blocked state.
        # Without this hysteresis, a noisy/rasterized scan can alternate
        # between forwarding and stopping at every timer tick.
        if self.was_blocked:
            clear_distance = required_distance + self.clearance_hysteresis
            blocked = distance is None or distance <= clear_distance
        else:
            blocked = distance is None or distance <= required_distance

        if blocked:
            if not self.was_blocked:
                self.recovery_turn = self.choose_recovery_turn(self.latest_scan)
            # Stop forward motion. If the path follower already knows which
            # way the planned path turns, preserve that angular command. The
            # supervisor supplies its own turn only as a fallback; otherwise
            # the two controllers can fight each other at the obstacle edge.
            safe_command = Twist()
            if abs(raw.angular.z) >= self.minimum_planned_turn_speed:
                safe_command.angular.z = raw.angular.z
            else:
                safe_command.angular.z = self.recovery_turn
            self.publisher.publish(safe_command)
            if not self.was_blocked:
                self.intervention_count += 1
                observed = "unknown" if distance is None else f"{distance:.3f} m"
                self.get_logger().warn(
                    "Safety stop: front distance "
                    f"{observed} <= stopping envelope {required_distance:.3f} m. "
                    f"Interventions: {self.intervention_count}"
                )
        else:
            self.recovery_turn = 0.0
            self.publisher.publish(raw)

        self.was_blocked = blocked


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
