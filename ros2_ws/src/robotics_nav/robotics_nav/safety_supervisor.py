#!/usr/bin/env python3
"""Velocity safety layer based on a LiDAR stopping-distance envelope."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetySupervisor(Node):
    """Gate raw velocity commands using the closest forward LiDAR return."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")

        self.declare_parameter("max_deceleration", 0.8)
        self.declare_parameter("sensor_latency", 0.10)
        self.declare_parameter("safety_margin", 0.15)
        # Keep additional clearance beyond the planner's footprint inflation.
        # The speed-dependent envelope can still impose a larger distance
        # when the commanded speed requires it.
        self.declare_parameter("minimum_clearance", 0.50)
        # Prevent command chattering without requiring a large clearance jump
        # after the robot has already turned away from the obstacle.
        self.declare_parameter("clearance_hysteresis", 0.03)
        self.declare_parameter("front_angle_deg", 60.0)
        self.declare_parameter("side_inner_angle_deg", 30.0)
        self.declare_parameter("recovery_turn_speed", 0.60)
        # Calibration for this Gazebo laser frame: a positive scan-side angle
        # maps to a negative base angular command in this model.
        self.declare_parameter("recovery_turn_sign", -1.0)
        self.declare_parameter("max_tilt_deg", 10.0)
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
        self.side_inner_angle = math.radians(
            float(self.get_parameter("side_inner_angle_deg").value)
        )
        self.recovery_turn_speed = float(
            self.get_parameter("recovery_turn_speed").value
        )
        self.recovery_turn_sign = float(
            self.get_parameter("recovery_turn_sign").value
        )
        self.max_tilt = math.radians(
            float(self.get_parameter("max_tilt_deg").value)
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.override_publisher = self.create_publisher(
            Bool,
            "/safety_override",
            10,
        )
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
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_safe_command)

        self.latest_raw_command: Optional[Twist] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.intervention_count = 0
        self.was_blocked = False
        self.recovery_turn = 0.0
        self.tilt_stop_active = False

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def publish_override_state(self, active: bool) -> None:
        message = Bool()
        message.data = active
        self.override_publisher.publish(message)

    def raw_callback(self, message: Twist) -> None:
        self.latest_raw_command = message

    def scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def roll_pitch(self) -> Optional[tuple[float, float]]:
        """Return roll and pitch from the latest odometry orientation."""
        if self.latest_odom is None:
            return None

        orientation = self.latest_odom.pose.pose.orientation
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w

        sin_roll = 2.0 * (w * x + y * z)
        cos_roll = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sin_roll, cos_roll)

        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)
        return roll, pitch

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
        """Return the closest valid return in a side sector.

        The central forward sector is excluded so a single obstacle corner
        does not make the preferred recovery side alternate every scan.
        """
        closest = None
        saw_clear_ray = False
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            in_sector = self.side_inner_angle <= angle <= math.pi / 2.0
            if not left:
                in_sector = -math.pi / 2.0 <= angle <= -self.side_inner_angle
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
        direction = (
            self.recovery_turn_sign
            if left_clearance >= right_clearance
            else -self.recovery_turn_sign
        )
        return direction * self.recovery_turn_speed

    def publish_safe_command(self) -> None:
        if self.latest_raw_command is None:
            return

        if self.latest_scan is None:
            self.publisher.publish(Twist())
            self.publish_override_state(True)
            if not self.was_blocked:
                self.get_logger().warn("No LiDAR scan received; holding robot stopped.")
            self.was_blocked = True
            return

        tilt = self.roll_pitch()
        if tilt is None:
            self.publisher.publish(Twist())
            self.publish_override_state(True)
            if not self.tilt_stop_active:
                self.get_logger().warn("No /odom pose received; holding robot stopped.")
            self.tilt_stop_active = True
            return

        roll, pitch = tilt
        if max(abs(roll), abs(pitch)) > self.max_tilt:
            self.publisher.publish(Twist())
            self.publish_override_state(True)
            if not self.tilt_stop_active:
                self.get_logger().error(
                    "Tilt safety stop: "
                    f"roll={math.degrees(roll):.1f} deg, "
                    f"pitch={math.degrees(pitch):.1f} deg, "
                    f"limit={math.degrees(self.max_tilt):.1f} deg."
                )
            self.tilt_stop_active = True
            self.was_blocked = True
            self.recovery_turn = 0.0
            return

        self.tilt_stop_active = False

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
                # Safety owns the recovery direction. The path follower can
                # request a turn toward an obstacle when its map/path is stale
                # or the robot has cut a corner, so use measured side clearance
                # rather than trusting raw.angular.z in this state.
                self.recovery_turn = self.choose_recovery_turn(self.latest_scan)
            # Keep the initial recovery direction for the complete blocked
            # episode. Recomputing left/right clearance while the robot turns
            # makes the obstacle move between the two scan sectors and can
            # cause the supervisor to alternate directions indefinitely.
            # Do not forward the path follower's angular command here: the
            # safety supervisor owns the command while the forward sector is
            # blocked.
            safe_command = Twist()
            safe_command.angular.z = self.recovery_turn
            self.publisher.publish(safe_command)
            self.publish_override_state(True)
            if not self.was_blocked:
                self.intervention_count += 1
                observed = "unknown" if distance is None else f"{distance:.3f} m"
                self.get_logger().warn(
                    "Safety recovery: front distance "
                    f"{observed} <= stopping envelope {required_distance:.3f} m. "
                    f"angular_z={safe_command.angular.z:.2f} rad/s. "
                    f"Interventions: {self.intervention_count}"
                )
        else:
            self.recovery_turn = 0.0
            self.publisher.publish(raw)
            self.publish_override_state(False)

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
