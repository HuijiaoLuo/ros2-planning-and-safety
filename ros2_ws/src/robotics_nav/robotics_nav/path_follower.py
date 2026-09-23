#!/usr/bin/env python3
"""Follow a nav_msgs/Path with a simple differential-drive controller."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def planar_position_sigma(message: Odometry) -> Optional[float]:
    """Return the largest 1-sigma position uncertainty from x/y covariance.

    The covariance is a 2-D ellipse in the pose message. Using its largest
    eigenvalue avoids accepting a goal along the ellipse's most uncertain
    direction, even when the diagonal entries alone look acceptable.
    """
    covariance = message.pose.covariance
    if len(covariance) < 8:
        return None
    pxx = float(covariance[0])
    pxy = 0.5 * (float(covariance[1]) + float(covariance[6]))
    pyy = float(covariance[7])
    if not all(math.isfinite(value) for value in (pxx, pxy, pyy)):
        return None
    trace = pxx + pyy
    discriminant = math.sqrt(max(0.0, (pxx - pyy) ** 2 + 4.0 * pxy**2))
    largest_variance = 0.5 * (trace + discriminant)
    if largest_variance < 0.0:
        return None
    return math.sqrt(largest_variance)


class PathFollower(Node):
    """Convert a planned path and odometry into raw velocity commands."""

    def __init__(self) -> None:
        super().__init__("path_follower")

        # Keep the target close to the grid path. A long lookahead makes the
        # unicycle controller cut corners around inflated obstacles.
        self.declare_parameter("lookahead_distance", 0.10)
        # Use a tighter final-position tolerance than the grid resolution so
        # the robot does not stop visibly far from the exact goal marker.
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("distance_gain", 0.8)
        self.declare_parameter("heading_gain", 2.0)
        # Close to the final goal, following changing grid waypoints can make
        # the desired bearing jump from one side to the other. Track the
        # endpoint directly with a gentler angular controller instead.
        self.declare_parameter("final_approach_distance", 0.60)
        self.declare_parameter("final_approach_heading_gain", 1.0)
        self.declare_parameter("final_approach_max_angular_speed", 0.60)
        self.declare_parameter("heading_deadband", 0.03)
        self.declare_parameter("odom_topic", "/odom")
        # When the controller follows a scan-corrected pose, a distance
        # threshold alone can produce a false goal if the last correction is
        # stale or ambiguous. Require a current valid localization match in
        # that mode before latching the terminal stop.
        self.declare_parameter("require_localization_match_for_goal", False)
        self.declare_parameter("localization_match_valid_topic", "/localization_match_valid")
        self.declare_parameter("localization_match_status_topic", "/localization_match_status")
        self.declare_parameter("goal_confirmation_samples", 5)
        # A scan matcher can produce a locally self-consistent pose that is
        # still wrong in the global map. When navigation uses that corrected
        # pose, also require the independent wheel/IMU estimate to be inside
        # the goal tolerance before declaring success.
        self.declare_parameter("goal_reference_topic", "/state_estimate")
        self.declare_parameter("goal_reference_tolerance", -1.0)
        self.declare_parameter("goal_reference_position_sigma_max_m", 0.15)
        # Grid paths contain sharp 90-degree corners. Rotate before driving
        # through a large heading error instead of cutting the corner.
        self.declare_parameter("rotate_in_place_threshold", math.pi / 6.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.lookahead_distance = float(
            self.get_parameter("lookahead_distance").value
        )
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.distance_gain = float(self.get_parameter("distance_gain").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.final_approach_distance = float(
            self.get_parameter("final_approach_distance").value
        )
        self.final_approach_heading_gain = float(
            self.get_parameter("final_approach_heading_gain").value
        )
        self.final_approach_max_angular_speed = float(
            self.get_parameter("final_approach_max_angular_speed").value
        )
        self.heading_deadband = float(
            self.get_parameter("heading_deadband").value
        )
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_topic = odom_topic
        self.require_localization_match_for_goal = bool(
            self.get_parameter("require_localization_match_for_goal").value
        ) or odom_topic == "/localized_estimate"
        localization_match_valid_topic = str(
            self.get_parameter("localization_match_valid_topic").value
        )
        localization_match_status_topic = str(
            self.get_parameter("localization_match_status_topic").value
        )
        self.goal_confirmation_samples = max(
            1, int(self.get_parameter("goal_confirmation_samples").value)
        )
        self.goal_reference_topic = str(
            self.get_parameter("goal_reference_topic").value
        )
        configured_reference_tolerance = float(
            self.get_parameter("goal_reference_tolerance").value
        )
        self.goal_reference_tolerance = (
            self.goal_tolerance
            if configured_reference_tolerance <= 0.0
            else configured_reference_tolerance
        )
        self.goal_reference_position_sigma_max_m = float(
            self.get_parameter("goal_reference_position_sigma_max_m").value
        )
        self.require_goal_reference_for_goal = (
            odom_topic == "/localized_estimate"
            and self.goal_reference_topic != odom_topic
        )
        self.rotate_in_place_threshold = float(
            self.get_parameter("rotate_in_place_threshold").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel_raw", 10)

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_subscription = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            path_qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.localization_valid_subscription = None
        self.localization_status_subscription = None
        if self.require_localization_match_for_goal:
            self.localization_valid_subscription = self.create_subscription(
                Bool,
                localization_match_valid_topic,
                self.localization_valid_callback,
                10,
            )
            self.localization_status_subscription = self.create_subscription(
                String,
                localization_match_status_topic,
                self.localization_status_callback,
                10,
            )
        self.goal_reference_subscription = None
        if self.require_goal_reference_for_goal:
            self.goal_reference_subscription = self.create_subscription(
                Odometry,
                self.goal_reference_topic,
                self.goal_reference_callback,
                qos_profile_sensor_data,
            )
        self.timer = self.create_timer(1.0 / publish_rate, self.control_loop)

        self.latest_path: Optional[Path] = None
        self.latest_odom: Optional[Odometry] = None
        self.goal_reached = False
        # The planner may republish a path while the robot is already inside
        # the goal tolerance.  Keep the endpoint separately so that a normal
        # replan (same destination, different start prefix) does not clear
        # the terminal-stop state and command the robot to move again.
        self.goal_endpoint: Optional[tuple[float, float]] = None
        self.last_control_state: Optional[str] = None
        self.received_odom = False
        self.localization_match_valid: Optional[bool] = None
        self.localization_match_status: Optional[str] = None
        self.localization_accepted_events_since_entry = 0
        self.in_goal_tolerance = False
        self.latest_goal_reference: Optional[Odometry] = None
        self.goal_confirmation_count = 0

        self.get_logger().info(
            "Path follower inputs: "
            f"path=/plan, pose={self.odom_topic}, output=/cmd_vel_raw."
        )
        if self.require_localization_match_for_goal:
            self.get_logger().info(
                "Goal confirmation requires "
                f"{self.goal_confirmation_samples} consecutive accepted "
                "LiDAR match events after entering the goal tolerance."
            )
        if self.require_goal_reference_for_goal:
            self.get_logger().info(
                "Goal confirmation also requires "
                f"{self.goal_reference_topic} within "
                f"{self.goal_reference_tolerance:.3f} m."
            )
            self.get_logger().info(
                "Goal confirmation also requires independent position "
                "uncertainty <= "
                f"{self.goal_reference_position_sigma_max_m:.3f} m (1-sigma)."
            )

    def report_state(self, state: str) -> None:
        """Log only control-state transitions, not every timer tick."""
        if state != self.last_control_state:
            self.get_logger().info(state)
            self.last_control_state = state

    def path_callback(self, message: Path) -> None:
        self.latest_path = message
        if message.poses:
            endpoint = message.poses[-1].pose.position
            new_endpoint = (float(endpoint.x), float(endpoint.y))
            if self.goal_endpoint is None:
                # The first path establishes the destination for this run.
                self.goal_reached = False
                self.last_control_state = None
            else:
                endpoint_change = math.hypot(
                    new_endpoint[0] - self.goal_endpoint[0],
                    new_endpoint[1] - self.goal_endpoint[1],
                )
                if endpoint_change > max(self.goal_tolerance, 1e-3):
                    # A materially different endpoint is a new navigation
                    # task, so a previous terminal-stop decision is invalid.
                    self.goal_reached = False
                    self.last_control_state = None
                    self.goal_confirmation_count = 0
                    self.in_goal_tolerance = False
                    self.localization_accepted_events_since_entry = 0
            self.goal_endpoint = new_endpoint
            self.get_logger().info(
                f"Received path with {len(message.poses)} poses; "
                f"endpoint=({endpoint.x:.2f}, {endpoint.y:.2f})."
            )
        else:
            self.goal_endpoint = None
            self.goal_reached = False
            self.last_control_state = None
            self.goal_confirmation_count = 0
            self.in_goal_tolerance = False
            self.localization_accepted_events_since_entry = 0
            self.get_logger().warn("Received an empty path.")

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        if not self.received_odom:
            self.received_odom = True
            self.get_logger().info(
                f"Received first pose message on {self.odom_topic}."
            )

    def localization_valid_callback(self, message: Bool) -> None:
        """Store whether the latest scan-to-map match passed its gates."""
        self.localization_match_valid = bool(message.data)

    def localization_status_callback(self, message: String) -> None:
        """Track fresh matcher events instead of sampling a latched Bool.

        ``/localization_match_valid`` describes the latest matcher state and
        can remain ``true`` between scans.  Counting control-loop ticks would
        therefore mistake one accepted scan for many independent matches.
        This callback counts only new ``accepted`` status messages, and a
        rejected status clears the consecutive-event streak.
        """
        status = str(message.data)
        self.localization_match_status = status
        if status == "accepted":
            if self.in_goal_tolerance:
                self.localization_accepted_events_since_entry += 1
        else:
            self.localization_accepted_events_since_entry = 0

    def goal_reference_callback(self, message: Odometry) -> None:
        """Store the independent pose used to confirm a localized goal.

        This reference is deliberately kept separate from the pose that
        drives the controller. It is the wheel/IMU EKF estimate in the
        localized-navigation experiments, so a single scan-matching local
        minimum cannot satisfy both goal tests by construction.
        """
        self.latest_goal_reference = message

    def publish_stop(self) -> None:
        # The evaluation logger may shut down the ROS context before this
        # node's final stop callback runs.  The stop command is best effort at
        # shutdown; never turn a clean experiment timeout into a process
        # failure by publishing through an invalid context.
        if not rclpy.ok():
            return
        try:
            self.publisher.publish(Twist())
        except Exception:
            # The Python exception class differs across ROS 2 distributions.
            # If the context is still live, let real publish failures surface;
            # during shutdown, suppress the DDS teardown race only.
            if rclpy.ok():
                raise

    def select_target(self, _x: float, _y: float) -> Optional[tuple[float, float]]:
        """Select the first ordered waypoint at the configured lookahead.

        Distance is accumulated along the path prefix rather than measured to
        the globally nearest waypoint. That preserves the planner's detour
        around obstacles and avoids jumping across a U-shaped route.
        """
        if self.latest_path is None or not self.latest_path.poses:
            return None

        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in self.latest_path.poses
        ]
        # The planner publishes an ordered collision-free path whose first
        # pose is the current start cell. Follow its prefix in order. Choosing
        # the globally nearest pose can jump across a U-shaped detour: a later
        # point may be geometrically closer while the intervening path still
        # has to go around an obstacle.
        travelled = 0.0
        for index in range(len(points) - 1):
            travelled += math.hypot(
                points[index + 1][0] - points[index][0],
                points[index + 1][1] - points[index][1],
            )
            if travelled >= self.lookahead_distance:
                return points[index + 1]
        return points[-1]

    def control_loop(self) -> None:
        """Turn the current path target into a bounded unicycle command.

        The controller first handles missing inputs and the endpoint stop
        condition. Away from the goal it points toward a lookahead waypoint;
        near the goal it tracks the exact endpoint. Linear speed is reduced
        as heading error grows, and large errors force an in-place rotation so
        the robot does not cut across inflated grid corners.
        """
        if self.latest_path is None:
            self.report_state("Waiting for /plan.")
            self.publish_stop()
            return

        if self.latest_odom is None:
            self.report_state(f"Waiting for {self.odom_topic}.")
            self.publish_stop()
            return

        if not self.latest_path.poses:
            self.report_state("Stopped because /plan is empty.")
            self.publish_stop()
            return

        position = self.latest_odom.pose.pose.position
        orientation = self.latest_odom.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        goal = self.latest_path.poses[-1].pose.position
        goal_distance = math.hypot(goal.x - position.x, goal.y - position.y)
        if self.goal_reached:
            # Once this endpoint has been reached, keep publishing zero
            # velocity even if estimator noise moves the reported distance a
            # few centimetres outside the tolerance.  A new endpoint is the
            # only event that clears the latch in path_callback().
            self.report_state(
                f"Stopped at latched endpoint; distance={goal_distance:.3f} m."
            )
            self.publish_stop()
            return
        if goal_distance <= self.goal_tolerance:
            if self.require_localization_match_for_goal:
                if not self.in_goal_tolerance:
                    # Start a new confirmation window. An accepted status
                    # from before entering the goal does not count toward
                    # terminal confirmation.
                    self.in_goal_tolerance = True
                    self.localization_accepted_events_since_entry = 0
                if self.localization_match_valid is not True:
                    self.goal_confirmation_count = 0
                    self.localization_accepted_events_since_entry = 0
                    self.report_state(
                        "At estimated goal, waiting for a valid localization match."
                    )
                    self.publish_stop()
                    return
                if self.localization_match_status != "accepted":
                    self.goal_confirmation_count = 0
                    self.localization_accepted_events_since_entry = 0
                    status = self.localization_match_status or "not_received"
                    self.report_state(
                        "At estimated goal, waiting for a fresh accepted "
                        f"localization match; status={status}."
                    )
                    self.publish_stop()
                    return
            if self.require_goal_reference_for_goal:
                if self.latest_goal_reference is None:
                    self.goal_confirmation_count = 0
                    self.localization_accepted_events_since_entry = 0
                    self.report_state(
                        f"At estimated goal, waiting for {self.goal_reference_topic}."
                    )
                    self.publish_stop()
                    return
                reference_position = self.latest_goal_reference.pose.pose.position
                reference_distance = math.hypot(
                    goal.x - reference_position.x,
                    goal.y - reference_position.y,
                )
                if reference_distance > self.goal_reference_tolerance:
                    self.goal_confirmation_count = 0
                    self.localization_accepted_events_since_entry = 0
                    self.report_state(
                        "At localized goal, waiting for independent estimate; "
                        f"distance={reference_distance:.3f} m."
                    )
                    self.publish_stop()
                    return
                reference_sigma = planar_position_sigma(self.latest_goal_reference)
                if (
                    reference_sigma is None
                    or reference_sigma > self.goal_reference_position_sigma_max_m
                ):
                    self.goal_confirmation_count = 0
                    self.localization_accepted_events_since_entry = 0
                    sigma_text = (
                        "unavailable"
                        if reference_sigma is None
                        else f"{reference_sigma:.3f} m"
                    )
                    self.report_state(
                        "At localized goal, waiting for independent estimate "
                        f"uncertainty; sigma={sigma_text}."
                    )
                    self.publish_stop()
                    return
            if self.require_localization_match_for_goal:
                if (
                    self.localization_accepted_events_since_entry
                    < self.goal_confirmation_samples
                ):
                    self.goal_confirmation_count = 0
                    self.report_state(
                        "Confirming fresh LiDAR matches; "
                        f"accepted={self.localization_accepted_events_since_entry}/"
                        f"{self.goal_confirmation_samples}."
                    )
                    self.publish_stop()
                    return
            else:
                self.goal_confirmation_count += 1
                if self.goal_confirmation_count < self.goal_confirmation_samples:
                    self.report_state(
                        "Confirming goal distance; "
                        f"sample={self.goal_confirmation_count}/"
                        f"{self.goal_confirmation_samples}."
                    )
                    self.publish_stop()
                    return
            if not self.goal_reached:
                self.get_logger().info("Planned path goal reached.")
                self.goal_reached = True
            self.report_state(
                f"Stopped at planned endpoint; distance={goal_distance:.3f} m."
            )
            self.publish_stop()
            return
        self.goal_confirmation_count = 0
        self.in_goal_tolerance = False
        self.localization_accepted_events_since_entry = 0

        final_approach = goal_distance <= self.final_approach_distance
        if final_approach:
            # The final path endpoint is more stable than the next grid cell
            # once the robot is close enough to the goal.
            target_x, target_y = goal.x, goal.y
        else:
            target = self.select_target(position.x, position.y)
            if target is None:
                self.report_state("Stopped because no path target is available.")
                self.publish_stop()
                return
            target_x, target_y = target

        target_heading = math.atan2(target_y - position.y, target_x - position.x)
        heading_error = wrap_angle(target_heading - yaw)
        target_distance = math.hypot(target_x - position.x, target_y - position.y)

        if abs(heading_error) < self.heading_deadband:
            heading_error = 0.0

        angular_gain = (
            self.final_approach_heading_gain
            if final_approach
            else self.heading_gain
        )
        angular_limit = (
            self.final_approach_max_angular_speed
            if final_approach
            else self.max_angular_speed
        )

        command = Twist()
        command.angular.z = clamp(
            angular_gain * heading_error,
            -angular_limit,
            angular_limit,
        )
        if abs(heading_error) <= self.rotate_in_place_threshold:
            # Reduce forward speed continuously as the heading error grows.
            # This prevents a command such as v=0.15, omega=1.20 from cutting
            # across an inflated-grid corner near an obstacle.
            heading_scale = 1.0 - (
                abs(heading_error) / self.rotate_in_place_threshold
            )
            command.linear.x = clamp(
                self.distance_gain * target_distance * heading_scale,
                0.0,
                self.max_linear_speed,
            )
        else:
            command.linear.x = 0.0

        self.report_state(
            f"Tracking {'final goal' if final_approach else 'path'}; "
            f"target=({target_x:.2f}, {target_y:.2f}), "
            f"v={command.linear.x:.2f}, omega={command.angular.z:.2f}."
        )
        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
