#!/usr/bin/env python3
"""Follow a nav_msgs/Path with a simple differential-drive controller."""

from __future__ import annotations

import math
import time
from enum import Enum
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


class GoalControlState(str, Enum):
    """Explicit terminal-control states used by the path follower.

    ``APPROACHING`` is ordinary path tracking. ``CONFIRMING`` is entered only
    after the navigation pose reaches the goal tolerance. ``FINAL_APPROACH``
    is a bounded low-speed recovery after confirmation evidence times out.
    ``GOAL_LATCHED`` is the only successful state that permanently commands
    zero velocity for the current endpoint. ``GOAL_UNCONFIRMED`` is an explicit
    safe failure state after a bounded number of unsuccessful attempts.
    """

    APPROACHING = "APPROACHING"
    CONFIRMING = "CONFIRMING"
    FINAL_APPROACH = "FINAL_APPROACH"
    GOAL_LATCHED = "GOAL_LATCHED"
    GOAL_UNCONFIRMED = "GOAL_UNCONFIRMED"


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
        self.declare_parameter("final_approach_speed_m_s", 0.05)
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
        # A localized pose can remain inside the goal tolerance while the
        # independent confirmation source never becomes usable.  A finite
        # deadline changes that condition into a bounded low-speed final
        # approach instead of an unbounded stop.  Zero preserves the legacy
        # comparison behavior.
        self.declare_parameter("goal_confirmation_timeout_s", 0.0)
        self.declare_parameter("goal_confirmation_max_attempts", 0)
        self.declare_parameter("goal_confirmation_max_speed_m_s", 0.05)
        self.declare_parameter("goal_confirmation_max_pose_age_s", 0.15)
        # A scan matcher can produce a locally self-consistent pose that is
        # still wrong in the global map. When navigation uses that corrected
        # pose, also require the independent wheel/IMU estimate to be inside
        # the goal tolerance before declaring success.
        self.declare_parameter("goal_reference_topic", "/state_estimate")
        self.declare_parameter("goal_reference_tolerance", -1.0)
        self.declare_parameter("goal_reference_position_sigma_max_m", 0.15)
        self.declare_parameter(
            "goal_event_topic",
            "/path_follower_goal_event",
        )
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
        self.final_approach_speed_m_s = max(
            0.0,
            float(self.get_parameter("final_approach_speed_m_s").value),
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
        self.goal_confirmation_timeout_s = max(
            0.0,
            float(self.get_parameter("goal_confirmation_timeout_s").value),
        )
        self.goal_confirmation_max_attempts = max(
            0,
            int(self.get_parameter("goal_confirmation_max_attempts").value),
        )
        self.goal_confirmation_max_speed_m_s = max(
            0.0,
            float(self.get_parameter("goal_confirmation_max_speed_m_s").value),
        )
        self.goal_confirmation_max_pose_age_s = max(
            0.0,
            float(self.get_parameter("goal_confirmation_max_pose_age_s").value),
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
        goal_event_topic = str(self.get_parameter("goal_event_topic").value)
        self.require_goal_reference_for_goal = (
            odom_topic == "/localized_estimate"
            and self.goal_reference_topic != odom_topic
        )
        self.rotate_in_place_threshold = float(
            self.get_parameter("rotate_in_place_threshold").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.goal_event_publisher = self.create_publisher(
            String,
            goal_event_topic,
            10,
        )

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
        self.latest_goal_reference: Optional[Odometry] = None
        self.goal_confirmation_count = 0
        self.goal_confirmation_started_at: Optional[float] = None
        self.goal_confirmation_timeout_count = 0
        self.final_approach_reentry_count = 0
        self.final_approach_has_left_tolerance = False
        self.final_approach_heading: Optional[float] = None
        self.goal_state = GoalControlState.APPROACHING

    @staticmethod
    def stamp_seconds(message: Odometry) -> float:
        """Return the source timestamp carried by an odometry message."""
        stamp = message.header.stamp
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)

    def publish_goal_event(
        self,
        event: str,
        goal_distance: Optional[float],
    ) -> None:
        """Publish a timestamped controller decision for temporal audits.

        The normal velocity-control behavior is intentionally unchanged.  The
        event contains both the controller's current ROS time and the source
        timestamp of the pose consumed by this callback, so an offline audit
        can distinguish a fresh terminal decision from one based on an old
        estimate.  ``wall_time_s`` is monotonic and is only useful for
        comparing callback receipt order within one process.
        """
        node_stamp = self.get_clock().now().nanoseconds * 1.0e-9
        pose_stamp = (
            None
            if self.latest_odom is None
            else self.stamp_seconds(self.latest_odom)
        )
        pose_age = (
            None
            if pose_stamp is None
            else node_stamp - pose_stamp
        )
        pose_stamp_text = "" if pose_stamp is None else f"{pose_stamp:.9f}"
        pose_age_text = "" if pose_age is None else f"{pose_age:.9f}"
        goal_distance_text = (
            "" if goal_distance is None else f"{goal_distance:.9f}"
        )
        confirmation_start_text = (
            ""
            if self.goal_confirmation_started_at is None
            else f"{self.goal_confirmation_started_at:.9f}"
        )
        confirmation_duration_text = (
            ""
            if self.goal_confirmation_started_at is None
            else f"{max(0.0, node_stamp - self.goal_confirmation_started_at):.9f}"
        )
        message = String()
        message.data = (
            f"event={event};"
            f"goal_state={self.goal_state.value};"
            f"node_stamp_s={node_stamp:.9f};"
            f"pose_stamp_s={pose_stamp_text};"
            f"pose_age_s={pose_age_text};"
            f"goal_distance_m={goal_distance_text};"
            f"confirmation_start_time_s={confirmation_start_text};"
            f"confirmation_duration_s={confirmation_duration_text};"
            f"confirmation_timeout_count={self.goal_confirmation_timeout_count};"
            f"final_approach_reentry_count={self.final_approach_reentry_count};"
            f"wall_time_s={time.monotonic():.9f}"
        )
        self.goal_event_publisher.publish(message)

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
        if self.goal_confirmation_timeout_s > 0.0:
            self.get_logger().info(
                "Goal confirmation timeout is "
                f"{self.goal_confirmation_timeout_s:.3f} s; timeout is "
                "followed by bounded FINAL_APPROACH recovery."
            )
        self.get_logger().info(
            "Goal confirmation also requires navigation speed <= "
            f"{self.goal_confirmation_max_speed_m_s:.3f} m/s and pose age <= "
            f"{self.goal_confirmation_max_pose_age_s:.3f} s."
        )

    def set_goal_state(self, state: GoalControlState) -> None:
        """Change the explicit terminal-control state and log transitions."""
        if state == self.goal_state:
            return
        previous = self.goal_state.value
        self.goal_state = state
        self.get_logger().info(
            f"Goal control state: {previous} -> {state.value}."
        )
        # State transitions are diagnostic events in their own right.  The
        # evaluation logger cannot infer the live state from a later terminal
        # event, because FINAL_APPROACH may return to APPROACHING before the
        # run ends.  Publishing the transition keeps the final CSV state
        # aligned with the controller state machine rather than with the last
        # confirmation-related event only.
        self.publish_goal_event("goal_state_changed", None)

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
                self.goal_confirmation_count = 0
                self.goal_confirmation_started_at = None
                self.goal_confirmation_timeout_count = 0
                self.final_approach_reentry_count = 0
                self.final_approach_has_left_tolerance = False
                self.final_approach_heading = None
                self.set_goal_state(GoalControlState.APPROACHING)
                self.localization_accepted_events_since_entry = 0
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
                    self.goal_confirmation_started_at = None
                    self.goal_confirmation_timeout_count = 0
                    self.final_approach_reentry_count = 0
                    self.final_approach_has_left_tolerance = False
                    self.final_approach_heading = None
                    self.set_goal_state(GoalControlState.APPROACHING)
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
            self.goal_confirmation_started_at = None
            self.goal_confirmation_timeout_count = 0
            self.final_approach_reentry_count = 0
            self.final_approach_has_left_tolerance = False
            self.final_approach_heading = None
            self.set_goal_state(GoalControlState.APPROACHING)
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
            if self.goal_state == GoalControlState.CONFIRMING:
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
        if self.goal_state == GoalControlState.GOAL_LATCHED:
            # Once this endpoint has been reached, keep publishing zero
            # velocity even if estimator noise moves the reported distance a
            # few centimetres outside the tolerance.  A new endpoint is the
            # only event that clears the latch in path_callback().
            self.report_state(
                f"Stopped at latched endpoint; distance={goal_distance:.3f} m."
            )
            self.publish_stop()
            return

        if self.goal_state == GoalControlState.GOAL_UNCONFIRMED:
            # Repeated confirmation failure is an explicit safe failure, not
            # evidence of arrival and not permission to keep cycling through
            # low-speed recovery indefinitely.
            self.report_state(
                "Stopped because goal confirmation failed; awaiting a new path."
            )
            self.publish_stop()
            return

        if goal_distance <= self.goal_tolerance:
            if self.goal_state == GoalControlState.APPROACHING:
                self.goal_confirmation_started_at = (
                    self.get_clock().now().nanoseconds * 1.0e-9
                )
                # Accepted events from before entering the goal tolerance do
                # not count toward this terminal confirmation window.
                self.localization_accepted_events_since_entry = 0
                self.set_goal_state(GoalControlState.CONFIRMING)
                self.publish_goal_event("goal_tolerance_entered", goal_distance)
            elif (
                self.goal_state == GoalControlState.FINAL_APPROACH
                and self.final_approach_has_left_tolerance
            ):
                self.final_approach_reentry_count += 1
                self.goal_confirmation_started_at = (
                    self.get_clock().now().nanoseconds * 1.0e-9
                )
                self.localization_accepted_events_since_entry = 0
                self.final_approach_has_left_tolerance = False
                self.final_approach_heading = None
                self.set_goal_state(GoalControlState.CONFIRMING)
                self.publish_goal_event(
                    "goal_confirmation_reentered",
                    goal_distance,
                )

            confirmation_timed_out = False
            if self.goal_state == GoalControlState.CONFIRMING:
                confirmation_elapsed = (
                    None
                    if self.goal_confirmation_started_at is None
                    else (
                        self.get_clock().now().nanoseconds * 1.0e-9
                        - self.goal_confirmation_started_at
                    )
                )
                if (
                    self.goal_confirmation_timeout_s > 0.0
                    and confirmation_elapsed is not None
                    and confirmation_elapsed >= self.goal_confirmation_timeout_s
                ):
                    self.goal_confirmation_count = 0
                    self.localization_accepted_events_since_entry = 0
                    self.goal_confirmation_timeout_count += 1
                    self.final_approach_has_left_tolerance = False
                    if (
                        self.goal_confirmation_max_attempts > 0
                        and self.goal_confirmation_timeout_count
                        >= self.goal_confirmation_max_attempts
                    ):
                        self.set_goal_state(GoalControlState.GOAL_UNCONFIRMED)
                        self.get_logger().warning(
                            "Goal confirmation failed after "
                            f"{self.goal_confirmation_timeout_count} attempts; "
                            "stopping without declaring success."
                        )
                        self.publish_goal_event(
                            "goal_confirmation_failed",
                            goal_distance,
                        )
                        self.publish_stop()
                        return
                    # Once the estimated residual is only a few centimetres,
                    # recomputing atan2(goal - pose) at every control tick
                    # makes the bearing dominated by localization noise.
                    # Preserve the direction at this transition so recovery
                    # can keep making translational progress instead of
                    # oscillating between forward motion and saturated turns.
                    self.final_approach_heading = (
                        math.atan2(goal.y - position.y, goal.x - position.x)
                        if goal_distance > 1.0e-4
                        else yaw
                    )
                    self.set_goal_state(GoalControlState.FINAL_APPROACH)
                    confirmation_timed_out = True
                    self.get_logger().warning(
                        "Goal confirmation timed out; entering bounded "
                        "FINAL_APPROACH without declaring success."
                    )
                    self.publish_goal_event(
                        "goal_confirmation_timeout",
                        goal_distance,
                    )

                if not confirmation_timed_out:
                    confirmation_speed = math.hypot(
                        self.latest_odom.twist.twist.linear.x,
                        self.latest_odom.twist.twist.linear.y,
                    )
                    confirmation_pose_age = (
                        self.get_clock().now().nanoseconds * 1.0e-9
                        - self.stamp_seconds(self.latest_odom)
                    )
                    if confirmation_speed > self.goal_confirmation_max_speed_m_s:
                        self.goal_confirmation_count = 0
                        self.localization_accepted_events_since_entry = 0
                        self.report_state(
                            "At estimated goal, waiting for low speed; "
                            f"speed={confirmation_speed:.3f} m/s."
                        )
                        self.publish_stop()
                        return
                    if (
                        confirmation_pose_age < 0.0
                        or confirmation_pose_age
                        > self.goal_confirmation_max_pose_age_s
                    ):
                        self.goal_confirmation_count = 0
                        self.localization_accepted_events_since_entry = 0
                        self.report_state(
                            "At estimated goal, waiting for a fresh pose; "
                            f"age={confirmation_pose_age:.3f} s."
                        )
                        self.publish_stop()
                        return
                    if self.require_localization_match_for_goal:
                        if self.localization_match_valid is not True:
                            self.goal_confirmation_count = 0
                            self.localization_accepted_events_since_entry = 0
                            self.report_state(
                                "At estimated goal, waiting for a valid "
                                "localization match."
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
                                "At localized goal, waiting for independent "
                                f"estimate; distance={reference_distance:.3f} m."
                            )
                            self.publish_stop()
                            return
                        reference_sigma = planar_position_sigma(
                            self.latest_goal_reference
                        )
                        if (
                            reference_sigma is None
                            or reference_sigma
                            > self.goal_reference_position_sigma_max_m
                        ):
                            self.goal_confirmation_count = 0
                            self.localization_accepted_events_since_entry = 0
                            sigma_text = (
                                "unavailable"
                                if reference_sigma is None
                                else f"{reference_sigma:.3f} m"
                            )
                            self.report_state(
                                "At localized goal, waiting for independent "
                                f"estimate uncertainty; sigma={sigma_text}."
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
                        if (
                            self.goal_confirmation_count
                            < self.goal_confirmation_samples
                        ):
                            self.report_state(
                                "Confirming goal distance; "
                                f"sample={self.goal_confirmation_count}/"
                                f"{self.goal_confirmation_samples}."
                            )
                            self.publish_stop()
                            return
                    self.set_goal_state(GoalControlState.GOAL_LATCHED)
                    self.goal_reached = True
                    self.publish_goal_event("goal_reached_latched", goal_distance)
                    self.get_logger().info("Planned path goal reached.")
                    self.report_state(
                        f"Stopped at planned endpoint; distance={goal_distance:.3f} m."
                    )
                    self.publish_stop()
                    return
        else:
            self.goal_confirmation_count = 0
            self.localization_accepted_events_since_entry = 0
            if self.goal_state == GoalControlState.CONFIRMING:
                self.publish_goal_event("goal_tolerance_exited", goal_distance)
                self.goal_confirmation_started_at = None
                self.set_goal_state(GoalControlState.APPROACHING)
            elif self.goal_state == GoalControlState.FINAL_APPROACH:
                self.final_approach_has_left_tolerance = True
                if goal_distance > self.final_approach_distance:
                    self.final_approach_heading = None
                    self.set_goal_state(GoalControlState.APPROACHING)

        final_approach = (
            self.goal_state == GoalControlState.FINAL_APPROACH
            or goal_distance <= self.final_approach_distance
        )
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

        if (
            self.goal_state == GoalControlState.FINAL_APPROACH
            and self.final_approach_heading is not None
        ):
            target_heading = self.final_approach_heading
        else:
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
            if (
                self.goal_state == GoalControlState.FINAL_APPROACH
                and target_distance > 1.0e-4
            ):
                # Confirmation failure is not evidence of arrival.  Creep
                # toward the endpoint at a bounded speed until the pose exits
                # the tolerance and can be re-confirmed normally.
                command.linear.x = min(
                    self.final_approach_speed_m_s,
                    max(command.linear.x, self.final_approach_speed_m_s),
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
