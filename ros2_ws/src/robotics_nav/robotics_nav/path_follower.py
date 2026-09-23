#!/usr/bin/env python3
"""Follow a nav_msgs/Path with a simple differential-drive controller."""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import Optional, Sequence

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


def project_path_lookahead(
    points: Sequence[tuple[float, float]],
    x: float,
    y: float,
    lookahead_distance: float,
) -> Optional[tuple[tuple[float, float], int, float, float]]:
    """Project a pose onto an ordered path and return a continuous target.

    The planner publishes grid vertices, but the robot moves continuously and
    the planner may republish a path with a slightly different first cell on
    every update.  Selecting ``points[1]`` therefore makes the desired bearing
    depend on grid quantization.  This helper instead finds the closest point
    on the ordered polyline, advances by a lookahead arc length, and
    interpolates the target.  It preserves the route order while removing the
    artificial target jump caused by discrete cell boundaries.

    The return value is ``(target, segment_index, target_arc_length,
    lateral_error)``.  The diagnostics are intentionally returned alongside
    the target so the controller can expose path-association failures without
    changing the control law's inputs.
    """
    if not points:
        return None
    if len(points) == 1:
        target = (float(points[0][0]), float(points[0][1]))
        return target, 0, 0.0, math.hypot(target[0] - x, target[1] - y)

    best_distance_sq = float("inf")
    best_progress = 0.0
    best_segment = 0
    accumulated = 0.0
    total_length = 0.0
    segment_lengths: list[float] = []

    for index in range(len(points) - 1):
        x0, y0 = float(points[index][0]), float(points[index][1])
        x1, y1 = float(points[index + 1][0]), float(points[index + 1][1])
        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        segment_lengths.append(length)
        total_length += length
        if length <= 1.0e-12:
            projection = 0.0
        else:
            projection = clamp(((x - x0) * dx + (y - y0) * dy) / (length * length), 0.0, 1.0)
        projected_x = x0 + projection * dx
        projected_y = y0 + projection * dy
        distance_sq = (x - projected_x) ** 2 + (y - projected_y) ** 2
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_segment = index
            best_progress = accumulated + projection * length
        accumulated += length

    target_progress = min(
        total_length,
        best_progress + max(0.0, float(lookahead_distance)),
    )
    remaining = target_progress
    target_segment = 0
    for index, length in enumerate(segment_lengths):
        if remaining <= length or index == len(segment_lengths) - 1:
            target_segment = index
            fraction = 0.0 if length <= 1.0e-12 else remaining / length
            x0, y0 = float(points[index][0]), float(points[index][1])
            x1, y1 = float(points[index + 1][0]), float(points[index + 1][1])
            target = (
                x0 + fraction * (x1 - x0),
                y0 + fraction * (y1 - y0),
            )
            break
        remaining -= length
    else:
        target_segment = len(points) - 2
        target = (float(points[-1][0]), float(points[-1][1]))

    return (
        target,
        max(best_segment, target_segment),
        target_progress,
        math.sqrt(max(0.0, best_distance_sq)),
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
        # When enabled, a distance threshold alone is not enough to latch the
        # terminal stop: the latest localization match must also be valid.
        # This is explicit rather than inferred from the control-pose topic so
        # A/B experiments cannot silently change the confirmation contract.
        self.declare_parameter("require_localization_match_for_goal", False)
        self.declare_parameter("localization_match_valid_topic", "/localization_match_valid")
        self.declare_parameter("localization_match_status_topic", "/localization_match_status")
        self.declare_parameter("localization_candidate_topic", "/localization_candidate")
        # MCL publishes one Odometry event only for an accepted map candidate.
        # This optional contract lets terminal confirmation count fresh
        # timestamped evidence instead of a repeated status string.
        self.declare_parameter("require_timestamped_localization_evidence", False)
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
        self.declare_parameter("require_goal_reference_for_goal", False)
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
        )
        localization_match_valid_topic = str(
            self.get_parameter("localization_match_valid_topic").value
        )
        localization_match_status_topic = str(
            self.get_parameter("localization_match_status_topic").value
        )
        localization_candidate_topic = str(
            self.get_parameter("localization_candidate_topic").value
        )
        self.require_timestamped_localization_evidence = bool(
            self.get_parameter("require_timestamped_localization_evidence").value
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
        self.require_goal_reference_for_goal = bool(
            self.get_parameter("require_goal_reference_for_goal").value
        )
        self.rotate_in_place_threshold = float(
            self.get_parameter("rotate_in_place_threshold").value
        )
        # A timestamped-evidence contract must never be allowed to wait
        # forever.  ``goal_confirmation_timeout_s=0`` remains the legacy
        # behavior when no independent evidence is requested, but under the
        # evidence contract it means "derive a bounded confirmation window"
        # from the existing control semantics.  This prevents the controller
        # from holding zero velocity indefinitely when the localizer is
        # temporarily ambiguous or unavailable.
        publish_rate = max(
            1.0,
            float(self.get_parameter("publish_rate_hz").value),
        )
        self.effective_goal_confirmation_timeout_s = (
            self.goal_confirmation_timeout_s
        )
        if (
            self.require_timestamped_localization_evidence
            and self.effective_goal_confirmation_timeout_s <= 0.0
        ):
            self.effective_goal_confirmation_timeout_s = max(
                1.0,
                2.0 * self.goal_confirmation_samples / publish_rate,
            )

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
        self.localization_candidate_subscription = None
        if self.require_timestamped_localization_evidence:
            self.localization_candidate_subscription = self.create_subscription(
                Odometry,
                localization_candidate_topic,
                self.localization_candidate_callback,
                qos_profile_sensor_data,
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
        self.localization_candidate_events_since_entry = 0
        self.latest_localization_candidate_stamp_s: Optional[float] = None
        self.latest_localization_candidate_received_at: Optional[float] = None
        self.localization_candidate_sequence = 0
        self.localization_candidate_sequence_at_entry = 0
        self.latest_goal_reference: Optional[Odometry] = None
        self.goal_confirmation_count = 0
        self.goal_confirmation_started_at: Optional[float] = None
        self.goal_confirmation_timeout_count = 0
        self.final_approach_reentry_count = 0
        self.final_approach_has_left_tolerance = False
        self.final_approach_heading: Optional[float] = None
        # While rotating at a grid corner, keep the committed turn heading
        # stable across asynchronous planner updates.  Without this small
        # state machine, a new A* prefix can change the first cell from left
        # to right while the robot is still turning, producing visible
        # back-and-forth motion.
        self.turn_target_heading: Optional[float] = None
        self.path_target_segment_index = -1
        self.path_target_progress_m = 0.0
        self.path_lateral_error_m = 0.0
        # A confirmation timeout must not authorize unbounded blind motion.
        # The budget is derived from the existing terminal tolerance so this
        # guard is a safety invariant, not another experiment parameter.
        self.final_approach_distance_budget_m = max(self.goal_tolerance, 0.0)
        self.final_approach_distance_used_m = 0.0
        self.last_control_time_s: Optional[float] = None
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
        candidate_age = (
            None
            if self.latest_localization_candidate_received_at is None
            else max(
                0.0,
                time.monotonic()
                - self.latest_localization_candidate_received_at,
            )
        )
        candidate_age_text = "" if candidate_age is None else f"{candidate_age:.9f}"
        candidate_stamp_text = (
            ""
            if self.latest_localization_candidate_stamp_s is None
            else f"{self.latest_localization_candidate_stamp_s:.9f}"
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
            f"localization_candidate_events_since_entry={self.localization_candidate_events_since_entry};"
            f"localization_candidate_stamp_s={candidate_stamp_text};"
            f"localization_candidate_receipt_age_s={candidate_age_text};"
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
        if self.require_timestamped_localization_evidence:
            self.get_logger().info(
                "Goal confirmation also requires fresh timestamped accepted "
                "localization candidates after entering the goal tolerance."
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
        if self.effective_goal_confirmation_timeout_s > 0.0:
            timeout_origin = (
                "configured"
                if self.goal_confirmation_timeout_s > 0.0
                else "derived from confirmation samples and control rate"
            )
            self.get_logger().info(
                "Goal confirmation timeout is "
                f"{self.effective_goal_confirmation_timeout_s:.3f} s "
                f"({timeout_origin}); timeout is followed by bounded "
                "FINAL_APPROACH recovery."
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
                self.turn_target_heading = None
                self.final_approach_distance_used_m = 0.0
                self.set_goal_state(GoalControlState.APPROACHING)
                self.localization_accepted_events_since_entry = 0
                self.localization_candidate_events_since_entry = 0
                self.localization_candidate_sequence_at_entry = (
                    self.localization_candidate_sequence
                )
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
                    self.turn_target_heading = None
                    self.final_approach_distance_used_m = 0.0
                    self.set_goal_state(GoalControlState.APPROACHING)
                    self.localization_accepted_events_since_entry = 0
                    self.localization_candidate_events_since_entry = 0
                    self.localization_candidate_sequence_at_entry = (
                        self.localization_candidate_sequence
                    )
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
            self.turn_target_heading = None
            self.final_approach_distance_used_m = 0.0
            self.set_goal_state(GoalControlState.APPROACHING)
            self.localization_accepted_events_since_entry = 0
            self.localization_candidate_events_since_entry = 0
            self.localization_candidate_sequence_at_entry = (
                self.localization_candidate_sequence
            )
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
            # A rejected matcher update breaks a run of consecutive accepted
            # status messages, but it does not invalidate accepted candidate
            # events that were already received after entering confirmation.
            # These are separate contracts: the status stream describes the
            # latest matcher result, while /localization_candidate is an
            # event stream of independently accepted, timestamped evidence.
            # Clearing both here made valid MCL events disappear whenever an
            # accepted update was followed by an ordinary rejected update.
            self.localization_accepted_events_since_entry = 0

    def localization_candidate_callback(self, message: Odometry) -> None:
        """Record one accepted, timestamped map-localization candidate.

        MCL publishes this event only after an update passes its gates.  The
        monotonic receipt time is used for freshness because simulator header
        stamps and node/system time need not share an epoch.  The source stamp
        remains available for offline event correlation.
        """
        stamp = self.stamp_seconds(message)
        if not math.isfinite(stamp):
            return
        if (
            self.latest_localization_candidate_stamp_s is not None
            and stamp <= self.latest_localization_candidate_stamp_s
        ):
            return
        self.latest_localization_candidate_stamp_s = stamp
        self.latest_localization_candidate_received_at = time.monotonic()
        self.localization_candidate_sequence += 1
        if self.goal_state == GoalControlState.CONFIRMING:
            if (
                self.localization_candidate_sequence
                > self.localization_candidate_sequence_at_entry
            ):
                self.localization_candidate_events_since_entry += 1

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

    def select_target(self, x: float, y: float) -> Optional[tuple[float, float]]:
        """Select a continuous lookahead target on the ordered path.

        The target is associated with the robot's closest point on the current
        path, then advanced by arc length.  This prevents a replan from
        resetting the controller to a discrete first cell that is already
        behind the continuous robot pose.
        """
        if self.latest_path is None or not self.latest_path.poses:
            return None

        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in self.latest_path.poses
        ]
        projected = project_path_lookahead(
            points,
            x,
            y,
            self.lookahead_distance,
        )
        if projected is None:
            return None
        target, segment_index, progress_m, lateral_error_m = projected
        self.path_target_segment_index = segment_index
        self.path_target_progress_m = progress_m
        self.path_lateral_error_m = lateral_error_m
        return target

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
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        if self.last_control_time_s is None:
            control_dt_s = 0.0
        else:
            control_dt_s = min(0.2, max(0.0, now_s - self.last_control_time_s))
        self.last_control_time_s = now_s
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
                self.localization_candidate_events_since_entry = 0
                self.localization_candidate_sequence_at_entry = (
                    self.localization_candidate_sequence
                )
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
                self.localization_candidate_events_since_entry = 0
                self.localization_candidate_sequence_at_entry = (
                    self.localization_candidate_sequence
                )
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
                    self.effective_goal_confirmation_timeout_s > 0.0
                    and confirmation_elapsed is not None
                    and confirmation_elapsed
                    >= self.effective_goal_confirmation_timeout_s
                ):
                    self.goal_confirmation_count = 0
                    self.localization_accepted_events_since_entry = 0
                    self.localization_candidate_events_since_entry = 0
                    self.localization_candidate_sequence_at_entry = (
                        self.localization_candidate_sequence
                    )
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
                        self.localization_candidate_events_since_entry = 0
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
                        self.localization_candidate_events_since_entry = 0
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
                            self.localization_candidate_events_since_entry = 0
                            self.report_state(
                                "At estimated goal, waiting for a valid "
                                "localization match."
                            )
                            self.publish_stop()
                            return
                        if self.localization_match_status != "accepted":
                            self.goal_confirmation_count = 0
                            self.localization_accepted_events_since_entry = 0
                            self.localization_candidate_events_since_entry = 0
                            status = self.localization_match_status or "not_received"
                            self.report_state(
                                "At estimated goal, waiting for a fresh accepted "
                                f"localization match; status={status}."
                            )
                            self.publish_stop()
                            return
                    if self.require_timestamped_localization_evidence:
                        candidate_age = (
                            None
                            if self.latest_localization_candidate_received_at is None
                            else (
                                time.monotonic()
                                - self.latest_localization_candidate_received_at
                            )
                        )
                        if (
                            self.localization_candidate_events_since_entry
                            < self.goal_confirmation_samples
                            or candidate_age is None
                            or candidate_age > self.goal_confirmation_max_pose_age_s
                        ):
                            self.goal_confirmation_count = 0
                            candidate_age_text = (
                                "unavailable"
                                if candidate_age is None
                                else f"{max(0.0, candidate_age):.3f} s"
                            )
                            self.report_state(
                                "At estimated goal, waiting for fresh "
                                "timestamped localization evidence; "
                                f"accepted={self.localization_candidate_events_since_entry}/"
                                f"{self.goal_confirmation_samples}, "
                                f"age={candidate_age_text}."
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
            self.localization_candidate_events_since_entry = 0
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
            self.turn_target_heading = None
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
        elif final_approach:
            target_heading = math.atan2(target_y - position.y, target_x - position.x)
        else:
            desired_heading = math.atan2(
                target_y - position.y,
                target_x - position.x,
            )
            # Commit to a large turn until the robot has actually aligned with
            # that heading.  A* can publish a new path while the robot is
            # rotating; recomputing the first-cell bearing on every callback
            # otherwise allows the turn direction to change mid-rotation.
            if self.turn_target_heading is not None:
                committed_error = wrap_angle(self.turn_target_heading - yaw)
                if abs(committed_error) <= self.heading_deadband:
                    self.turn_target_heading = None
            if self.turn_target_heading is None:
                desired_error = wrap_angle(desired_heading - yaw)
                if abs(desired_error) > self.rotate_in_place_threshold:
                    self.turn_target_heading = desired_heading
            target_heading = (
                self.turn_target_heading
                if self.turn_target_heading is not None
                else desired_heading
            )
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
        committed_turn = (
            not final_approach and self.turn_target_heading is not None
        )
        if not committed_turn and abs(heading_error) <= self.rotate_in_place_threshold:
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

        if self.goal_state == GoalControlState.FINAL_APPROACH:
            remaining_budget_m = (
                self.final_approach_distance_budget_m
                - self.final_approach_distance_used_m
            )
            if remaining_budget_m <= 1.0e-6:
                self.set_goal_state(GoalControlState.GOAL_UNCONFIRMED)
                self.get_logger().warning(
                    "Final-approach distance budget exhausted; stopping "
                    "without declaring success."
                )
                self.publish_goal_event("final_approach_budget_exhausted", goal_distance)
                self.publish_stop()
                return
            if command.linear.x > 0.0 and control_dt_s > 0.0:
                command.linear.x = min(
                    command.linear.x,
                    remaining_budget_m / control_dt_s,
                )
                self.final_approach_distance_used_m += (
                    command.linear.x * control_dt_s
                )

        self.report_state(
            f"Tracking {'final goal' if final_approach else 'path'}; "
            f"target=({target_x:.2f}, {target_y:.2f}), "
            f"segment={self.path_target_segment_index}, "
            f"path_s={self.path_target_progress_m:.2f}, "
            f"lateral={self.path_lateral_error_m:.3f}, "
            f"turn={'committed' if committed_turn else 'free'}, "
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
