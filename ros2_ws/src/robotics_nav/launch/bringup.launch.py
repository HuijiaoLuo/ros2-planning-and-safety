from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    evaluation_output = LaunchConfiguration("evaluation_output")
    trace_output = LaunchConfiguration("trace_output")
    plan_output = LaunchConfiguration("plan_output")
    map_output = LaunchConfiguration("map_output")
    minimum_clearance = LaunchConfiguration("minimum_clearance")
    sensor_latency = LaunchConfiguration("sensor_latency")
    safety_margin = LaunchConfiguration("safety_margin")
    recovery_timeout_s = LaunchConfiguration("recovery_timeout_s")
    planning_radius_m = LaunchConfiguration("planning_radius_m")
    experiment_timeout_s = LaunchConfiguration("experiment_timeout_s")
    scan_delay_s = LaunchConfiguration("scan_delay_s")
    scan_noise_std_m = LaunchConfiguration("scan_noise_std_m")
    scan_noise_seed = LaunchConfiguration("scan_noise_seed")
    navigation_pose_topic = LaunchConfiguration("navigation_pose_topic")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    estimation_output = LaunchConfiguration("estimation_output")
    estimation_trace_output = LaunchConfiguration("estimation_trace_output")
    imu_gyro_bias_rad_s = LaunchConfiguration("imu_gyro_bias_rad_s")
    imu_gyro_noise_std_rad_s = LaunchConfiguration("imu_gyro_noise_std_rad_s")
    imu_gyro_noise_seed = LaunchConfiguration("imu_gyro_noise_seed")
    wheel_slip_ratio = LaunchConfiguration("wheel_slip_ratio")
    wheel_slip_noise_std = LaunchConfiguration("wheel_slip_noise_std")
    position_mode = LaunchConfiguration("position_mode")
    localization_output_topic = LaunchConfiguration("localization_output_topic")
    localization_diagnostic_output = LaunchConfiguration(
        "localization_diagnostic_output"
    )
    localization_search_radius_m = LaunchConfiguration("localization_search_radius_m")
    localization_search_step_m = LaunchConfiguration("localization_search_step_m")
    localization_scan_stride = LaunchConfiguration("localization_scan_stride")
    localization_score_mode = LaunchConfiguration("localization_score_mode")
    localization_optimizer_mode = LaunchConfiguration("localization_optimizer_mode")
    localization_refine_top_k = LaunchConfiguration("localization_refine_top_k")
    localization_publish_rate_hz = LaunchConfiguration(
        "localization_publish_rate_hz"
    )
    localization_match_rate_hz = LaunchConfiguration(
        "localization_match_rate_hz"
    )
    localization_max_correction_m = LaunchConfiguration(
        "localization_max_correction_m"
    )
    localization_max_total_correction_m = LaunchConfiguration(
        "localization_max_total_correction_m"
    )
    localization_robot_radius_m = LaunchConfiguration(
        "localization_robot_radius_m"
    )
    localization_max_match_score_m = LaunchConfiguration(
        "localization_max_match_score_m"
    )
    localization_correction_smoothing = LaunchConfiguration(
        "localization_correction_smoothing"
    )
    localization_minimum_consecutive_matches = LaunchConfiguration(
        "localization_minimum_consecutive_matches"
    )
    localization_candidate_consistency_m = LaunchConfiguration(
        "localization_candidate_consistency_m"
    )
    localization_minimum_reapplication_change_m = LaunchConfiguration(
        "localization_minimum_reapplication_change_m"
    )
    localization_max_match_age_s = LaunchConfiguration(
        "localization_max_match_age_s"
    )
    localization_max_odom_motion_during_match_m = LaunchConfiguration(
        "localization_max_odom_motion_during_match_m"
    )
    localization_max_odom_yaw_change_during_match_rad = LaunchConfiguration(
        "localization_max_odom_yaw_change_during_match_rad"
    )
    localization_map_frame_id = LaunchConfiguration("localization_map_frame_id")
    localization_odom_frame_id = LaunchConfiguration("localization_odom_frame_id")
    localization_broadcast_map_odom_tf = LaunchConfiguration(
        "localization_broadcast_map_odom_tf"
    )
    localization_yaw_search_radius_rad = LaunchConfiguration(
        "localization_yaw_search_radius_rad"
    )
    localization_yaw_search_step_rad = LaunchConfiguration(
        "localization_yaw_search_step_rad"
    )
    localization_yaw_prior_weight = LaunchConfiguration(
        "localization_yaw_prior_weight"
    )
    localization_max_heading_correction_rad = LaunchConfiguration(
        "localization_max_heading_correction_rad"
    )
    localization_minimum_score_improvement_m = LaunchConfiguration(
        "localization_minimum_score_improvement_m"
    )
    wheel_weight = LaunchConfiguration("wheel_weight")
    fusion_mode = LaunchConfiguration("fusion_mode")
    gyro_rate_noise_std_rad_s = LaunchConfiguration("gyro_rate_noise_std_rad_s")
    wheel_yaw_noise_std_rad = LaunchConfiguration("wheel_yaw_noise_std_rad")
    gyro_bias_mode = LaunchConfiguration("gyro_bias_mode")
    initial_gyro_bias_rad_s = LaunchConfiguration("initial_gyro_bias_rad_s")
    gyro_bias_random_walk_std_rad_s2 = LaunchConfiguration(
        "gyro_bias_random_walk_std_rad_s2"
    )
    wheel_yaw_bias_random_walk_std_rad_sqrt_s = LaunchConfiguration(
        "wheel_yaw_bias_random_walk_std_rad_sqrt_s"
    )
    initial_position_variance_m2 = LaunchConfiguration(
        "initial_position_variance_m2"
    )
    initial_heading_variance_rad2 = LaunchConfiguration(
        "initial_heading_variance_rad2"
    )
    initial_bias_variance_rad2_s2 = LaunchConfiguration(
        "initial_bias_variance_rad2_s2"
    )
    initial_wheel_yaw_bias_variance_rad2 = LaunchConfiguration(
        "initial_wheel_yaw_bias_variance_rad2"
    )
    adaptive_wheel_noise = LaunchConfiguration("adaptive_wheel_noise")
    wheel_yaw_noise_min_std_rad = LaunchConfiguration(
        "wheel_yaw_noise_min_std_rad"
    )
    wheel_yaw_noise_max_std_rad = LaunchConfiguration(
        "wheel_yaw_noise_max_std_rad"
    )
    wheel_noise_adaptation_rate = LaunchConfiguration(
        "wheel_noise_adaptation_rate"
    )
    wheel_speed_noise_std_m_s = LaunchConfiguration("wheel_speed_noise_std_m_s")
    nis_gate_threshold = LaunchConfiguration("nis_gate_threshold")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "evaluation_output",
                default_value="",
                description="Optional CSV path for closed-loop evaluation metrics.",
            ),
            DeclareLaunchArgument(
                "trace_output",
                default_value="",
                description="Optional CSV path for per-sample trajectory trace.",
            ),
            DeclareLaunchArgument(
                "plan_output",
                default_value="",
                description="Optional CSV path for the final planned path.",
            ),
            DeclareLaunchArgument(
                "map_output",
                default_value="",
                description="Optional CSV path for occupied map cells.",
            ),
            DeclareLaunchArgument(
                "collision_topic",
                default_value="/collision/contacts",
                description="ROS topic carrying Gazebo contact messages.",
            ),
            DeclareLaunchArgument(
                "minimum_clearance",
                default_value="0.50",
                description="Minimum front clearance enforced by the safety supervisor.",
            ),
            DeclareLaunchArgument(
                "sensor_latency",
                default_value="0.10",
                description="Assumed perception/control latency in seconds.",
            ),
            DeclareLaunchArgument(
                "scan_delay_s",
                default_value="0.0",
                description="Artificial delay before the safety supervisor uses a LiDAR scan.",
            ),
            DeclareLaunchArgument(
                "scan_noise_std_m",
                default_value="0.0",
                description="Gaussian LiDAR range-noise standard deviation in metres.",
            ),
            DeclareLaunchArgument(
                "scan_noise_seed",
                default_value="0",
                description="Random seed for reproducible LiDAR range noise.",
            ),
            DeclareLaunchArgument(
                "safety_margin",
                default_value="0.15",
                description="Additional stopping-distance safety margin in metres.",
            ),
            DeclareLaunchArgument(
                "recovery_timeout_s",
                default_value="8.0",
                description="Maximum continuous safety-recovery time in seconds.",
            ),
            DeclareLaunchArgument(
                "planning_radius_m",
                default_value="0.35",
                description="Obstacle-inflation radius used by the global planner.",
            ),
            DeclareLaunchArgument(
                "experiment_timeout_s",
                default_value="0.0",
                description="Optional total evaluation timeout; zero disables it.",
            ),
            DeclareLaunchArgument(
                "navigation_pose_topic",
                default_value="/odom",
                description=(
                    "Pose topic consumed by navigation and safety nodes. "
                    "The default /odom preserves the ideal V2 baseline."
                ),
            ),
            DeclareLaunchArgument(
                "goal_tolerance",
                default_value="0.05",
                description=(
                    "Distance in metres used by both the path follower and the "
                    "evaluation logger to declare the goal reached."
                ),
            ),
            DeclareLaunchArgument(
                "estimation_output",
                default_value="",
                description="Optional CSV path for V3 estimator error diagnostics.",
            ),
            DeclareLaunchArgument(
                "estimation_trace_output",
                default_value="",
                description=(
                    "Optional per-sample CSV path for estimator covariance "
                    "calibration; ground truth remains logger-only."
                ),
            ),
            DeclareLaunchArgument(
                "imu_gyro_bias_rad_s",
                default_value="0.0",
                description="Synthetic gyro bias used by the V3 estimator.",
            ),
            DeclareLaunchArgument(
                "imu_gyro_noise_std_rad_s",
                default_value="0.0",
                description="Synthetic gyro white-noise standard deviation.",
            ),
            DeclareLaunchArgument(
                "imu_gyro_noise_seed",
                default_value="0",
                description="Seed for reproducible synthetic gyro noise.",
            ),
            DeclareLaunchArgument(
                "wheel_slip_ratio",
                default_value="0.0",
                description=(
                    "Estimator-side fraction of wheel-odometry translation "
                    "lost to longitudinal slip."
                ),
            ),
            DeclareLaunchArgument(
                "wheel_slip_noise_std",
                default_value="0.0",
                description=(
                    "Standard deviation of the configured wheel-slip fraction "
                    "used in EKF process covariance."
                ),
            ),
            DeclareLaunchArgument(
                "position_mode",
                default_value="wheel_pose",
                description=(
                    "Estimator position mode: wheel_pose or propagated "
                    "differential-drive integration."
                ),
            ),
            DeclareLaunchArgument(
                "localization_output_topic",
                default_value="/localized_estimate",
                description="Output topic for the optional LiDAR-map corrected pose.",
            ),
            DeclareLaunchArgument(
                "localization_diagnostic_output",
                default_value="",
                description=(
                    "Optional JSONL file containing LiDAR matching inputs and "
                    "gate decisions for offline replay."
                ),
            ),
            DeclareLaunchArgument(
                "localization_search_radius_m",
                default_value="0.35",
                description="Local x/y search radius for LiDAR-map matching.",
            ),
            DeclareLaunchArgument(
                "localization_search_step_m",
                default_value="0.025",
                description="Grid step for the local LiDAR-map search.",
            ),
            DeclareLaunchArgument(
                "localization_scan_stride",
                default_value="6",
                description="Use every Nth LiDAR ray for map matching.",
            ),
            DeclareLaunchArgument(
                "localization_score_mode",
                default_value="range",
                description=(
                    "LiDAR-map score model: range preserves the baseline; "
                    "endpoint scores measured endpoints against occupied "
                    "cells; boundary scores against continuous "
                    "occupied/free boundaries."
                ),
            ),
            DeclareLaunchArgument(
                "localization_optimizer_mode",
                default_value="grid",
                description=(
                    "Matcher search: grid preserves the baseline; "
                    "coarse_to_fine refines the best deterministic basins."
                ),
            ),
            DeclareLaunchArgument(
                "localization_refine_top_k",
                default_value="5",
                description="Number of coarse basins refined in coarse_to_fine mode.",
            ),
            DeclareLaunchArgument(
                "localization_publish_rate_hz",
                default_value="30.0",
                description="High-rate localized pose publication for navigation.",
            ),
            DeclareLaunchArgument(
                "localization_match_rate_hz",
                default_value="2.0",
                description=(
                    "Background LiDAR-map matching rate.  Matching is kept "
                    "separate from pose publication so it cannot block control."
                ),
            ),
            DeclareLaunchArgument(
                "localization_max_correction_m",
                default_value="0.15",
                description=(
                    "Reject a LiDAR-map match if its position correction "
                    "exceeds this distance."
                ),
            ),
            DeclareLaunchArgument(
                "localization_max_total_correction_m",
                default_value="0.20",
                description=(
                    "Maximum accumulated translation of the persistent "
                    "map-to-odom correction from its initial value."
                ),
            ),
            DeclareLaunchArgument(
                "localization_robot_radius_m",
                default_value="0.35",
                description=(
                    "Robot-centre clearance radius used to reject LiDAR "
                    "candidates inside the planner's inflated obstacles."
                ),
            ),
            DeclareLaunchArgument(
                "localization_max_match_score_m",
                default_value="0.12",
                description=(
                    "Reject a LiDAR-map match if its range residual exceeds "
                    "this score threshold."
                ),
            ),
            DeclareLaunchArgument(
                "localization_correction_smoothing",
                default_value="0.25",
                description=(
                    "Fraction of an accepted LiDAR-map correction applied per "
                    "update."
                ),
            ),
            DeclareLaunchArgument(
                "localization_minimum_consecutive_matches",
                default_value="3",
                description=(
                    "Number of consecutive consistent matches required before "
                    "a correction is applied."
                ),
            ),
            DeclareLaunchArgument(
                "localization_candidate_consistency_m",
                default_value="0.05",
                description=(
                    "Maximum change in candidate correction between valid "
                    "matches."
                ),
            ),
            DeclareLaunchArgument(
                "localization_minimum_reapplication_change_m",
                default_value="0.05",
                description=(
                    "Minimum change in a candidate correction before an "
                    "external LiDAR correction may be applied again."
                ),
            ),
            DeclareLaunchArgument(
                "localization_max_match_age_s",
                default_value="1.0",
                description=(
                    "Reject a match result if it has spent longer than this "
                    "time in the asynchronous worker pipeline."
                ),
            ),
            DeclareLaunchArgument(
                "localization_max_odom_motion_during_match_m",
                default_value="0.10",
                description=(
                    "Reject a match if live odometry moved farther than this "
                    "during its asynchronous computation."
                ),
            ),
            DeclareLaunchArgument(
                "localization_max_odom_yaw_change_during_match_rad",
                default_value="0.35",
                description=(
                    "Reject a match if live odometry rotated farther than "
                    "this during its asynchronous computation."
                ),
            ),
            DeclareLaunchArgument(
                "localization_map_frame_id",
                default_value="map",
                description="Parent frame for the persistent map-to-odom correction.",
            ),
            DeclareLaunchArgument(
                "localization_odom_frame_id",
                default_value="odom",
                description="Child frame for the persistent map-to-odom correction.",
            ),
            DeclareLaunchArgument(
                "localization_broadcast_map_odom_tf",
                default_value="true",
                description="Broadcast the persistent map-to-odom correction on TF.",
            ),
            DeclareLaunchArgument(
                "localization_yaw_search_radius_rad",
                default_value="0.0",
                description=(
                    "Optional diagnostic heading search; 0 keeps the fused "
                    "heading fixed because only x/y correction is applied."
                ),
            ),
            DeclareLaunchArgument(
                "localization_yaw_search_step_rad",
                default_value="0.05",
                description="Heading search step for LiDAR-map matching.",
            ),
            DeclareLaunchArgument(
                "localization_yaw_prior_weight",
                default_value="0.02",
                description="Prior penalty on heading displacement in matching.",
            ),
            DeclareLaunchArgument(
                "localization_max_heading_correction_rad",
                default_value="0.25",
                description="Maximum accepted LiDAR-map heading correction.",
            ),
            DeclareLaunchArgument(
                "localization_minimum_score_improvement_m",
                default_value="0.005",
                description=(
                    "Minimum reduction in mean scan residual required before "
                    "applying a candidate correction."
                ),
            ),
            DeclareLaunchArgument(
                "wheel_weight",
                default_value="0.02",
                description=(
                    "Complementary-fusion correction weight applied to wheel yaw."
                ),
            ),
            DeclareLaunchArgument(
                "fusion_mode",
                default_value="fixed",
                description="Fusion mode: fixed, adaptive heading, or full pose EKF.",
            ),
            DeclareLaunchArgument(
                "gyro_rate_noise_std_rad_s",
                default_value="0.01",
                description="Gyro rate standard deviation used by adaptive fusion.",
            ),
            DeclareLaunchArgument(
                "wheel_yaw_noise_std_rad",
                default_value="0.07",
                description="Wheel yaw standard deviation used by adaptive fusion.",
            ),
            DeclareLaunchArgument(
                "gyro_bias_random_walk_std_rad_s2",
                default_value="0.001",
                description="Gyro-bias random-walk standard deviation.",
            ),
            DeclareLaunchArgument(
                "gyro_bias_mode",
                default_value="estimated",
                description="Pose-EKF gyro-bias mode: estimated or fixed.",
            ),
            DeclareLaunchArgument(
                "initial_gyro_bias_rad_s",
                default_value="0.0",
                description=(
                    "Pre-calibrated gyro bias used in fixed-bias mode."
                ),
            ),
            DeclareLaunchArgument(
                "wheel_yaw_bias_random_walk_std_rad_sqrt_s",
                default_value="0.001",
                description=(
                    "Wheel-yaw bias random-walk standard deviation in "
                    "rad/sqrt(s)."
                ),
            ),
            DeclareLaunchArgument(
                "initial_position_variance_m2",
                default_value="0.25",
                description="Initial x/y variance for the pose EKF in m^2.",
            ),
            DeclareLaunchArgument(
                "initial_heading_variance_rad2",
                default_value="0.25",
                description="Initial heading variance for adaptive fusion.",
            ),
            DeclareLaunchArgument(
                "initial_bias_variance_rad2_s2",
                default_value="0.01",
                description="Initial gyro-bias variance for adaptive fusion.",
            ),
            DeclareLaunchArgument(
                "initial_wheel_yaw_bias_variance_rad2",
                default_value="0.01",
                description="Initial variance of the wheel-yaw bias state in rad^2.",
            ),
            DeclareLaunchArgument(
                "adaptive_wheel_noise",
                default_value="false",
                description=(
                    "Adapt wheel-yaw measurement noise from the innovation "
                    "sequence in adaptive fusion."
                ),
            ),
            DeclareLaunchArgument(
                "wheel_yaw_noise_min_std_rad",
                default_value="0.02",
                description="Lower bound for adaptive wheel-yaw noise standard deviation.",
            ),
            DeclareLaunchArgument(
                "wheel_yaw_noise_max_std_rad",
                default_value="0.20",
                description="Upper bound for adaptive wheel-yaw noise standard deviation.",
            ),
            DeclareLaunchArgument(
                "wheel_noise_adaptation_rate",
                default_value="0.05",
                description="Exponential update rate for adaptive wheel-yaw noise.",
            ),
            DeclareLaunchArgument(
                "wheel_speed_noise_std_m_s",
                default_value="0.02",
                description="Wheel forward-speed measurement noise standard deviation.",
            ),
            DeclareLaunchArgument(
                "nis_gate_threshold",
                default_value="9.0",
                description=(
                    "One-dimensional NIS threshold for rejecting implausible "
                    "wheel-yaw EKF updates; zero disables the gate."
                ),
            ),
            Node(
                package="robotics_nav",
                executable="heading_estimator",
                name="heading_estimator",
                output="screen",
                parameters=[
                    {
                        "wheel_odom_topic": "/wheel_odom",
                        "imu_topic": "/imu",
                        "output_topic": "/state_estimate",
                        "fusion_mode": fusion_mode,
                        "wheel_weight": wheel_weight,
                        "gyro_rate_noise_std_rad_s": gyro_rate_noise_std_rad_s,
                        "wheel_yaw_noise_std_rad": wheel_yaw_noise_std_rad,
                        "gyro_bias_random_walk_std_rad_s2": gyro_bias_random_walk_std_rad_s2,
                        "gyro_bias_mode": gyro_bias_mode,
                        "initial_gyro_bias_rad_s": initial_gyro_bias_rad_s,
                        "wheel_yaw_bias_random_walk_std_rad_sqrt_s": wheel_yaw_bias_random_walk_std_rad_sqrt_s,
                        "initial_position_variance_m2": initial_position_variance_m2,
                        "initial_heading_variance_rad2": initial_heading_variance_rad2,
                        "initial_bias_variance_rad2_s2": initial_bias_variance_rad2_s2,
                        "initial_wheel_yaw_bias_variance_rad2": initial_wheel_yaw_bias_variance_rad2,
                        "adaptive_wheel_noise": adaptive_wheel_noise,
                        "wheel_yaw_noise_min_std_rad": wheel_yaw_noise_min_std_rad,
                        "wheel_yaw_noise_max_std_rad": wheel_yaw_noise_max_std_rad,
                        "wheel_noise_adaptation_rate": wheel_noise_adaptation_rate,
                        "wheel_speed_noise_std_m_s": wheel_speed_noise_std_m_s,
                        "wheel_slip_noise_std": wheel_slip_noise_std,
                        "nis_gate_threshold": nis_gate_threshold,
                        "imu_gyro_bias_rad_s": imu_gyro_bias_rad_s,
                        "imu_gyro_noise_std_rad_s": imu_gyro_noise_std_rad_s,
                        "imu_gyro_noise_seed": imu_gyro_noise_seed,
                        "wheel_slip_ratio": wheel_slip_ratio,
                        "position_mode": position_mode,
                    }
                ],
            ),
            Node(
                package="robotics_nav",
                executable="estimation_logger",
                name="estimation_logger",
                output="screen",
                parameters=[
                    {
                        "ground_truth_topic": "/odom",
                        "wheel_odom_topic": "/wheel_odom",
                        "estimate_topic": "/state_estimate",
                        "imu_topic": "/imu",
                        "output_path": estimation_output,
                        "trace_output_path": estimation_trace_output,
                        "sample_rate_hz": 20.0,
                        "configured_imu_gyro_bias_rad_s": imu_gyro_bias_rad_s,
                        "configured_imu_gyro_noise_std_rad_s": imu_gyro_noise_std_rad_s,
                        "configured_imu_gyro_noise_seed": imu_gyro_noise_seed,
                        "configured_wheel_slip_ratio": wheel_slip_ratio,
                        "configured_wheel_slip_noise_std": wheel_slip_noise_std,
                        "configured_position_mode": position_mode,
                        "configured_wheel_weight": wheel_weight,
                        "configured_fusion_mode": fusion_mode,
                        "configured_gyro_rate_noise_std_rad_s": gyro_rate_noise_std_rad_s,
                        "configured_wheel_yaw_noise_std_rad": wheel_yaw_noise_std_rad,
                        "configured_gyro_bias_random_walk_std_rad_s2": gyro_bias_random_walk_std_rad_s2,
                        "configured_gyro_bias_mode": gyro_bias_mode,
                        "configured_initial_gyro_bias_rad_s": initial_gyro_bias_rad_s,
                        "configured_wheel_yaw_bias_random_walk_std_rad_sqrt_s": wheel_yaw_bias_random_walk_std_rad_sqrt_s,
                        "configured_initial_position_variance_m2": initial_position_variance_m2,
                        "configured_initial_heading_variance_rad2": initial_heading_variance_rad2,
                        "configured_initial_bias_variance_rad2_s2": initial_bias_variance_rad2_s2,
                        "configured_initial_wheel_yaw_bias_variance_rad2": initial_wheel_yaw_bias_variance_rad2,
                        "configured_adaptive_wheel_noise": adaptive_wheel_noise,
                        "configured_wheel_yaw_noise_min_std_rad": wheel_yaw_noise_min_std_rad,
                        "configured_wheel_yaw_noise_max_std_rad": wheel_yaw_noise_max_std_rad,
                        "configured_wheel_noise_adaptation_rate": wheel_noise_adaptation_rate,
                        "configured_wheel_speed_noise_std_m_s": wheel_speed_noise_std_m_s,
                        "configured_nis_gate_threshold": nis_gate_threshold,
                        "wheel_yaw_bias_estimate_topic": "/wheel_yaw_bias_estimate",
                    }
                ],
            ),
            Node(
                package="robotics_nav",
                executable="lidar_localizer",
                name="lidar_localizer",
                output="screen",
                parameters=[
                    {
                        "input_pose_topic": "/state_estimate",
                        "output_pose_topic": localization_output_topic,
                        "diagnostic_output": localization_diagnostic_output,
                        "scan_topic": "/scan",
                        "map_topic": "/map",
                        "search_radius_m": localization_search_radius_m,
                        "search_step_m": localization_search_step_m,
                        "scan_stride": localization_scan_stride,
                        "score_mode": localization_score_mode,
                        "optimizer_mode": localization_optimizer_mode,
                        "refine_top_k": localization_refine_top_k,
                        "max_correction_m": localization_max_correction_m,
                        "max_total_correction_m": localization_max_total_correction_m,
                        "robot_radius_m": localization_robot_radius_m,
                        "max_match_score_m": localization_max_match_score_m,
                        "correction_smoothing": localization_correction_smoothing,
                        "minimum_consecutive_matches": localization_minimum_consecutive_matches,
                        "candidate_consistency_m": localization_candidate_consistency_m,
                        "minimum_reapplication_change_m": localization_minimum_reapplication_change_m,
                        "max_match_age_s": localization_max_match_age_s,
                        "max_odom_motion_during_match_m": localization_max_odom_motion_during_match_m,
                        "max_odom_yaw_change_during_match_rad": localization_max_odom_yaw_change_during_match_rad,
                        "map_frame_id": localization_map_frame_id,
                        "odom_frame_id": localization_odom_frame_id,
                        "broadcast_map_odom_tf": localization_broadcast_map_odom_tf,
                        "yaw_search_radius_rad": localization_yaw_search_radius_rad,
                        "yaw_search_step_rad": localization_yaw_search_step_rad,
                        "yaw_prior_weight": localization_yaw_prior_weight,
                        "max_heading_correction_rad": localization_max_heading_correction_rad,
                        "minimum_score_improvement_m": localization_minimum_score_improvement_m,
                        "publish_rate_hz": localization_publish_rate_hz,
                        "match_rate_hz": localization_match_rate_hz,
                    }
                ],
            ),
            Node(
                package="robotics_nav",
                executable="path_follower",
                name="path_follower",
                output="screen",
                parameters=[
                    {
                        "lookahead_distance": 0.10,
                        "max_linear_speed": 0.20,
                        "goal_tolerance": goal_tolerance,
                        "rotate_in_place_threshold": 0.5235987756,
                        "final_approach_distance": 0.60,
                        "final_approach_heading_gain": 1.0,
                        "final_approach_max_angular_speed": 0.60,
                        "heading_deadband": 0.03,
                        "odom_topic": navigation_pose_topic,
                    }
                ],
            ),
            Node(
                package="robotics_nav",
                executable="safety_supervisor",
                name="safety_supervisor",
                output="screen",
                parameters=[
                    {
                        "max_deceleration": 0.8,
                        "sensor_latency": sensor_latency,
                        "scan_delay_s": scan_delay_s,
                        "scan_noise_std_m": scan_noise_std_m,
                        "scan_noise_seed": scan_noise_seed,
                        "safety_margin": safety_margin,
                        "minimum_clearance": minimum_clearance,
                        "clearance_hysteresis": 0.03,
                        "front_angle_deg": 60.0,
                        "side_inner_angle_deg": 30.0,
                        "recovery_turn_speed": 0.60,
                        "recovery_timeout_s": recovery_timeout_s,
                        "recovery_turn_sign": -1.0,
                        "max_tilt_deg": 10.0,
                        "publish_rate_hz": 20.0,
                        "odom_topic": navigation_pose_topic,
                    }
                ],
            ),
            Node(
                package="robotics_nav",
                executable="static_map_publisher",
                name="static_map_publisher",
                output="screen",
                parameters=[
                    {"frame_id": localization_map_frame_id},
                ],
            ),
            Node(
                package="robotics_nav",
                executable="global_planner",
                name="global_planner",
                output="screen",
                parameters=[
                    {
                        "goal_x": 2.0,
                        "goal_y": 0.0,
                        "robot_radius_m": planning_radius_m,
                        "odom_topic": navigation_pose_topic,
                    }
                ],
            ),
            evaluation_node := Node(
                package="robotics_nav",
                executable="evaluation_logger",
                name="evaluation_logger",
                output="screen",
                parameters=[
                    {
                        "goal_tolerance": goal_tolerance,
                        "front_angle_deg": 60.0,
                        "sample_rate_hz": 20.0,
                        "output_path": evaluation_output,
                        "trace_output": trace_output,
                        "plan_output": plan_output,
                        "map_output": map_output,
                        "collision_topic": LaunchConfiguration("collision_topic"),
                        "minimum_clearance": minimum_clearance,
                        "sensor_latency": sensor_latency,
                        "scan_delay_s": scan_delay_s,
                        "scan_noise_std_m": scan_noise_std_m,
                        "scan_noise_seed": scan_noise_seed,
                        "safety_margin": safety_margin,
                        "planning_radius_m": planning_radius_m,
                        "experiment_timeout_s": experiment_timeout_s,
                        "navigation_pose_topic": navigation_pose_topic,
                    }
                ],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=evaluation_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason="evaluation logger exited")
                        )
                    ],
                )
            ),
        ]
    )
