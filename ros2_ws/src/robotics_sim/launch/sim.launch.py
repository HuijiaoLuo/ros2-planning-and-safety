import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from robotics_nav.scenario_profiles import get_scenario_profile, render_world


def generate_launch_description():
    sim_share = get_package_share_directory("robotics_sim")
    nav_share = get_package_share_directory("robotics_nav")
    world_template_path = os.path.join(
        sim_share,
        "worlds",
        "differential_drive.sdf",
    )
    nav_launch_path = os.path.join(nav_share, "launch", "bringup.launch.py")
    scenario = LaunchConfiguration("scenario")
    evaluation_output = LaunchConfiguration("evaluation_output")
    trace_output = LaunchConfiguration("trace_output")
    plan_output = LaunchConfiguration("plan_output")
    map_output = LaunchConfiguration("map_output")
    collision_topic = LaunchConfiguration("collision_topic")
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
    control_pose_topic = LaunchConfiguration("control_pose_topic")
    goal_event_topic = LaunchConfiguration("goal_event_topic")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    goal_confirmation_timeout_s = LaunchConfiguration(
        "goal_confirmation_timeout_s"
    )
    goal_confirmation_max_attempts = LaunchConfiguration(
        "goal_confirmation_max_attempts"
    )
    goal_confirmation_max_speed_m_s = LaunchConfiguration(
        "goal_confirmation_max_speed_m_s"
    )
    goal_confirmation_max_pose_age_s = LaunchConfiguration(
        "goal_confirmation_max_pose_age_s"
    )
    final_approach_speed_m_s = LaunchConfiguration("final_approach_speed_m_s")
    require_localization_match_for_goal = LaunchConfiguration(
        "require_localization_match_for_goal"
    )
    require_timestamped_localization_evidence = LaunchConfiguration(
        "require_timestamped_localization_evidence"
    )
    require_goal_reference_for_goal = LaunchConfiguration(
        "require_goal_reference_for_goal"
    )
    goal_reference_topic = LaunchConfiguration("goal_reference_topic")
    goal_reference_tolerance = LaunchConfiguration("goal_reference_tolerance")
    goal_reference_position_sigma_max_m = LaunchConfiguration(
        "goal_reference_position_sigma_max_m"
    )
    estimation_output = LaunchConfiguration("estimation_output")
    estimation_trace_output = LaunchConfiguration("estimation_trace_output")
    imu_gyro_bias_rad_s = LaunchConfiguration("imu_gyro_bias_rad_s")
    imu_gyro_noise_std_rad_s = LaunchConfiguration("imu_gyro_noise_std_rad_s")
    imu_gyro_noise_seed = LaunchConfiguration("imu_gyro_noise_seed")
    wheel_slip_ratio = LaunchConfiguration("wheel_slip_ratio")
    wheel_slip_noise_std = LaunchConfiguration("wheel_slip_noise_std")
    position_mode = LaunchConfiguration("position_mode")
    motion_prior_topic = LaunchConfiguration("motion_prior_topic")
    localization_output_topic = LaunchConfiguration("localization_output_topic")
    localization_belief_topic = LaunchConfiguration("localization_belief_topic")
    localization_backend = LaunchConfiguration("localization_backend")
    localization_diagnostic_output = LaunchConfiguration(
        "localization_diagnostic_output"
    )
    mcl_particle_count = LaunchConfiguration("mcl_particle_count")
    mcl_motion_distance_noise_std_m = LaunchConfiguration(
        "mcl_motion_distance_noise_std_m"
    )
    mcl_motion_yaw_noise_std_rad = LaunchConfiguration(
        "mcl_motion_yaw_noise_std_rad"
    )
    mcl_lidar_range_sigma_m = LaunchConfiguration("mcl_lidar_range_sigma_m")
    mcl_lidar_likelihood_floor = LaunchConfiguration(
        "mcl_lidar_likelihood_floor"
    )
    mcl_resample_ess_ratio = LaunchConfiguration("mcl_resample_ess_ratio")
    mcl_random_seed = LaunchConfiguration("mcl_random_seed")
    mcl_initial_position_std_m = LaunchConfiguration(
        "mcl_initial_position_std_m"
    )
    mcl_initial_heading_std_rad = LaunchConfiguration(
        "mcl_initial_heading_std_rad"
    )
    mcl_initialization_mode = LaunchConfiguration("mcl_initialization_mode")
    mcl_minimum_valid_beams = LaunchConfiguration("mcl_minimum_valid_beams")
    mcl_observation_window_size = LaunchConfiguration(
        "mcl_observation_window_size"
    )
    mcl_max_scan_pose_age_s = LaunchConfiguration("mcl_max_scan_pose_age_s")
    mcl_max_position_std_m = LaunchConfiguration("mcl_max_position_std_m")
    mcl_max_normalized_entropy = LaunchConfiguration("mcl_max_normalized_entropy")
    mcl_update_rate_hz = LaunchConfiguration("mcl_update_rate_hz")
    mcl_scan_stride = LaunchConfiguration("mcl_scan_stride")
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
    localization_max_candidate_mahalanobis_sq = LaunchConfiguration(
        "localization_max_candidate_mahalanobis_sq"
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
    localization_minimum_score_margin_m = LaunchConfiguration(
        "localization_minimum_score_margin_m"
    )
    wheel_weight = LaunchConfiguration("wheel_weight")
    fusion_mode = LaunchConfiguration("fusion_mode")
    external_position_fusion = LaunchConfiguration("external_position_fusion")
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

    def launch_world(context):
        """Render the selected obstacle profile before starting Gazebo."""

        scenario_name = scenario.perform(context)
        profile = get_scenario_profile(scenario_name)
        template = Path(world_template_path).read_text(encoding="utf-8")
        rendered_world = render_world(template, profile)
        generated_world_path = (
            Path(tempfile.gettempdir())
            / f"robotics_{profile.name}_{os.getpid()}.sdf"
        )
        generated_world_path.write_text(rendered_world, encoding="utf-8")
        return [
            ExecuteProcess(
                cmd=["gz", "sim", "-r", str(generated_world_path)],
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="baseline_obstacle",
                description=(
                    "Shared map/Gazebo scene profile. Supported values: "
                    "baseline_obstacle, l_corridor, symmetric_corridor."
                ),
            ),
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
                "navigation_pose_topic",
                default_value="/odom",
                description="Pose topic recorded as the navigation/localization result.",
            ),
            DeclareLaunchArgument(
                "control_pose_topic",
                default_value="/odom",
                description=(
                    "Continuous pose topic consumed by planning, control, and "
                    "safety; separate from asynchronous localization output."
                ),
            ),
            DeclareLaunchArgument(
                "goal_event_topic",
                default_value="/path_follower_goal_event",
                description=(
                    "Topic carrying timestamped path-follower terminal events "
                    "for temporal consistency diagnostics."
                ),
            ),
            DeclareLaunchArgument(
                "goal_tolerance",
                default_value="0.05",
                description=(
                    "Distance in metres used by navigation and evaluation to "
                    "declare the goal reached."
                ),
            ),
            DeclareLaunchArgument(
                "goal_confirmation_timeout_s",
                default_value="0.0",
                description=(
                    "Maximum time allowed for terminal goal confirmation; "
                    "timeout enters bounded final approach, and zero "
                    "preserves the unbounded legacy behavior."
                ),
            ),
            DeclareLaunchArgument(
                "goal_confirmation_max_attempts",
                default_value="0",
                description=(
                    "Maximum number of timed-out confirmation attempts before "
                    "the controller enters GOAL_UNCONFIRMED; zero is unlimited."
                ),
            ),
            DeclareLaunchArgument(
                "final_approach_speed_m_s",
                default_value="0.05",
                description=(
                    "Maximum linear speed used by confirmation-timeout "
                    "FINAL_APPROACH recovery."
                ),
            ),
            DeclareLaunchArgument(
                "goal_confirmation_max_speed_m_s",
                default_value="0.05",
                description=(
                    "Maximum navigation speed allowed while confirming the goal."
                ),
            ),
            DeclareLaunchArgument(
                "goal_confirmation_max_pose_age_s",
                default_value="0.15",
                description=(
                    "Maximum source-pose age allowed while confirming the goal."
                ),
            ),
            DeclareLaunchArgument(
                "require_localization_match_for_goal",
                default_value="false",
                description=(
                    "Require a fresh accepted LiDAR localization event before "
                    "latching the goal."
                ),
            ),
            DeclareLaunchArgument(
                "require_timestamped_localization_evidence",
                default_value="false",
                description=(
                    "Require new accepted timestamped localization candidates "
                    "after entering the goal tolerance before latching."
                ),
            ),
            DeclareLaunchArgument(
                "require_goal_reference_for_goal",
                default_value="false",
                description=(
                    "Require the configured independent goal-reference pose "
                    "and its covariance before latching the goal."
                ),
            ),
            DeclareLaunchArgument(
                "goal_reference_topic",
                default_value="/state_estimate",
                description="Independent pose topic used for terminal confirmation.",
            ),
            DeclareLaunchArgument(
                "goal_reference_tolerance",
                default_value="-1.0",
                description=(
                    "Independent-reference goal tolerance; negative reuses "
                    "goal_tolerance."
                ),
            ),
            DeclareLaunchArgument(
                "goal_reference_position_sigma_max_m",
                default_value="0.15",
                description=(
                    "Maximum 1-sigma planar uncertainty allowed for the "
                    "independent goal reference."
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
                description="Estimator position mode: wheel_pose or propagated.",
            ),
            DeclareLaunchArgument(
                "motion_prior_topic",
                default_value="/state_prediction",
                description=(
                    "Wheel/IMU-only prediction topic used as the motion prior "
                    "by asynchronous map localizers."
                ),
            ),
            DeclareLaunchArgument(
                "localization_output_topic",
                default_value="/localized_estimate",
                description="Output topic for the optional LiDAR-map corrected pose.",
            ),
            DeclareLaunchArgument(
                "localization_belief_topic",
                default_value="/localization_belief",
                description=(
                    "Per-update MCL posterior-quality event stream, separate "
                    "from accepted correction events."
                ),
            ),
            DeclareLaunchArgument(
                "localization_backend",
                default_value="v4",
                description=(
                    "Localization backend: v4 local matcher, mcl particle "
                    "filter, or icp point-to-point registration."
                ),
            ),
            DeclareLaunchArgument(
                "mcl_particle_count",
                default_value="500",
                description="Particle count; fixed for cross-map experiments.",
            ),
            DeclareLaunchArgument(
                "mcl_motion_distance_noise_std_m",
                default_value="0.01",
                description="Motion-model distance noise standard deviation.",
            ),
            DeclareLaunchArgument(
                "mcl_motion_yaw_noise_std_rad",
                default_value="0.01",
                description="Motion-model yaw noise standard deviation.",
            ),
            DeclareLaunchArgument(
                "mcl_lidar_range_sigma_m",
                default_value="0.08",
                description="Occupancy likelihood-field range sigma.",
            ),
            DeclareLaunchArgument(
                "mcl_lidar_likelihood_floor",
                default_value="1e-6",
                description="Likelihood floor for invalid map evidence.",
            ),
            DeclareLaunchArgument(
                "mcl_resample_ess_ratio",
                default_value="0.5",
                description="Resampling threshold as a fraction of particles.",
            ),
            DeclareLaunchArgument(
                "mcl_random_seed",
                default_value="0",
                description="Particle-filter random seed.",
            ),
            DeclareLaunchArgument(
                "mcl_initial_position_std_m",
                default_value="0.05",
                description="Initial position spread around the state estimate.",
            ),
            DeclareLaunchArgument(
                "mcl_initial_heading_std_rad",
                default_value="0.10",
                description="Initial heading spread around the state estimate.",
            ),
            DeclareLaunchArgument(
                "mcl_initialization_mode",
                default_value="local",
                description=(
                    "MCL prior: local Gaussian around odometry or global "
                    "uniform free-space hypotheses."
                ),
            ),
            DeclareLaunchArgument(
                "mcl_minimum_valid_beams",
                default_value="6",
                description="Minimum valid scan beams for an accepted update.",
            ),
            DeclareLaunchArgument(
                "mcl_observation_window_size",
                default_value="3",
                description=(
                    "Number of consecutive odometry-compensated scans jointly "
                    "scored by the particle filter."
                ),
            ),
            DeclareLaunchArgument(
                "mcl_max_scan_pose_age_s",
                default_value="0.25",
                description=(
                    "Reject a scan when its timestamp is older than the "
                    "current state-estimate timestamp by this bound."
                ),
            ),
            DeclareLaunchArgument(
                "mcl_max_position_std_m",
                default_value="0.15",
                description=(
                    "Reject a candidate when its largest one-sigma planar "
                    "position standard deviation exceeds this bound."
                ),
            ),
            DeclareLaunchArgument(
                "mcl_max_normalized_entropy",
                default_value="0.98",
                description=(
                    "Reject a large correction when normalized particle "
                    "entropy exceeds this information-quality bound."
                ),
            ),
            DeclareLaunchArgument(
                "mcl_update_rate_hz",
                default_value="2.0",
                description="LiDAR update rate; pose publication remains high-rate.",
            ),
            DeclareLaunchArgument(
                "mcl_scan_stride",
                default_value="6",
                description="LiDAR beam stride used for the likelihood update.",
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
                    "occupied/free boundaries; point_to_line uses "
                    "nearest-boundary ICP-style normal residuals."
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
                "localization_max_candidate_mahalanobis_sq",
                default_value="0.0",
                description=(
                    "Optional EKF covariance gate for LiDAR candidates; "
                    "zero disables the gate."
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
                "localization_minimum_score_margin_m",
                default_value="0.0",
                description=(
                    "Optional minimum score gap between the best and second-"
                    "best LiDAR candidates; zero disables this ambiguity gate."
                ),
            ),
            DeclareLaunchArgument(
                "wheel_weight",
                default_value="0.02",
                description="Complementary-fusion correction weight applied to wheel yaw.",
            ),
            DeclareLaunchArgument(
                "fusion_mode",
                default_value="fixed",
                description="Fusion mode: fixed, adaptive heading, or full pose EKF.",
            ),
            DeclareLaunchArgument(
                "external_position_fusion",
                default_value="false",
                description=(
                    "Fuse one accepted map-localizer position event into the "
                    "pose EKF; requires fusion_mode=ekf."
                ),
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
                description="Wheel-yaw bias random-walk standard deviation in rad/sqrt(s).",
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
                description="Initial wheel-yaw bias variance in rad^2.",
            ),
            DeclareLaunchArgument(
                "adaptive_wheel_noise",
                default_value="false",
                description="Adapt wheel-yaw measurement noise from innovations.",
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
            OpaqueFunction(function=launch_world),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/collision/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch_path),
                launch_arguments={
                    "scenario": scenario,
                    "evaluation_output": evaluation_output,
                    "trace_output": trace_output,
                    "plan_output": plan_output,
                    "map_output": map_output,
                    "collision_topic": collision_topic,
                    "minimum_clearance": minimum_clearance,
                    "sensor_latency": sensor_latency,
                    "scan_delay_s": scan_delay_s,
                    "scan_noise_std_m": scan_noise_std_m,
                    "scan_noise_seed": scan_noise_seed,
                    "navigation_pose_topic": navigation_pose_topic,
                    "control_pose_topic": control_pose_topic,
                    "goal_event_topic": goal_event_topic,
                    "goal_tolerance": goal_tolerance,
                    "goal_confirmation_timeout_s": goal_confirmation_timeout_s,
                    "goal_confirmation_max_attempts": (
                        goal_confirmation_max_attempts
                    ),
                    "final_approach_speed_m_s": final_approach_speed_m_s,
                    "goal_confirmation_max_speed_m_s": (
                        goal_confirmation_max_speed_m_s
                    ),
                    "goal_confirmation_max_pose_age_s": (
                        goal_confirmation_max_pose_age_s
                    ),
                    "require_localization_match_for_goal": (
                        require_localization_match_for_goal
                    ),
                    "require_timestamped_localization_evidence": (
                        require_timestamped_localization_evidence
                    ),
                    "require_goal_reference_for_goal": (
                        require_goal_reference_for_goal
                    ),
                    "goal_reference_topic": goal_reference_topic,
                    "goal_reference_tolerance": goal_reference_tolerance,
                    "goal_reference_position_sigma_max_m": (
                        goal_reference_position_sigma_max_m
                    ),
                    "estimation_output": estimation_output,
                    "estimation_trace_output": estimation_trace_output,
                    "imu_gyro_bias_rad_s": imu_gyro_bias_rad_s,
                    "imu_gyro_noise_std_rad_s": imu_gyro_noise_std_rad_s,
                    "imu_gyro_noise_seed": imu_gyro_noise_seed,
                    "wheel_slip_ratio": wheel_slip_ratio,
                    "wheel_slip_noise_std": wheel_slip_noise_std,
                    "position_mode": position_mode,
                    "motion_prior_topic": motion_prior_topic,
                    "localization_output_topic": localization_output_topic,
                    "localization_belief_topic": localization_belief_topic,
                    "localization_backend": localization_backend,
                    "mcl_particle_count": mcl_particle_count,
                    "mcl_motion_distance_noise_std_m": mcl_motion_distance_noise_std_m,
                    "mcl_motion_yaw_noise_std_rad": mcl_motion_yaw_noise_std_rad,
                    "mcl_lidar_range_sigma_m": mcl_lidar_range_sigma_m,
                    "mcl_lidar_likelihood_floor": mcl_lidar_likelihood_floor,
                    "mcl_resample_ess_ratio": mcl_resample_ess_ratio,
                    "mcl_random_seed": mcl_random_seed,
                    "mcl_initial_position_std_m": mcl_initial_position_std_m,
                    "mcl_initial_heading_std_rad": mcl_initial_heading_std_rad,
                    "mcl_initialization_mode": mcl_initialization_mode,
                    "mcl_minimum_valid_beams": mcl_minimum_valid_beams,
                    "mcl_observation_window_size": mcl_observation_window_size,
                    "mcl_max_scan_pose_age_s": mcl_max_scan_pose_age_s,
                    "mcl_max_position_std_m": mcl_max_position_std_m,
                    "mcl_max_normalized_entropy": mcl_max_normalized_entropy,
                    "mcl_update_rate_hz": mcl_update_rate_hz,
                    "mcl_scan_stride": mcl_scan_stride,
                    "localization_diagnostic_output": localization_diagnostic_output,
                    "localization_search_radius_m": localization_search_radius_m,
                    "localization_search_step_m": localization_search_step_m,
                    "localization_scan_stride": localization_scan_stride,
                    "localization_score_mode": localization_score_mode,
                    "localization_optimizer_mode": localization_optimizer_mode,
                    "localization_refine_top_k": localization_refine_top_k,
                    "localization_publish_rate_hz": localization_publish_rate_hz,
                    "localization_match_rate_hz": localization_match_rate_hz,
                    "localization_max_correction_m": localization_max_correction_m,
                    "localization_max_total_correction_m": localization_max_total_correction_m,
                    "localization_robot_radius_m": localization_robot_radius_m,
                    "localization_max_match_score_m": localization_max_match_score_m,
                    "localization_max_candidate_mahalanobis_sq": (
                        localization_max_candidate_mahalanobis_sq
                    ),
                    "localization_correction_smoothing": localization_correction_smoothing,
                    "localization_minimum_consecutive_matches": localization_minimum_consecutive_matches,
                    "localization_candidate_consistency_m": localization_candidate_consistency_m,
                    "localization_minimum_reapplication_change_m": localization_minimum_reapplication_change_m,
                    "localization_max_match_age_s": localization_max_match_age_s,
                    "localization_max_odom_motion_during_match_m": localization_max_odom_motion_during_match_m,
                    "localization_max_odom_yaw_change_during_match_rad": localization_max_odom_yaw_change_during_match_rad,
                    "localization_map_frame_id": localization_map_frame_id,
                    "localization_odom_frame_id": localization_odom_frame_id,
                    "localization_broadcast_map_odom_tf": localization_broadcast_map_odom_tf,
                    "localization_yaw_search_radius_rad": localization_yaw_search_radius_rad,
                    "localization_yaw_search_step_rad": localization_yaw_search_step_rad,
                    "localization_yaw_prior_weight": localization_yaw_prior_weight,
                    "localization_max_heading_correction_rad": localization_max_heading_correction_rad,
                    "localization_minimum_score_improvement_m": localization_minimum_score_improvement_m,
                    "localization_minimum_score_margin_m": localization_minimum_score_margin_m,
                    "wheel_weight": wheel_weight,
                    "fusion_mode": fusion_mode,
                    "external_position_fusion": external_position_fusion,
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
                    "nis_gate_threshold": nis_gate_threshold,
                    "safety_margin": safety_margin,
                    "recovery_timeout_s": recovery_timeout_s,
                    "planning_radius_m": planning_radius_m,
                    "experiment_timeout_s": experiment_timeout_s,
                }.items(),
            ),
        ]
    )
