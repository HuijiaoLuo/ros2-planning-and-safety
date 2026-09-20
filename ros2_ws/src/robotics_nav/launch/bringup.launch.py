from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    scan_delay_s = LaunchConfiguration("scan_delay_s")
    scan_noise_std_m = LaunchConfiguration("scan_noise_std_m")
    scan_noise_seed = LaunchConfiguration("scan_noise_seed")

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
            Node(
                package="robotics_nav",
                executable="path_follower",
                name="path_follower",
                output="screen",
                parameters=[
                    {
                        "lookahead_distance": 0.10,
                        "max_linear_speed": 0.20,
                        "goal_tolerance": 0.05,
                        "rotate_in_place_threshold": 0.5235987756,
                        "final_approach_distance": 0.60,
                        "final_approach_heading_gain": 1.0,
                        "final_approach_max_angular_speed": 0.60,
                        "heading_deadband": 0.03,
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
                    }
                ],
            ),
            Node(
                package="robotics_nav",
                executable="static_map_publisher",
                name="static_map_publisher",
                output="screen",
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
                    }
                ],
            ),
            Node(
                package="robotics_nav",
                executable="evaluation_logger",
                name="evaluation_logger",
                output="screen",
                parameters=[
                    {
                        "goal_tolerance": 0.05,
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
                    }
                ],
            ),
        ]
    )
