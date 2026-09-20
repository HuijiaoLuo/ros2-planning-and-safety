from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "evaluation_output",
                default_value="",
                description="Optional CSV path for closed-loop evaluation metrics.",
            ),
            DeclareLaunchArgument(
                "collision_topic",
                default_value="/collision/contacts",
                description="ROS topic carrying Gazebo contact messages.",
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
                        "sensor_latency": 0.10,
                        "safety_margin": 0.15,
                        "minimum_clearance": 0.50,
                        "clearance_hysteresis": 0.03,
                        "front_angle_deg": 60.0,
                        "side_inner_angle_deg": 30.0,
                        "recovery_turn_speed": 0.60,
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
                        "robot_radius_m": 0.35,
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
                        "output_path": LaunchConfiguration("evaluation_output"),
                        "collision_topic": LaunchConfiguration("collision_topic"),
                    }
                ],
            ),
        ]
    )
