import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory("robotics_sim")
    nav_share = get_package_share_directory("robotics_nav")
    world_path = os.path.join(sim_share, "worlds", "differential_drive.sdf")
    nav_launch_path = os.path.join(nav_share, "launch", "bringup.launch.py")
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
            ExecuteProcess(
                cmd=["gz", "sim", "-r", world_path],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/collision/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch_path),
                launch_arguments={
                    "evaluation_output": evaluation_output,
                    "trace_output": trace_output,
                    "plan_output": plan_output,
                    "map_output": map_output,
                    "collision_topic": collision_topic,
                    "minimum_clearance": minimum_clearance,
                    "sensor_latency": sensor_latency,
                    "safety_margin": safety_margin,
                    "recovery_timeout_s": recovery_timeout_s,
                    "planning_radius_m": planning_radius_m,
                }.items(),
            ),
        ]
    )
