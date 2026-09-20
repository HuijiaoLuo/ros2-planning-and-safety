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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "evaluation_output",
                default_value="",
                description="Optional CSV path for closed-loop evaluation metrics.",
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
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch_path),
                launch_arguments={
                    "evaluation_output": evaluation_output,
                }.items(),
            ),
        ]
    )
