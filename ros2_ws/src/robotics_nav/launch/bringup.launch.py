from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
                    }
                ],
            ),
            Node(
                package="robotics_nav",
                executable="safety_supervisor",
                name="safety_supervisor",
                output="screen",
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
        ]
    )
