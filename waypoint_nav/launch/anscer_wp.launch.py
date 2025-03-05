from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_navigation', # package name
            executable='waypoint_manager', # executable name
            name='waypoint_manager', # node name
            output='screen',
            parameters=[
                {"min_distance_threshold": 1.0}, # parameter
                {"min_rotation_threshold": 0.5} # parameter
            ]
        ),
        Node(
            package='waypoint_navigation',
            executable='waypoint_planner',
            name='waypoint_planner',
            output='screen'
        )
    ])