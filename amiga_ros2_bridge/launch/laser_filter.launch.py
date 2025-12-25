from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                "/amiga-ros2-bridge/amiga_ros2_bridge/laser_filter.yaml"
            ],
            remappings=[
                ('/scan', '/scan/raw'),
                ('/scan_filtered', '/scan')
            ]
        )
    ])
