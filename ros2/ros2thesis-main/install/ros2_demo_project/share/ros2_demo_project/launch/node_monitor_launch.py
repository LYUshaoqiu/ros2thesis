from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_demo_project',
            executable='node_monitor_node',
            name='node_monitor_node',
            output='screen',
            parameters=[{'output_directory': '/home/lyu/new_ros2_ws/logs'}]
        ),
        Node(
            package='ros2_demo_project',
            executable='thingy52_node',
            name='thingy52_node',
            output='screen'
        ),
        Node(
            package='ros2_demo_project',
            executable='mmwave_radar_node',
            name='mmwave_radar_node',
            output='screen'
        ),
        Node(
            package='ros2_demo_project',
            executable='kinect_node',
            name='kinect_node',
            output='screen'
        ),
        Node(
            package='ros2_demo_project',
            executable='static_map_node',
            name='static_map_node',
            output='screen'
        )
    ])
