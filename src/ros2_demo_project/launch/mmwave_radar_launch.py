
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_demo_project',
            executable='mmwave_radar_node',
            name='mmwave_radar_node',
            output='screen',
            parameters=['config/mmwave_radar_params.yaml']
        )
    ])
