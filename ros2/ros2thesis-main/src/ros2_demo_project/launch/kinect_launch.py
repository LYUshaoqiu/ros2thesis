from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_demo_project',
            executable='kinect_node',
            name='kinect_node',
            output='screen',
            parameters=['config/kinect_params.yaml']
        )
    ])
