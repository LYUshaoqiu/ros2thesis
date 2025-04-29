from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_demo_project',
            executable='static_map_node',
            name='static_map_node',
            output='screen'
        ),
        Node(
            package='ros2_demo_project',
            executable='room_map_node',
            name='room_map_node',
            output='screen'
        ),
        Node(
            package='ros2_demo_project',
            executable='mmwave_radar_node',  # 添加点云节点
            name='mmwave_radar_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        Node(
            package='ros2_demo_project',
            executable='node_monitor_node',
            name='node_monitor_node',
            output='screen',
            parameters=[{'output_directory': '/home/lyu/new_ros2_ws/logs'}]
        )
    ])
