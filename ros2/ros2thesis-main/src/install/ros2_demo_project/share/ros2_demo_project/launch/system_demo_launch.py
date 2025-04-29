# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='ros2_demo_project',
#             executable='thingy52_node',
#             name='thingy52_node',
#             output='screen',
#             parameters=['config/thingy52_params.yaml']
#         ),
#         Node(
#             package='ros2_demo_project',
#             executable='mmwave_radar_node',
#             name='mmwave_radar_node',
#             output='screen',
#             parameters=['config/mmwave_radar_params.yaml']
#         ),
#         Node(
#             package='ros2_demo_project',
#             executable='kinect_node',
#             name='kinect_node',
#             output='screen',
#             parameters=['config/kinect_params.yaml']
#         )
#     ])
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_directory = os.path.join(
        os.path.expanduser('~'),
        'new_ros2_ws',
        'src',
        'ros2_demo_project',
        'config'
    )

    return LaunchDescription([
        Node(
            package='ros2_demo_project',
            executable='thingy52_node',
            name='thingy52_node',
            output='screen',
            parameters=[os.path.join(config_directory, 'thingy52_params.yaml')]
        ),
        Node(
            package='ros2_demo_project',
            executable='mmwave_radar_node',
            name='mmwave_radar_node',
            output='screen',
            parameters=[os.path.join(config_directory, 'mmwave_radar_params.yaml')]
        ),
        Node(
            package='ros2_demo_project',
            executable='kinect_node',
            name='kinect_node',
            output='screen',
            parameters=[os.path.join(config_directory, 'kinect_params.yaml')]
        ),
        Node(
            package='ros2_demo_project',
            executable='static_map_node',  # 新增的静态坐标系节点
            name='static_map_node',
            output='screen'
        )
    ])
