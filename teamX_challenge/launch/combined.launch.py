from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('teamX_challenge')
    params_path = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen'
        ),
        Node(
            package='teamX_challenge',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[params_path]
        ),
        Node(
            package='teamX_challenge',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[params_path]
        ),
        Node(
            package='teamX_challenge',
            executable='shooter_node',
            name='shooter_node',
            output='screen'
        )
    ])