from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('teamX_challenge')
    params_path = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    return LaunchDescription([
        # 启动决策节点
        Node(
            package='teamX_challenge',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[params_path]
        ),
        # 启动打击节点
        Node(
            package='teamX_challenge',
            executable='shooter_node',
            name='shooter_node',
            output='screen'
        )
    ])