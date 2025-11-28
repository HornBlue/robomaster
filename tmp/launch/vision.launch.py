from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件路径
    params_file = os.path.join(
        get_package_share_directory('player_pkg'),
        'config',
        'params.yaml'
    )
    
    # 创建视觉节点
    vision_node = Node(
        package='player_pkg',
        executable='TestNode',
        name='vision_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/camera/image_raw', '/camera/color/image_raw'),  # 根据实际相机话题调整
            ('/vision/target', '/detection/targets')
        ]
    )
    
    return LaunchDescription([
        vision_node,
    ])