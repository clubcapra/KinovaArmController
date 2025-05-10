from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('arm_controller'),
        'config',
        'arm_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='arm_controller',
            executable='arm_controller_node',
            name='arm_node',
            parameters=[params_file],
            output='screen'
        )
    ])
