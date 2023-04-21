import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    base_config = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'roboquest_base.yaml'
    )

    roboquest_base_node = Node(
        package="roboquest_core",
        executable="roboquest_base_node.py",
        parameters=[base_config]
    )

    ld.add_action(roboquest_base_node)

    return ld
