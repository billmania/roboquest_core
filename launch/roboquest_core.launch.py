import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'roboquest_base.yaml'
    )

    rq_base_node = Node(
        name='rq_base_node',
        package="roboquest_core",
        executable="roboquest_base_node.py",
        parameters=[params],
        respawn=True,
        respawn_delay=5
    )

    rq_camera_node = Node(
        name='rq_camera_node',
        package="usb_cam",
        executable="usb_cam_node_exe",
        respawn=True,
        respawn_delay=5
    )

    return LaunchDescription([rq_base_node,
                              rq_camera_node
                             ])
