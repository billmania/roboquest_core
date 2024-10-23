import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from pathlib import Path

def generate_launch_description():
    base_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'roboquest_base.yaml'
    )
    camera0_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'rq_camera0.yaml'
    )
    camera1_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'rq_camera1.yaml'
    )
    camera2_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'rq_camera2.yaml'
    )
    camera3_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'rq_camera3.yaml'
    )

    rq_base_node = Node(
        name='rq_base_node',
        package="roboquest_core",
        executable="roboquest_base_node.py",
        parameters=[base_params],
        respawn=True,
        respawn_delay=5
    )

    rq_camera_node0 = Node(
        name='rq_camera_node0',
        package="camera_ros",
        executable="camera_node",
        parameters=[camera0_params],
        respawn=True,
        respawn_delay=5
    )
    rq_camera_node1 = Node(
        name='rq_camera_node1',
        package="camera_ros",
        executable="camera_node",
        parameters=[camera1_params],
        respawn=False
    )
    rq_camera_node2 = Node(
        name='rq_camera_node2',
        package="camera_ros",
        executable="camera_node",
        parameters=[camera2_params],
        respawn=False
    )
    rq_camera_node3 = Node(
        name='rq_camera_node3',
        package="camera_ros",
        executable="camera_node",
        parameters=[camera3_params],
        respawn=False
    )

    USER_LAUNCH_FILE = (
        '/usr/src/ros2ws/install/roboquest_core/share/roboquest_core/persist' +
        '/nodes' +
        '/user_nodes.launch.py'
    )

    if Path(USER_LAUNCH_FILE).exists():
        user_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([USER_LAUNCH_FILE])
        )

        return LaunchDescription([rq_base_node,
                                  rq_camera_node0,
                                  rq_camera_node1,
                                  rq_camera_node2,
                                  rq_camera_node3,
                                  user_nodes
                                 ])
    else:
        return LaunchDescription([rq_base_node,
                                  rq_camera_node0,
                                  rq_camera_node1,
                                  rq_camera_node2,
                                  rq_camera_node3
                                 ])
