import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    base_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'roboquest_base.yaml'
    )
    ardu_camera_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'rq_camera.yaml'
    )
    usb_camera_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'usb_camera.yaml'
    )

    rq_base_node = Node(
        name='rq_base_node',
        package="roboquest_core",
        executable="roboquest_base_node.py",
        parameters=[base_params],
        respawn=True,
        respawn_delay=5
    )

    # un-comment one camera and adjust the return statement below

    #
    # For the ArduCam/RasPiCam
    #
    ardu_camera_node = Node(
        name='ardu_camera_node',
        package="camera_ros",
        executable="camera_node",
        parameters=[ardu_camera_params],
        respawn=False,
        respawn_delay=5
    )

    #
    # For a USB webcam
    #
    usb_camera_node = Node(
        name='usb_camera_node',
        package="usb_cam",
        executable="usb_cam_node_exe",
        parameters=[usb_camera_params],
        remappings=[
            ('/image_raw', '/video0_raw'),
        ],
        respawn=False,
        respawn_delay=5
    )

    return LaunchDescription([rq_base_node,
                              ardu_camera_node,
                              usb_camera_node
                             ])
