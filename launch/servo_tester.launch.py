import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from pathlib import Path

I2C_YAML_FILE = 'i2c.yaml'
I2C_PARAM_FILE = (
    '/usr/src/ros2ws/install/roboquest_core/share/roboquest_core/persist' +
    '/i2c' +
    '/' +
    I2C_YAML_FILE
)

def generate_launch_description():
    if Path(I2C_PARAM_FILE).exists():
        src = Path(I2C_PARAM_FILE)
        dest = (
            Path(get_package_share_directory('roboquest_core')) /
                'config' /
            I2C_YAML_FILE
        )
        dest.write_text(src.read_text())

    base_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'roboquest_base.yaml'
    )
    i2c_params = os.path.join(
        get_package_share_directory('roboquest_core'),
        'config',
        'i2c.yaml'
    )

    servo_tester_node = Node(
        name='servo_tester_node',
        package="roboquest_core",
        executable="servo_tester_node.py",
        parameters=[base_params, i2c_params],
        respawn=True,
        respawn_delay=5
    )

    return LaunchDescription([servo_tester_node])
