from launch import LaunchDescription
from launch_ros.actions import Node

NODES_PATH = (
    '/usr/src/ros2ws/install/roboquest_core/share/roboquest_core/persist' +
    '/nodes/'
)

def generate_launch_description():
    user_node = Node(
        name='example_publisher_node',
        executable=NODES_PATH+"example_publisher_node.py",
        respawn=True,
        respawn_delay=5
    )

    return LaunchDescription([user_node])
