#!/usr/bin/env python3
"""Example publisher node.

A simple ROS node intended for use as a template for creating
a more sophisticated publisher node.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

TEMPERATURE_FILE = '/sys/class/thermal/thermal_zone0/temp'
TIMER_PERIOD_S = 5
MILLIS_PER_S = 1000


class ExamplePublisher(Node):
    """ExamplePublisher.

    The class handling the setup and operation of the node template.
    """

    def __init__(self):
        """Create the publisher and the timer."""
        super().__init__('example_publisher')
        self._publisher_ = self.create_publisher(Float32, 'example', 1)
        self.timer = self.create_timer(TIMER_PERIOD_S, self._timer_callback)
        self.i = 0

    def _timer_callback(self):
        msg = Float32()
        with open(TEMPERATURE_FILE, 'r') as t:
            msg.data = int(t.read()) / MILLIS_PER_S
        self._publisher_.publish(msg)


def main(args=None):
    """Initialize the ROS node.

    Initialize the node and instantiate the ExamplePublisher class.
    """
    rclpy.init(args=args)

    example_publisher = ExamplePublisher()

    rclpy.spin(example_publisher)

    example_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
