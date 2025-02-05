#!/usr/bin/env python3
"""Test servos."""
from rclpy import init as ROS_init

from roboquest_core.servo_tester import RQServoTest

"""
Instantiates the RQServoTest object and starts it.
"""


def run(node_name: str = 'servo_tester_node'):
    """Create the node.

    Initialize the ROS environment then instantiate the RQServoTest object
    and set it in motion.
    """
    ROS_init(args=None)
    rq_servo = RQServoTest(node_name)
    rq_servo.main()


if __name__ == '__main__':
    run()
