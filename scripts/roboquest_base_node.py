#!/usr/bin/env python3

from rclpy import init as ROS_init
from roboquest_core.rq_base import RQBase

"""
Instantiates the RQBase object and starts it.
"""


def run(node_name: str = 'roboquest_base_node'):
    ROS_init(args=None)
    rq_base = RQBase(node_name)
    rq_base.main()


if __name__ == '__main__':
    run()
