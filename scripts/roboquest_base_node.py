#!/usr/bin/env python3

from rclpy import init as ROS_init
from roboquest_core.rq_manage import RQManage

"""
Instantiates the RQManage object and starts it.
"""


def run(node_name: str = 'roboquest_base_node'):
    ROS_init(args=None)
    rq_manage = RQManage(node_name)
    rq_manage.main()
    rq_manage.hat.cleanup_gpio()


if __name__ == '__main__':
    run()
