#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rq_msgs.msg import ServoAngles, ServoAngle


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('servo_publisher')
        self.publisher_ = self.create_publisher(ServoAngles, 'servos', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        servos = ServoAngles()
        elbow = ServoAngle()
        elbow.name = 'elbow'
        elbow.angle = 180
        servos.servos.append(elbow)
        wrist = ServoAngle()
        wrist.name = 'wrist_pan'
        wrist.angle = 180
        servos.servos.append(wrist)

        self.publisher_.publish(servos)
        self.get_logger().info('Publishing: "%s"' % servos)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)

    except KeyboardInterrupt:
        pass

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
