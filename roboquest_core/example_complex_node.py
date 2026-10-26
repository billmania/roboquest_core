#!/usr/bin/env python3
"""Example of a complex ROS node.

This node is intended to be used as an example of a
more complex node. It gives examples of integration with
the Roboquest application.
"""

import rclpy
from rclpy.node import Node


class ExampleComplex(Node):
    """ExampleComplex.

    The logic for the example node.
    """

    def __init__(self, node_name: str):
        """Create the publisher and the timer."""
        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.DEBUG
        )

        self.get_logger().info(
            f'{node_name}, the complex example node, has started'
        )


def main(args=None):
    """Initialize the ROS node.

    Initialize the node and instantiate the ExampleComplex class.
    """
    rclpy.init(args=args)

    example_node = ExampleComplex('example_complex')

    rclpy.spin(example_node)

    example_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
