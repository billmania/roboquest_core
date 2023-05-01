from typing import Callable
import rclpy
from rclpy.node import Node
import diagnostic_updater


class RQNode(Node):
    """
    The base ROS node for the RoboQuest project. It handles everything
    common to every kind of RoboQuest node.
    """

    def __init__(self,
                 node_name: str,
                 diags_cb: Callable = None):
        self._node_name = node_name
        super().__init__(self._node_name)
        self._diags_cb = diags_cb

    def setup_diags(self):
        """
        Define the diagnostics
        """

        if self._diags_cb and callable(self._diags_cb):
            self._diag_updater = diagnostic_updater.Updater(self)
            self._diag_updater.setHardwareID(self._node_name)
            self._diag_updater.add(f"{self._node_name}",
                                   self._diags_cb)

    def main(self):
        """
        An example of a simple main() method. A useful node will
        replace this method.
        """

        self.get_logger().info(f"{self._node_name} starting")

        self.setup_diags()

        self.get_logger().info(f"{self._node_name} spinning")
        rclpy.spin(self)

        self.get_logger().info(f"{self._node_name} stopping")
        self.destroy_node()

        rclpy.shutdown()
