import rclpy
from rclpy.node import Node
import diagnostic_updater


class RQNode(Node):
    """
    The base ROS node for the RoboQuest project. It handles everything
    common to every kind of RoboQuest node.
    """

    def __init__(self, node_name: str):
        self._node_name = node_name
        super().__init__(self._node_name)

        self.diags_cb = None

    def setup_diags(self):
        """
        Define the diagnostics
        """

        if self.diags_cb and callable(self.diags_cb):
            self._diag_updater = diagnostic_updater.Updater(self)
            self._diag_updater.setHardwareID(self._node_name)
            self._diag_updater.add(f"{self._node_name}",
                                   self.diags_cb)

    def setup_parameters(self):
        """
        Expected to be overwritten by the child class.
        """

        # TODO: Implement
        pass

    def main(self):
        self.get_logger().info(f"{self._node_name} starting")

        self.setup_diags()

        self.get_logger().info(f"{self._node_name} spinning")
        rclpy.spin(self)

        self.get_logger().info(f"{self._node_name} stopping")
        self.destroy_node()

        rclpy.shutdown()
