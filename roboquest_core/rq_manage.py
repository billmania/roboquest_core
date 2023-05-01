from typing import Callable
from rclpy.parameter import Parameter
from rclpy import spin_once as ROSspin_once
from rclpy import shutdown as ROSshutdown
from roboquest_core.rq_node import RQNode
from roboquest_core.rq_hat import RQHAT


class RQManage(RQNode):
    """
    The node which manages the RoboQuest application. It contains
    all of the logic specific to management and use of the
    RoboQuest HAT.
    """

    def __init__(self,
                 node_name: str = 'RQManage',
                 diags_cb: Callable = None):
        super().__init__(node_name, diags_cb)
        self._setup_parameters()

        self._timeout_sec = 1 / self._parameters['hat_comms_read_hz']
        self.hat = RQHAT(port=self._parameters['hat_port'],
                         data_rate=self._parameters['hat_data_rate'],
                         data_bits=self._parameters['hat_data_bits'],
                         parity=self._parameters['hat_parity'],
                         stop_bits=self._parameters['hat_stop_bits'],
                         read_timeout_sec=self._timeout_sec,
                         serial_errors_cb=None)

    def _setup_parameters(self):
        parameter_declarations = [
            ('name', Parameter.Type.STRING),
            ('hat_port', Parameter.Type.STRING),
            ('hat_data_rate', Parameter.Type.INTEGER),
            ('hat_data_bits', Parameter.Type.INTEGER),
            ('hat_parity', Parameter.Type.STRING),
            ('hat_stop_bits', Parameter.Type.INTEGER),
            ('hat_comms_read_hz', Parameter.Type.INTEGER)
        ]
        self.declare_parameters(
            namespace='',
            parameters=parameter_declarations
        )

        parameter_names = list()
        for parameter in parameter_declarations:
            parameter_names.append(parameter[0])
        parameters = self.get_parameters(parameter_names)
        self._parameters = dict()
        for index, name in enumerate(parameter_names):
            self._parameters[name] = parameters[index].value
            self.get_logger().info(f"Parameter {name}:"
                                   f" {self._parameters[name]}")

    def main(self):
        """
        Setup the serial port and then read incoming sentences in a loop.
        """

        self.get_logger().info(f"{self._node_name} starting")
        self.setup_diags()

        self.get_logger().info(f"{self._node_name} spinning"
                               f" with timeout {self._timeout_sec}")

        self.hat.control_comms(enable=True)
        while True:
            ROSspin_once(node=self, timeout_sec=0)
            sentence = self.hat._read_sentence()

            try:
                if sentence:
                    sentence.index('$$SCREEN')
                    self.get_logger().info(f"Screen {sentence}")

            except ValueError:
                pass

        self.get_logger().info(f"{self._node_name} stopping")
        self.destroy_node()
        ROSshutdown()
