#!/usr/bin/env python3
"""Example of a complex ROS node.

This node is intended to be used as an example of a
more complex node. It gives examples of integration with
the Roboquest application.
"""

from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.node import Node

from rq_msgs.srv import Control

from std_srvs.srv import Trigger


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

        self._log = self.get_logger()
        self._log.info(
            f'{node_name}, the complex example node, has started'
        )

        self._parameters = {}
        self._setup_parameters()

        self._trigger_srv = self.create_service(
            Trigger,
            'servo_trigger',
            self._trigger_cb
        )

        self._setup_servo_power()

    def _params_cb(self, updated_parameters) -> SetParametersResult:
        """Process parameter updates."""
        success = True

        for parameter in updated_parameters:
            try:
                self._parameters[parameter.name] = parameter
                self._log.info(
                    f'Updated parameter {parameter.name}'
                    f' to {self._parameters[parameter.name].value}'
                )

            except Exception as e:
                self._log.warning(
                    f'Failed to update parameter {parameter.name}'
                    f' Exception: {e}'
                )
                success = False

        if success:
            return SetParametersResult(successful=True)
        else:
            return SetParametersResult(successful=False)

    def _setup_parameters(self):
        """Get the parameters.

        Declare all of the parameters, retrieve any provided, and
        update the parameters dictionary.
        """
        declarations = [
            ('servo0', [0, 0]),
            ('servo1', [1, 90]),
            ('servo2', [0, 0]),
            ('servo3', [1, 80]),
            ('servo4', [0, 0]),
            ('servo5', [1, 70]),
            ('servo6', [0, 0]),
            ('servo7', [1, 60]),
            ('servo8', [0, 0]),
            ('servo9', [0, 0]),
            ('servo10', [0, 0]),
            ('servo11', [0, 0]),
            ('servo12', [0, 0]),
            ('servo13', [0, 0]),
            ('servo14', [0, 0]),
            ('servo15', [0, 0])
        ]

        for declaration in declarations:
            param_name = declaration[0]
            param_value = declaration[1]
            self.declare_parameter(
                param_name,
                param_value
            )
            self._parameters[param_name] = self.get_parameter(param_name)
            self._log.info(
                f'Parameter {param_name}'
                f' = {self._parameters[param_name].value}'
            )

        self.add_on_set_parameters_callback(self._params_cb)

    def _setup_servo_power(self):
        """Create the servo power client."""
        self._servo_power_future = None
        self._set_servo_power_finished = False
        self._servo_power_client = self.create_client(
            Control,
            'control_hat'
        )
        self._log.info(
            'Waiting for servo power control service to be available'
        )
        while not self._servo_power_client.wait_for_service(timeout_sec=1.0):
            rclpy.spin_once(
                node=self,
                executor=None,
                timeout_sec=0.0
            )
        self._log.info(
            'Servo power control service available'
        )

    def _set_servo_power(self, servos_state: str = 'OFF'):
        """Set the state of the servo power."""
        if self._servo_power_future:
            #
            # Still waiting for a previous call to complete.
            #
            self._log.warning(
                f'_set_servo_power({servos_state})'
                ' waiting for previous call to complete'
            )
            return

        request = Control.Request()
        request.set_servos = servos_state
        self._servo_power_future = self._servo_power_client.call_async(
            request
        )
        if self._servo_power_future.done():
            try:
                self._set_servo_power_finished = (
                    self._servo_power_future.result()
                )

            except Exception as e:
                self._log.warning(
                    f'_set_servo_power() excepted: {e}'
                )
        else:
            self._servo_power_future.add_done_callback(self._servo_power_done)

    def _servo_power_done(self, future):
        """Process the future."""
        if future.result().success:
            self._log.info(
                'Set servo power state succeeded'
            )
            self._set_servo_power_finished = True
        else:
            self._log.warning(
                'Set servo power state failed'
            )
            self._set_servo_power_finished = False

        self._servo_power_future = None

    def _trigger_cb(
         self,
         request: Trigger.Request,
         response: Trigger.Response
    ) -> Trigger.Response:
        """Start the servo control logic."""
        self._log.info(
            'Servo control triggered'
        )
        self._run_servo_logic()

        response.success = True
        response.message = 'Power controlled'
        return response

    def _run_servo_logic(self):
        """Run the servo logic.

        Power the servos.
        """
        self._set_servo_power('ON')
        self._log.info(
            f'Parameter servo5'
            f" = {self._parameters['servo5'].value}"
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
