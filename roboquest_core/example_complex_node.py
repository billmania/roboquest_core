#!/usr/bin/env python3
"""Example of a complex ROS node.

This node is intended to be used as an example of a
more complex node. It gives examples of integration with
the Roboquest application.
"""

#
# This node doesn't implement any ROS actions, but
# GoalStatus is most appropriate, existing interface
# to provide feedback about the /servo_trigger service.
#
from actionlib_msgs.msg import GoalStatus

from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.node import Node

from rq_msgs.msg import Servos
from rq_msgs.srv import Control

from std_srvs.srv import Trigger

#
# Set the first member of the list to 0 to ignore the servo. Set
# it to SET_ANGLE to set the ANGLE of the servo.
# When the first member is SET_ANGLE, the second is then interpreted as
# a degree angle position in the range [0, 359].
#
SERVO_PARAMS = {
    'servo0': [0, 0],
    'servo1': [0, 0],
    'servo2': [0, 0],
    'servo3': [1, 20],
    'servo4': [0, 0],
    'servo5': [1, 20],
    'servo6': [0, 0],
    'servo7': [1, 20],
    'servo8': [0, 0],
    'servo9': [0, 0],
    'servo10': [0, 0],
    'servo11': [0, 0],
    'servo12': [0, 0],
    'servo13': [0, 0],
    'servo14': [0, 0],
    'servo15': [0, 0]
}
SET_ANGLE = 1
COMMAND = 0
ANGLE = 1


class ExampleComplex(Node):
    """ExampleComplex.

    The logic for the example node.
    """

    def __init__(self, node_name: str):
        """Establish the facilities.

        Create the publishers, services, and service clients. Declare
        the parameters. Initialize the counters.
        """
        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.DEBUG
        )

        self._log = self.get_logger()
        self._log.info(
            f'{node_name}, the complex example node, has started'
        )

        self._setup_parameters()

        self._sequence_number = 0
        self._trigger_status_pub = self.create_publisher(
            GoalStatus,
            'servo_trigger_status',
            1
        )
        self._servos_pub = self.create_publisher(
            Servos,
            'servos',
            1
        )
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
        self._parameters = {}

        for servo_name in SERVO_PARAMS:
            param_value = SERVO_PARAMS[servo_name]
            self.declare_parameter(
                servo_name,
                param_value
            )
            self._parameters[servo_name] = self.get_parameter(servo_name)
            self._log.info(
                f'Parameter {servo_name}'
                f' = {self._parameters[servo_name].value}'
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
        self._set_servo_power('ON')

    def _set_servo_power(self, servos_state: str = 'OFF'):
        """Set the state of the servo power."""
        while self._servo_power_future:
            #
            # Still waiting for a previous call to complete.
            #
            self._log.warning(
                (
                    f'_set_servo_power({servos_state})'
                    ' waiting for previous call to complete'
                ),
                throttle_duration_sec=5.0
            )

        self._log.info(
            f'Setting servo power to {servos_state}'
        )
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
        self._set_servo_positions()

        response.success = True
        response.message = 'Servo control completed'
        return response

    def _set_servo_positions(self):
        """Set the position for each servo.

        Power the servos and then use the parameters to determine which
        servo to position. Publish a status message before and after
        positioning the servoes.
        """
        servo_trigger_status = GoalStatus()

        servo_trigger_status.goal_id.stamp = self.get_clock().now().to_msg()
        servo_trigger_status.status = GoalStatus.ACTIVE
        servo_trigger_status.text = f'{self._sequence_number} Received'
        self._trigger_status_pub.publish(servo_trigger_status)

        status_msg = 'no exception'
        servos = Servos()
        servos.header.stamp = self.get_clock().now().to_msg()

        for servo_name in SERVO_PARAMS:
            if self._parameters[servo_name].value[COMMAND] != SET_ANGLE:
                continue

            try:
                servo_object = getattr(servos, servo_name)
                setattr(
                    servo_object,
                    'command_type',
                    self._parameters[servo_name].value[COMMAND]
                )
                setattr(
                    servo_object,
                    'angle_deg',
                    self._parameters[servo_name].value[ANGLE]
                )

            except Exception:
                status_msg = 'exception'

        self._servos_pub.publish(servos)

        servo_trigger_status.goal_id.stamp = self.get_clock().now().to_msg()
        servo_trigger_status.status = GoalStatus.SUCCEEDED
        servo_trigger_status.text = (
            f'{self._sequence_number} Completed with {status_msg}'
        )
        self._trigger_status_pub.publish(servo_trigger_status)

        self._sequence_number += 1


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
