from typing import List
from rclpy.parameter import Parameter
from rclpy import spin_once as ROSspin_once
from rclpy import shutdown as ROSshutdown
import diagnostic_msgs
from roboquest_core.rq_node import RQNode
from roboquest_core.rq_hat import RQHAT
from rq_msgs.srv import Control
from rq_msgs.msg import Telemetry


class RQManage(RQNode):
    """
    The node which manages the RoboQuest application. It contains
    all of the logic specific to management and use of the
    RoboQuest HAT.

    It requires rclpy.init() has already been called.
    """

    def __init__(self,
                 node_name: str = 'RQManage'):
        super().__init__(node_name)
        self._setup_parameters()

        self._telem_sentences = 0
        self._screen_sentences = 0
        self._sentence_errors = 0

        self._setup_ros_graph()

        self._timeout_sec = 1 / self._parameters['hat_comms_read_hz']
        self.hat = RQHAT(port=self._parameters['hat_port'],
                         data_rate=self._parameters['hat_data_rate'],
                         data_bits=self._parameters['hat_data_bits'],
                         parity=self._parameters['hat_parity'],
                         stop_bits=self._parameters['hat_stop_bits'],
                         read_timeout_sec=self._timeout_sec,
                         serial_errors_cb=None)

    def _control_cb(self, request, response):
        response.success = True

        if request.set_charger != 'IGNORE':
            set_charger_on = True if request.set_charger == 'ON' \
                else False
            self.hat.charger_control(set_charger_on)

        if request.set_fet1 != 'IGNORE':
            set_fet1_on = True if request.set_fet1 == 'ON' \
                else False
            self.hat.fet1_control(set_fet1_on)

        if request.set_fet2 != 'IGNORE':
            set_fet2_on = True if request.set_fet2 == 'ON' \
                else False
            self.hat.fet2_control(set_fet2_on)

        return response

    def _setup_ros_graph(self):
        """
        Setup the publishers, subscribers, and services.
        """

        self._telemetry_pub = self.create_publisher(Telemetry, 'telemetry', 1)
        self._charger_srv = self.create_service(Control,
                                                'control_hat',
                                                self._control_cb)

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

    def _publish_telemetry(self, fields: List[str]) -> None:
        telemetry_msg = Telemetry()

        telemetry_msg.header.stamp = self.get_clock().now().to_msg()

        telemetry_msg.battery_v = float(fields[1])
        telemetry_msg.battery_ma = float(fields[2])
        telemetry_msg.system_ma = float(fields[3])
        telemetry_msg.adc0_v = float(fields[4])
        telemetry_msg.adc1_v = float(fields[5])
        telemetry_msg.adc2_v = float(fields[6])
        telemetry_msg.adc3_v = float(fields[7])
        telemetry_msg.adc4_v = float(fields[8])
        telemetry_msg.battery_charging, telemetry_msg.charger_has_power = \
            self.hat.charger_state()

        self._telemetry_pub.publish(telemetry_msg)

    def _process_screen(self, fields: List[str]) -> None:
        # TODO: Implement
        pass

    def _process_sentence(self, sentence: str):
        """
        Determine if it's a SCREEN or a TELEM sentence.
        """

        if sentence:
            fields = sentence.split()

            if fields[0] == '$$TELEM':
                if len(fields) == 9:
                    self._publish_telemetry(fields)
                    self._telem_sentences += 1
                    return

            elif fields[0] == '$$SCREEN':
                if len(fields) == 3:
                    self.hat.control_comms(enable=False)
                    self._process_screen(fields)
                    self.hat.control_comms(enable=True)
                    self._screen_sentences += 1
                    return

        self.get_logger().warning(f"Unusable sentence {sentence}",
                                  throttle_duration_sec=60)
        self._sentence_errors += 1

    def _diags_cb(self, statuses):
        """
        Called by the diagnostics_updater utility.
        """

        # TODO: Use something more informative for the Status
        statuses.summary(
            diagnostic_msgs.msg.DiagnosticStatus.OK,
            'Telemetry')
        statuses.add('telem_sentences', f"{self._telem_sentences}")
        statuses.add('screen_sentences', f"{self._screen_sentences}")
        statuses.add('sentence_errors', f"{self._sentence_errors}")

        return statuses

    def main(self):
        """
        Setup the serial port and then read incoming sentences in a loop.
        """

        self.get_logger().info(f"{self._node_name} starting")
        self.setup_diags(diags_cb=self._diags_cb)

        self.get_logger().info(f"{self._node_name} spinning"
                               f" with timeout {self._timeout_sec}")

        self.hat.control_comms(enable=True)
        self.hat.charger_control(on=True)
        while True:
            ROSspin_once(node=self, timeout_sec=0)
            sentence = self.hat._read_sentence()

            if sentence:
                self._process_sentence(sentence)

        self.get_logger().info(f"{self._node_name} stopping")
        self.destroy_node()
        ROSshutdown()
