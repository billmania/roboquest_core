"""Manage the core functionality of the RoboQuest backend.

Control the features of the HAT, provide the ROS services and topics.
"""

from os import getpid, kill
from signal import SIGKILL
from threading import Timer
from typing import List

import diagnostic_msgs

from geometry_msgs.msg import TwistStamped

import rclpy.logging
from rclpy import ok as ROSok
from rclpy import shutdown as ROSshutdown
from rclpy import spin_once as ROSspin_once
from rclpy.parameter import Parameter

from roboquest_core.rq_config_file import ConfigFile
from roboquest_core.rq_drive_utilities import DriveUtils
from roboquest_core.rq_gpio_user import PinError, USER_GPIO_PIN, UserGPIO
from roboquest_core.rq_hat import HAT_BUTTON, HAT_SCREEN
from roboquest_core.rq_hat import RQHAT
from roboquest_core.rq_hat import SCREEN_HEADER, TELEM_HEADER
from roboquest_core.rq_i2c import RQI2CComms
from roboquest_core.rq_i2c_modules import I2CSupport
from roboquest_core.rq_motors import RQMotors
from roboquest_core.rq_network import RQNetwork
from roboquest_core.rq_node import RQNode
from roboquest_core.rq_servos import RQServos
from roboquest_core.rq_servos import ServoError, TranslateError
from roboquest_core.rq_servos_config import servo_config

from rq_msgs.msg import GPIOInput
from rq_msgs.msg import GPIOOutput
from rq_msgs.msg import MotorSpeed
from rq_msgs.msg import Servos
from rq_msgs.msg import Telemetry
from rq_msgs.srv import Control

from std_srvs.srv import Empty

VERSION = '24.1'

MODULE_DIR = (
    '/usr/src/ros2ws'
    '/install/roboquest_core/share/roboquest_core/persist'
    '/i2c'
)


#
# How many seconds to wait before calling exit().
#
EXIT_DELAY_S = 5

INSTALL_DIR = '/usr/src/ros2ws/install'
PERSIST_BASE_DIR = INSTALL_DIR + '/roboquest_core/share/roboquest_core'
PERSIST_DIR = PERSIST_BASE_DIR + '/persist'
SERVO_CONFIG = 'servos_config.json'
COMMAND_IGNORE = 0
COMMAND_ANGLE = 1
COMMAND_INCR = 2
COMMAND_SPEED = 3


class RQManage(RQNode):
    """Manage the ROS framework.

    The node which manages the RoboQuest application. It contains
    all of the logic specific to management and use of the
    RoboQuest HAT, network interfaces, motors, servos, and I2C
    communications.

    It requires rclpy.init() has already been called.
    """

    def __init__(self,
                 node_name: str = 'RQManage'):
        """Initialize the object."""
        super().__init__(node_name)
        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.DEBUG)
        self._setup_parameters()

        self._telem_sentences = 0
        self._screen_sentences = 0
        self._sentence_errors = 0

        self._previous_screen = 0

        self._setup_ros_graph()

        self._timeout_sec = 1 / self._parameters['hat_comms_read_hz']
        self._hat = RQHAT(port=self._parameters['hat_port'],
                          data_rate=self._parameters['hat_data_rate'],
                          data_bits=self._parameters['hat_data_bits'],
                          parity=self._parameters['hat_parity'],
                          stop_bits=self._parameters['hat_stop_bits'],
                          read_timeout_sec=self._timeout_sec,
                          serial_errors_cb=None)
        self._network = RQNetwork(
            self.get_logger().warning,
            self._hat.pad_line,
            self._hat.pad_text)
        self._motors = RQMotors(
            self._parameters['motors_i2c_bus_id'],
            self._parameters['min_rpm'],
            self._parameters['max_rpm'],
            self._parameters['rpm_to_tps'],
            self.get_logger
        )
        self._drive_utils = DriveUtils(
            self._parameters['sprocket_radius'],
            self._parameters['track_circumference'],
            self._parameters['track_separation']
        )
        self._gpio = UserGPIO()
        self._i2c = RQI2CComms()
        self._i2c_support = I2CSupport()
        self._i2c_objects = self._i2c_support.import_modules(MODULE_DIR)
        if self._i2c_objects:
            for i2c_object in self._i2c_objects:
                try:
                    self._i2c_objects[i2c_object].setup()
                    self.get_logger().info(
                        f'I2C {i2c_object} setup() executed'
                    )
                except Exception as e:
                    self.get_logger().warn(
                        f'I2C {i2c_object} setup() excepted: {e}'
                    )

        #
        # servo_config() provides a default servo configuration. It's
        # expected to be called only if there isn't a servo configuration file
        # in the persistent configuration directory.
        #
        self._servo_config = ConfigFile(PERSIST_DIR)
        self._servo_config.init_config(SERVO_CONFIG, servo_config)
        self._servos = RQServos(
            self._servo_config.get_config(SERVO_CONFIG),
            self._parameters['servos_i2c_bus_id'],
            self.get_logger
        )

        self._exit_timer = Timer(EXIT_DELAY_S, self._exit_worker)

    def _exit_worker(self):
        """Terminate the worker.

        Call the exit() function to terminate the node. This method
        of terminating the node does NOT perform a clean shutdown.
        Any in-flight ROS publishes, subscribes, or service calls
        may be incomplete.
        """
        self.get_logger().fatal('_exit_worker: Calling kill')
        kill(getpid(), SIGKILL)

    def _gpio_output_cb(self, msg: GPIOOutput) -> None:
        """Handle a GPIO configuration and output command.

        Extract the contents of the GPIOOutput message. Use the _direction
        parts to configure the GPIO pins as one of: input, output, or
        idle. Use the _state parts to determine how to set each pin
        configured as an output.
        """
        for gpio_pin in USER_GPIO_PIN:
            direction = getattr(msg, gpio_pin.name+'_direction')
            if direction == GPIOOutput.INPUT:
                try:
                    self._gpio.make_input(gpio_pin)
                except PinError:
                    self.get_logger().warning(
                        '_gpio_output_cb: make_input PinError'
                    )
            elif direction == GPIOOutput.OUTPUT:
                try:
                    self._gpio.make_output(gpio_pin)
                except PinError:
                    self.get_logger().warning(
                        '_gpio_output_cb: make_output PinError'
                    )

                try:
                    self._gpio.set_pin(
                        gpio_pin,
                        getattr(msg, gpio_pin.name+'_state')
                    )
                except PinError:
                    self.get_logger().warning(
                        '_gpio_output_cb: set_pin PinError'
                    )

            elif direction == GPIOOutput.UNUSED:
                try:
                    self._gpio.clear_pin(gpio_pin)
                except PinError:
                    self.get_logger().warning(
                        '_gpio_output_cb: clear_pin PinError'
                    )

    def _servo_cb(self, msg: Servos) -> None:
        """Handle a servo command.

        Extract the command for each servo and send them to the
        servo controller. This method could need a long time to
        run, dependent upon the configuration of the servo delays
        and the quantity of servo commands in the message.
        """
        if not self._servos.controller_powered():
            self.get_logger().warning('servos are not enabled',
                                      throttle_duration_sec=60)
            return

        for servo_id in self._servo_list:
            servo = getattr(msg, servo_id)
            if servo.command_type == COMMAND_IGNORE:
                continue

            try:
                which_servo = servo_id.replace('servo', '')

                if servo.command_type == COMMAND_ANGLE:
                    self._servos.set_servo_angle(
                        which_servo,
                        servo.angle_deg)
                elif servo.command_type == COMMAND_INCR:
                    self._servos.incr_servo_angle(
                        which_servo,
                        servo.angle_incr_deg)
                elif servo.command_type == COMMAND_SPEED:
                    self._servos.set_servo_speed(
                        which_servo,
                        servo.speed_dps
                    )

            except TranslateError as e:
                self.get_logger().warning(
                    f'_servo_cb: {e}',
                    throttle_duration_sec=60)

            except ServoError as e:
                self.get_logger().warning(
                    f'_servo_cb: {e}',
                    throttle_duration_sec=60)

    def _motor_speed_cb(self, msg: MotorSpeed):
        """Set the maximum motor speed."""
        if not (0 <= msg.max_rpm <= self._parameters['max_rpm']):
            self.get_logger().error(
                'motor_speed_cb'
                f' max: {msg.max_rpm}'
                f" must be in [0, {self._parameters['max_rpm']}]"
            )
            return

        self._motors.set_user_max_rpm(msg.max_rpm)

    def _motor_cb(self, msg: TwistStamped):
        """Handle a motor command.

        Extract the linear.x and angular.z from the Twist message,
        convert it to an x and y RPM, and pass it to the
        motors controller.
        """
        if not self._motors.motors_are_enabled():
            self.get_logger().warning('motors are not enabled',
                                      throttle_duration_sec=60)
            return

        #
        # The units for linear.x are meters per second and for angular.z
        # are radians per second.
        # The robot has a left and right motor, so this method uses
        # linear.x and angular.z to calculate a net velocity for each
        # of the two drive motors.
        #
        right_velocity = left_velocity = (
            self._drive_utils.linear_to_rpm(msg.twist.linear.x)
        )
        angular_velocity = self._drive_utils.angular_to_rpm(
            msg.twist.angular.z
        )
        left_velocity -= angular_velocity
        right_velocity += angular_velocity

        if not self._motors.set_motors_rpm(right=round(right_velocity),
                                           left=round(left_velocity)):
            self.get_logger().warning('failed to set motors RPM')

    def _restart_cb(self, request, response):
        """Handle a restart command.

        Cause the node to exit, in order to have some other daemon
        automatically restart it. This is usually done to load an
        updated configuration.

        In order for this callback to return a service response so
        the caller doesn't hang, it uses a Timer to call exit().
        """
        self.get_logger().info('_restart_cb called')

        self._exit_timer.start()
        return response

    def _control_cb(self, request, response):
        """Handle a control command.

        Implement the control_hat service. Valid values for each
        attribute of the interface are: ON and OFF. Any other value
        is ignored.
        """
        self.get_logger().info('_control_cb'
                               f' request: {request}')

        response.success = True
        valid_values = ['ON', 'OFF']

        if request.set_charger in valid_values:
            set_charger_on = True if request.set_charger == 'ON' \
                else False
            self._hat.charger_control(set_charger_on)

        if request.set_fet1 in valid_values:
            set_fet1_on = True if request.set_fet1 == 'ON' \
                else False
            self._hat.fet1_control(set_fet1_on)

        if request.set_fet2 in valid_values:
            set_fet2_on = True if request.set_fet2 == 'ON' \
                else False
            self._hat.fet2_control(set_fet2_on)

        if request.set_motors in valid_values:
            set_motors_on = True if request.set_motors == 'ON' \
                else False
            self._motors.enable_motors(set_motors_on)

        if request.set_servos in valid_values:
            set_servos_on = True if request.set_servos == 'ON' \
                else False
            self._servos.set_power(set_servos_on)

        return response

    def _setup_ros_graph(self):
        """Connect to the ROS graph.

        Setup the publishers, subscribers, and services.
        """
        self._telemetry_pub = self.create_publisher(Telemetry, 'telemetry', 1)
        self._gpio_pub = self.create_publisher(GPIOInput, 'gpio_input', 1)
        self._motor_sub = self.create_subscription(
            TwistStamped,
            'cmd_vel',
            self._motor_cb,
            1)
        self._motor_speed_sub = self.create_subscription(
            MotorSpeed,
            'motor_speed',
            self._motor_speed_cb,
            1)
        #
        # Collect the servos specified in the Servos message,
        # to optimize the callback.
        #
        self._servo_list = []
        for servo in dir(Servos()):
            if servo.find('servo') == 0:
                self._servo_list.append(servo)

        self._servo_sub = self.create_subscription(Servos,
                                                   'servos',
                                                   self._servo_cb,
                                                   1)
        self._gpio_sub = self.create_subscription(GPIOOutput,
                                                  'gpio_output',
                                                  self._gpio_output_cb,
                                                  1)
        self._control_srv = self.create_service(Control,
                                                'control_hat',
                                                self._control_cb)
        self._restart_srv = self.create_service(Empty,
                                                'restart',
                                                self._restart_cb)

    def _setup_parameters(self):
        parameter_declarations = [
            ('name', Parameter.Type.STRING),
            ('hat_port', Parameter.Type.STRING),
            ('hat_data_rate', Parameter.Type.INTEGER),
            ('hat_data_bits', Parameter.Type.INTEGER),
            ('hat_parity', Parameter.Type.STRING),
            ('hat_stop_bits', Parameter.Type.INTEGER),
            ('hat_comms_read_hz', Parameter.Type.INTEGER),
            ('servos_i2c_bus_id', Parameter.Type.INTEGER),
            ('motors_i2c_bus_id', Parameter.Type.INTEGER),
            ('min_rpm', Parameter.Type.INTEGER),
            ('max_rpm', Parameter.Type.INTEGER),
            ('rpm_to_tps', Parameter.Type.DOUBLE),
            ('track_circumference', Parameter.Type.DOUBLE),
            ('track_separation', Parameter.Type.DOUBLE),
            ('sprocket_radius', Parameter.Type.DOUBLE)
        ]
        self.declare_parameters(
            namespace='',
            parameters=parameter_declarations
        )

        parameter_names = []
        for parameter in parameter_declarations:
            parameter_names.append(parameter[0])
        parameters = self.get_parameters(parameter_names)
        self._parameters = {}
        for index, name in enumerate(parameter_names):
            self._parameters[name] = parameters[index].value
            self.get_logger().info(f'Parameter {name}:'
                                   f' {self._parameters[name]}')

    def _publish_gpio(self):
        """Publish the current state of each GPIO input pin.

        Read the current list of GPIO pins configured as input
        and publish a message with each pins current state.
        """
        try:
            input_pins = self._gpio.read_input_pins()
        except PinError:
            self.get_logger().warning(
                '_publish_gpio: PinError for read_input_pins'
            )
            return

        if not input_pins:
            return

        gpio_msg = GPIOInput()
        gpio_msg.header.stamp = self.get_clock().now().to_msg()

        for gpio_pin in input_pins:
            self.get_logger().info(
                '_publish_gpio:'
                + f' {gpio_pin[0].name} = {gpio_pin[1]}',
                throttle_duration_sec=5.0
            )
            setattr(gpio_msg, gpio_pin[0].name, gpio_pin[1])

        self._gpio_pub.publish(gpio_msg)

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
            self._hat.charger_state()
        telemetry_msg.motors_on = self._motors.motors_are_enabled()
        telemetry_msg.servos_on = self._servos.controller_powered()

        self._telemetry_pub.publish(telemetry_msg)

    def _process_sentence(self, sentence: str):
        """Determine if it's a SCREEN or a TELEM sentence."""
        if sentence:
            fields = sentence.split()

            if fields[0] == TELEM_HEADER:
                if len(fields) == 9:
                    self._publish_telemetry(fields)
                    self._telem_sentences += 1
                    return

            elif fields[0] == SCREEN_HEADER:
                if len(fields) == 3:
                    try:
                        screen = HAT_SCREEN(fields[1])
                        button = HAT_BUTTON(int(fields[2]))

                    except Exception as e:
                        self.get_logger().warning(
                            f'SCREEN_HEADER Exception: {e}')
                        return

                    if (screen != self._previous_screen
                            or button != HAT_BUTTON.NO_BUTTON):
                        self._hat.control_comms(enable=False)

                        if screen in [HAT_SCREEN.CONNECTIONS,
                                      HAT_SCREEN.DEVICES]:
                            page_text = self._network.process_screen_request(
                                screen, button)
                            self._hat.write_sentence(page_text)
                        elif screen == HAT_SCREEN.STATUS:
                            self._hat.show_status_msgs()

                        self._screen_sentences += 1
                        self._previous_screen = screen
                        self._hat.control_comms(enable=True)

                    return

        self.get_logger().warning(f'Unusable sentence {sentence}',
                                  throttle_duration_sec=60)
        self._sentence_errors += 1

    def _diags_cb(self, statuses):
        """Assemble the collection of diagnostics."""
        # TODO: Use something more informative for the Status
        statuses.summary(
            diagnostic_msgs.msg.DiagnosticStatus.OK,
            'Telemetry')
        statuses.add('telem_sentences', f'{self._telem_sentences}')
        statuses.add('screen_sentences', f'{self._screen_sentences}')
        statuses.add('sentence_errors', f'{self._sentence_errors}')

        return statuses

    def main(self):
        """Configure the serial port.

        Configure and then read incoming sentences in a loop.
        """
        self.get_logger().info(f'{self._node_name} v{VERSION} starting')
        self.setup_diags(diags_cb=self._diags_cb)

        self.get_logger().info(f'{self._node_name} spinning'
                               f' with timeout {self._timeout_sec}')

        self._hat.control_comms(enable=True)
        self._hat.charger_control(on=True)
        while ROSok():
            ROSspin_once(node=self, timeout_sec=0)
            sentence = self._hat.read_sentence()
            ROSspin_once(node=self, timeout_sec=0)

            if sentence:
                self._process_sentence(sentence)

            self._publish_gpio()
            if self._i2c_objects:
                for i2c_object in self._i2c_objects:
                    try:
                        self._i2c_objects[i2c_object].run_once()
                    except Exception as e:
                        self.get_logger().warn(
                            f'I2C {i2c_object} run_once() excepted: {e}'
                        )

        if self._i2c_objects:
            for i2c_object in self._i2c_objects:
                try:
                    self._i2c_objects[i2c_object].cleanup()
                except Exception as e:
                    self.get_logger().warn(
                        f'I2C {i2c_object} cleanup() excepted: {e}'
                    )
        self.get_logger().info(f'{self._node_name} stopping')
        self.destroy_node()
        ROSshutdown()
