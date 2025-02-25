"""Test the servo control system.

Exercise the servos to either expose defects or prove
functionality.
"""

from os import getpid, kill
from signal import SIGKILL
from threading import Timer
from time import time

import rclpy.logging
from rclpy import shutdown as ROSshutdown
from rclpy import spin as ROSspin
from rclpy.parameter import Parameter

from roboquest_core.rq_config_file import ConfigFile
from roboquest_core.rq_hat import HAT_BUTTON, HAT_SCREEN
from roboquest_core.rq_hat import RQHAT
from roboquest_core.rq_hat import SCREEN_HEADER, TELEM_HEADER
from roboquest_core.rq_i2c import RQI2CComms
from roboquest_core.rq_i2c_modules import I2CSupport
from roboquest_core.rq_node import RQNode
from roboquest_core.rq_servos import RQServos
from roboquest_core.rq_servos import ServoError, TranslateError
from roboquest_core.rq_servos_config import servo_config

from rq_msgs.msg import Servos
from rq_msgs.srv import Control


VERSION = '25'

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
LOOP_PERIOD = 0.5
SPEED_SET_PERIOD = 2.0
SERVO_ID = 8
MIN_SERVO = 10
CENTER_SERVO = 90
MAX_SERVO = 170
SERVO_INCREMENT = 2
SERVO_SPEED = 3
SERVO_MODE = COMMAND_INCR


class RQServoTest(RQNode):
    """Test the servos.

    Exercises the servo sub-system.
    It requires rclpy.init() has already been called.
    """

    def __init__(self,
                 node_name: str = 'RQServoTest'):
        """Initialize the object."""
        super().__init__(node_name)
        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.DEBUG)
        self._setup_parameters()

        self._setup_ros_graph()

        self._timeout_sec = 1 / self._parameters['hat_comms_read_hz']
        self._hat = RQHAT(port=self._parameters['hat_port'],
                          data_rate=self._parameters['hat_data_rate'],
                          data_bits=self._parameters['hat_data_bits'],
                          parity=self._parameters['hat_parity'],
                          stop_bits=self._parameters['hat_stop_bits'],
                          read_timeout_sec=self._timeout_sec,
                          serial_errors_cb=None)
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
        self._control_srv = self.create_service(Control,
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

    def _process_sentence(self, sentence: str):
        """Determine if it's a SCREEN or a TELEM sentence."""
        if sentence:
            fields = sentence.split()

            if fields[0] == TELEM_HEADER:
                if len(fields) == 9:
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

    def _loop_callback(self):
        """Run the loop logic."""
        if SERVO_MODE == COMMAND_ANGLE:
            self._angle += self._increment
            self._servos.set_servo_angle(
                SERVO_ID,
                self._angle)

            if self._angle > MAX_SERVO:
                self.get_logger().info(
                    f'MAX angle: {self._angle}'
                    f', increment: {self._increment}'
                )
                self._increment = -(self._increment)
                self._angle = MAX_SERVO
            elif self._angle < MIN_SERVO:
                self.get_logger().info(
                    f'MIN angle: {self._angle}'
                    f', increment: {self._increment}'
                )
                self._increment = -(self._increment)
                self._angle = MIN_SERVO

        elif SERVO_MODE == COMMAND_INCR:
            self.get_logger().info(
                f"INCR: {self._servo_state['angle']}"
                f', increment: {self._increment}'
            )
            if self._servo_state['angle'] > MAX_SERVO:
                self._increment = -(self._increment)
            elif self._servo_state['angle'] < MIN_SERVO:
                self._increment = -(self._increment)
            self._servos.incr_servo_angle(
                SERVO_ID,
                self._increment)

        elif SERVO_MODE == COMMAND_SPEED:
            current_time = time()
            if (current_time - self._speed_last_set) < SPEED_SET_PERIOD:
                return
            else:
                self._speed_last_set = current_time

            self.get_logger().info(
                f"SPEED: {self._servo_state['angle']}"
                f', increment: {self._speed}'
            )
            if self._servo_state['angle'] > MAX_SERVO:
                self._speed = -(self._speed)
            elif self._servo_state['angle'] < MIN_SERVO:
                self._speed = -(self._speed)
            self._servos.set_servo_speed(
                SERVO_ID,
                self._speed)

    def main(self):
        """Configure the serial port.

        Configure and then read incoming sentences in a loop.
        """
        self.get_logger().info(f'{self._node_name} v{VERSION} starting')

        self.get_logger().info(f'{self._node_name} spinning'
                               f' with timeout {self._timeout_sec}')

        self._hat.control_comms(enable=True)
        self._hat.charger_control(on=True)
        self._servos.set_power(False)
        self._servos.set_power(True)

        self._servo_state = self._servos._servos_state_list[SERVO_ID]
        self._angle = CENTER_SERVO
        self._servos.set_servo_angle(
            SERVO_ID,
            self._angle)
        self.get_logger().info(f'Commanded {self._angle}'
                               f", Reported{self._servo_state['angle']}")
        self._speed = SERVO_SPEED
        self._increment = SERVO_INCREMENT

        self._speed_last_set = time()
        self.create_timer(LOOP_PERIOD, self._loop_callback)

        try:
            ROSspin(node=self)

        except KeyboardInterrupt:
            self._servos.set_power(False)

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
