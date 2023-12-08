from typing import Union, List
from time import sleep, time
from threading import Thread, Lock, Event

import smbus2

import RPi.GPIO as GPIO
from roboquest_core.rq_servos_config import servo_map_and_state
from roboquest_core.rq_servos_config import SERVO_QTY, Servo

SERVO_ENABLE_PIN = 23
I2C_BUS_ID = 1
I2C_DEVICE_ID = 0x40

WRITE_ERROR_WAIT_S = 0.01

#
# PCA9685 registers
#
# TODO: Convert to an Enum or ROS parameters
MODE1_REG = 0x00
MODE2_REG = 0x01
PRESCALE_REG = 0xfe
#
# The registers, low and high byte, controlling the Pulse Width.
#
PULSE0_ON_L_REG = 0x06
PULSE0_ON_H_REG = 0x07
PULSE0_OFF_L_REG = 0x08
PULSE0_OFF_H_REG = 0x09

ANGLE_STEP_DEG = 10
MIN_PULSE_US = 0
MAX_PULSE_US = 20000
MIN_COUNT = 0
MAX_COUNT = 4095
PULSE_ON_COUNT = MIN_COUNT
MOVE_PERIOD_S = 5.0
INIT_DELAY_S = 0.05

#
# PCA9685 constants
#
# TODO: Convert to an Enum or ROS parameters
OUTDRV_VALUE = 0x04  # totem pole outputs
PRESCALE_VALUE = 0x79
MODE1_VALUE = 0x00

SETUPS = [(MODE2_REG, OUTDRV_VALUE),
          (PRESCALE_REG, PRESCALE_VALUE),
          (MODE1_REG, MODE1_VALUE)]


class ServoError(Exception):
    """
    General errors with servos.
    """

    pass


class TranslateError(Exception):
    """
    Errors in calls to or operation of the _translate() method.
    """

    pass


class RQServos(object):
    """
    Manages the operation of the PCA9685 controller connected to the I2C
    bus.
    """

    def __init__(self, servos_list: List[dict], logger=print):
        """
        Setup communication with the servo sub-system. The PCA9685 I2C
        servo controller isn't configured until it's powered and then
        each time the power is cycled.
        """

        self._write_errors = 0

        self._servos_list = servos_list
        self._logger = logger
        #
        # self._servos_list is a list of Servo objects, indexed by their
        # position in the list. Their index position corresponds with their
        # servo channel number.
        #
        # self._servo_name_map is a dictionary of Servo objects, keyed by the
        # servo joint_name. It's a map from the joint_name to the Servo object.
        #
        # self._servos_state_list is a list of objects (not Servo objects)
        # which track the current state of the servo. These objects are indexed
        # by their position in the list, which corresponds again with their
        # servo channel number.
        #
        self._servo_name_map, self._servos_state_list = (
            servo_map_and_state(self._servos_list)
        )
        self._controller_powered = False

        self._setup_gpio()
        self._setup_i2c()
        self._servo_changed = Event()
        self._servo_changed.clear()
        self._servo_lock = Lock()
        self._servo_period = Thread(
            group=None,
            target=self._time_servos,
            name='servos_timer',
            daemon=None)
        self._servo_worker = Thread(
            group=None,
            target=self._move_servos,
            name='servos_worker',
            daemon=None)
        self._servo_worker.start()
        self._servo_period.start()

    def _setup_gpio(self) -> None:
        """
        Initialize the GPIO subsystem.
        """

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_ENABLE_PIN, GPIO.OUT)
        GPIO.output(SERVO_ENABLE_PIN, GPIO.LOW)

    def _setup_i2c(self) -> None:
        """
        Initialize the I2C bus.
        """

        self._bus = smbus2.SMBus(I2C_BUS_ID)

    def _translate(self,
                   input_value: int,
                   input_min: int, input_max: int,
                   output_min: int, output_max: int) -> int:
        """
        input_value must be in the range [input_min, input_max].
        Map input_value onto the range [output_min, output_max].
        Return the mapped value.
        """

        if not output_min < output_max:
            raise TranslateError("output range error"
                                 f" range {output_min}, {output_max}")
        if not input_min < input_max:
            raise TranslateError("input range error"
                                 f" range {input_min}, {input_max}")
        if not input_min <= input_value <= input_max:
            raise TranslateError(f"input_value {input_value} not within"
                                 f" range {input_min}, {input_max}")

        return ((input_value - input_min) * (output_max - output_min)
                / (input_max - input_min) + output_min)

    def _set_servo_pwm(
            self,
            channel: int,
            on_count: int,
            off_count: int) -> None:
        """
        Command the servo with a specific PWM signal, which was calculated
        based on a desired angle.
        """

        with self._servo_lock:
            self._bus.write_byte_data(I2C_DEVICE_ID,
                                      PULSE0_ON_L_REG+4*channel,
                                      on_count & 0xFF)
            self._bus.write_byte_data(I2C_DEVICE_ID,
                                      PULSE0_ON_H_REG+4*channel,
                                      on_count >> 8)
            self._bus.write_byte_data(I2C_DEVICE_ID,
                                      PULSE0_OFF_L_REG+4*channel,
                                      off_count & 0xFF)
            self._bus.write_byte_data(I2C_DEVICE_ID,
                                      PULSE0_OFF_H_REG+4*channel,
                                      off_count >> 8)

    def _get_servo(
            self,
            channel: Union[int, str]
            ) -> Servo:
        """
        Using the channel as key, retrieve and return the servo
        object.
        """

        try:
            channel_number = int(channel)
            if 0 <= channel_number < SERVO_QTY:
                servo = self._servos_list[channel_number]
            else:
                raise ServoError(
                    f"set_servo_angle: {channel_number} must be"
                    f" between 0 and {SERVO_QTY}"
                )

        except ValueError:
            if channel in self._servo_name_map:
                servo = self._servo_name_map[channel]
            else:
                raise ServoError(
                    f"set_servo_angle: {channel} not a recognized servo name"
                )

        return servo

    def _time_servos(self) -> None:
        """
        Set the _servo_changed Event every MOVE_PERIOD_S seconds.
        """

        while True:
            self._servo_changed.set()
            sleep(MOVE_PERIOD_S)

    def _move_servos(self) -> None:
        """
        Executed by a separate thread and controlled by an Event.

        Loops through self._servos_state_list to set the commanded angles
        and update the current angle for each servo.  Since there isn't
        any feedback from the servo about its current angle and there
        isn't any information about the servo's rate of loaded or
        unloaded change, the record of the servo's current angle is a
        crude guess at best.
        """

        while True:
            self._servo_changed.wait()
            self._servo_changed.clear()
            if not self._controller_powered:
                continue

            for channel, servo_state in enumerate(self._servos_state_list):
                if not servo_state['enabled']:
                    continue

                servo = self._servos_list[channel]
                command_angle = (
                    servo_state['command_angle']
                )

                if command_angle == servo_state['angle']:
                    servo_state['command_timestamp'] = time()
                    continue

                try:
                    pulse_duration_ms = self._translate(
                        command_angle,
                        servo['servo_angle_min_deg'],
                        servo['servo_angle_max_deg'],
                        servo['pulse_min_us'],
                        servo['pulse_max_us'])

                    pulse_off_count = self._translate(
                        pulse_duration_ms,
                        MIN_PULSE_US,
                        MAX_PULSE_US,
                        MIN_COUNT,
                        MAX_COUNT)

                except TranslateError as e:
                    self._logger(
                        f'_move_servos: {e}'
                    )
                    continue

                self._logger(
                    f'_move_servos: channel {channel}, angle {command_angle}'
                )
                self._set_servo_pwm(
                    channel,
                    PULSE_ON_COUNT,
                    round(pulse_off_count))
                servo_state['angle'] = command_angle
                servo_state['command_timestamp'] = time()

    def set_servo_speed(
            self,
            channel: Union[int, str],
            degrees_per_sec: float = 0.0) -> None:
        """
        Cause the servo to move away from its current position at the
        rate degrees_per_sec until stopped or a limit is reached. A
        thread is used to continue the motion until it's stopped.
        Motion can be stopped by any of: setting degrees_per_sec to
        0.0; calling incr_servo_angle; calling set_servo_angle.
        """
        # TODO: Implement
        pass

    def incr_servo_angle(
            self,
            channel: Union[int, str],
            increment_deg: int = 0) -> int:
        """
        Retrieve the current servo angle, change it by the
        signed value in increment_deg, and set that new
        angle.
        """

        servo = self._get_servo(channel)
        new_angle = (self._servos_state_list[servo['channel']]['angle'] +
                     increment_deg)
        angle = self._constrain(servo['joint_angle_min_deg'],
                                new_angle,
                                servo['joint_angle_max_deg'])
        self._servos_state_list[servo['channel']]['command_angle'] = angle
        self._servo_changed.set()

    def set_servo_angle(self,
                        channel: Union[int, str],
                        angle: int = None) -> None:
        """
        Servos can be identified by: a string name; a string representation
        of the channel number; or an integer channel number.

        For servo channel set its angle. If no angle is provided, set
        the default angle. The default is the previously set angle.
        """

        servo = self._get_servo(channel)
        if angle is None:
            angle = self._servos_state_list[servo['channel']]['angle']

        new_command_angle = self._constrain(
            servo['joint_angle_min_deg'],
            angle,
            servo['joint_angle_max_deg']
        )

        servo_state = self._servos_state_list[servo['channel']]
        if new_command_angle == servo_state['command_angle']:
            return

        servo_state['command_angle'] = new_command_angle
        self._servo_changed.set()

    def _pca9685_init(self):
        """
        Configure the PCA9685 for use, usually each time power is applied.

        Set the initial angle of each defined servo according to the
        configuration parameters.
        """

        with self._servo_lock:
            for register, value in SETUPS:
                self._bus.write_byte_data(I2C_DEVICE_ID, register, value)
                sleep(INIT_DELAY_S)

        for channel, servo in enumerate(self._servos_list):
            if servo['joint_name']:
                self._servos_state_list[channel]['enabled'] = True
                #
                # There isn't a way to know the servo's current
                # angle so the following call may cause high acceleration
                # of the servo angle.
                #
                try:
                    servo_state = self._servos_state_list[channel]
                    self.set_servo_angle(
                        channel,
                        servo['joint_angle_init_deg'])
                    servo_state['angle'] = servo['joint_angle_init_deg']
                    servo_state['command_angle'] = (
                        servo['joint_angle_init_deg']
                    )
                    servo_state['command_timestamp'] = time()

                except TranslateError:
                    servo_state['angle'] = None
                    servo_state['command_angle'] = None
                    servo_state['command_timestamp'] = None
            else:
                self.disable_servo(channel)

    def controller_powered(self) -> bool:
        """
        Returns True when the servo controller is powered.
        """

        return self._controller_powered

    def disable_servo(self, channel: int) -> None:
        """
        Flat-line the PWM signal to the servo and prevent a new angle
        from being set by set_servo_angle(). The effect of this method
        is undone by restore_servo_angle().
        """

        self._servos_state_list[channel]['enabled'] = False
        self._set_servo_pwm(channel, 0, 0)

    def restore_servo_angle(self, channel: int) -> None:
        """
        Set the servo to its most recent angle, to recover from the
        PWM signal having been flat-lined by disable_servo().
        """

        self.set_servo_angle(channel)
        self._servos_state_list[channel]['enabled'] = True

    def set_power(self, enable: bool = False) -> None:
        """
        Enable or disable power to the servo controller.
        """

        if (enable and not self._controller_powered):
            GPIO.output(SERVO_ENABLE_PIN, GPIO.HIGH)
            sleep(INIT_DELAY_S)
            self._pca9685_init()
            self._controller_powered = True

        if (not enable and self._controller_powered):
            self._controller_powered = False
            GPIO.output(SERVO_ENABLE_PIN, GPIO.LOW)

    def _constrain(self, min_value: int, value: int, max_value: int) -> int:
        """
        Clip value to be between min and max, inclusive.
        """

        return max(min(value, max_value), min_value)
