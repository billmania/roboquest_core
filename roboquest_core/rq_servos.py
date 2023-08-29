from typing import Union
from time import sleep

import smbus2

import RPi.GPIO as GPIO
from roboquest_core.rq_servos_config import servo_config

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


class TranslateError(Exception):
    """
    Errors in calls to or operation of the _translate() method.
    """

    pass


class MotionError(Exception):
    """
    Errors in calls to or operation of the _slow_motion() method.
    """

    pass


class RQServos(object):
    """
    Manages the operation of the PCA9685 controller connected to the I2C
    bus.
    """

    def __init__(self):
        """
        Setup communication with the servo sub-system. The PCA9685 I2C
        servo controller isn't configured until it's powered and then
        each time the power is cycled.
        """

        self._write_errors = 0

        self._servos, self._name_map, self._servos_state = servo_config()
        self._controller_powered = False

        self._setup_gpio()
        self._setup_i2c()

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

    def set_servo_angle(self,
                        channel: Union[int, str],
                        angle: int = None) -> int:
        """
        For servo channel set its angle. If no angle is provided, set
        the default angle. The default is the previously set angle.

        The flow is from an angle in degrees, to a pulse duration
        in microseconds, and finally to a register value in "counts".

        The return value is the angle to which the servo was actually
        moved, after limiting the acceleration.
        """

        if isinstance(channel, str):
            servo = self._name_map[channel]
        else:
            servo = self._servos[channel]

        if (self._controller_powered
                and self._servos_state[servo.channel]['enabled']):
            if angle is None:
                angle = self._servos_state[servo.channel]['angle']

            angle = self._constrain(servo.joint_angle_min_deg,
                                    angle,
                                    servo.joint_angle_max_deg)

            #
            # This is the spot where the _slow_motion() method could be
            # inserted.
            #

            pulse_duration_ms = self._translate(
                angle,
                servo.servo_angle_min_deg,
                servo.servo_angle_max_deg,
                servo.pulse_min_us,
                servo.pulse_max_us)

            pulse_off_count = self._translate(
                pulse_duration_ms,
                MIN_PULSE_US,
                MAX_PULSE_US,
                MIN_COUNT,
                MAX_COUNT)

            self._set_servo_pwm(
                servo.channel,
                PULSE_ON_COUNT,
                round(pulse_off_count))
            self._servos_state[servo.channel]['angle'] = angle

            return angle

    def _pca9685_init(self):
        """
        Configure the PCA9685 for use, usually each time power is applied.

        MODE1 to wake
        MODE2 to totem pole
        PRESCALE to 50 Hz

        Set the initial angle of each defined servo according to the
        configuration parameters.
        """

        for register, value in SETUPS:
            self._bus.write_byte_data(I2C_DEVICE_ID, register, value)
            sleep(0.05)

        for servo in self._servos:
            channel = servo.channel

            if servo.joint_name:
                self._servos_state[channel]['enabled'] = True
                #
                # There isn't a way to know the servo's current
                # angle so the following call may cause high acceleration
                # of the servo angle.
                #
                self.set_servo_angle(
                    channel,
                    servo.joint_angle_init_deg)
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

        self._servos_state[channel]['enabled'] = False
        self._set_servo_pwm(channel, 0, 0)

    def restore_servo_angle(self, channel: int) -> None:
        """
        Set the servo to its most recent angle, to recover from the
        PWM signal having been flat-lined by disable_servo().
        """

        self.set_servo_angle(channel)
        self._servos_state[channel]['enabled'] = True

    def set_power(self, enable: bool = False) -> None:
        """
        Enable or disable power to the servo controller.
        """

        if (enable and not self._controller_powered):
            GPIO.output(SERVO_ENABLE_PIN, GPIO.HIGH)
            sleep(0.5)
            self._controller_powered = True
            self._pca9685_init()

        if (not enable and self._controller_powered):
            GPIO.output(SERVO_ENABLE_PIN, GPIO.LOW)
            self._controller_powered = False

    def _constrain(self, min_value: int, value: int, max_value: int) -> int:
        """
        Clip value to be between min and max, inclusive.
        """

        return max(min(value, max_value), min_value)

    def _slow_motion(
            self,
            from_angle: int,
            to_angle: int,
            step_amount: int = ANGLE_STEP_DEG) -> int:
        """
        Since the servo controller doesn't provide a means to adjust
        the speed of the servo angle change or to reduce the acceleration,
        this method will limit the amount of angle change per cycle,
        as a crude way of reducing the acceleration.
        from_angle is the starting point and to_angle is the destination.
        step_amount is the maximum angle to change per cycle.

        The return value is the next angle for the servo.
        """

        if (not 0 <= from_angle <= 180
                or not 0 <= to_angle <= 180):
            raise MotionError("from or to out of range")

        if from_angle == to_angle:
            return to_angle

        distance = from_angle - to_angle
        if abs(distance) > step_amount:
            if from_angle > to_angle:
                next_position = from_angle - step_amount
            else:
                next_position = from_angle + step_amount
        else:
            next_position = to_angle

        return next_position
