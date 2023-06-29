from typing import Union
from time import sleep

import smbus2

import RPi.GPIO as GPIO
from roboquest_core.rq_servos_config import servo_config

SERVO_ENABLE_PIN = 23
I2C_BUS_ID = 1
#
# There's an unknown device at 0x70 on bus 1.
#
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

    def _translate(self, value, in_min, in_max, out_min, out_max) -> int:
        """
        Translate value somehow. The original implementation of this method
        was named 'map' which is an existing Python builtin function.
        """

        return ((value - in_min) * (out_max - out_min)
                / (in_max - in_min) + out_min)

    def _set_servo_pwm(self, channel: int, on: int, off: int) -> None:
        """
        Command the servo with a specific PWM signal, which was calculated
        based on a desired angle.
        """

        self._bus.write_byte_data(I2C_DEVICE_ID,
                                  PULSE0_ON_L_REG+4*channel,
                                  on & 0xFF)
        self._bus.write_byte_data(I2C_DEVICE_ID,
                                  PULSE0_ON_H_REG+4*channel,
                                  on >> 8)
        self._bus.write_byte_data(I2C_DEVICE_ID,
                                  PULSE0_OFF_L_REG+4*channel,
                                  off & 0xFF)
        self._bus.write_byte_data(I2C_DEVICE_ID,
                                  PULSE0_OFF_H_REG+4*channel,
                                  off >> 8)

    def set_servo_angle(self,
                        channel: Union[int, str],
                        angle: int = None) -> None:
        """
        For servo channel set its angle. If no angle is provided, set
        the default angle. The default is either the previously
        set angle or the init_angle_deg from the configuration.
        """

        if isinstance(channel, str):
            servo = self._name_map[channel]
        else:
            servo = self._servos[channel]

        if (self._controller_powered
                and self._servos_state[servo.channel]['enabled']):
            if angle is None:
                angle = self._servos_state[servo.channel]['angle']

            angle = self._constrain(servo.angle_min_deg,
                                    angle,
                                    servo.angle_max_deg)
            self._servos_state[servo.channel]['angle'] = angle

            # TODO: Figure out what this magic 0 means.
            off_pulse = self._translate(angle,
                                        0,
                                        servo.angle_max_deg,
                                        servo.pulse_min,
                                        servo.pulse_max)
            # TODO: And these magic numbers too.
            off_count = self._constrain(0,
                                        int(self._translate(off_pulse,
                                                            0,
                                                            20000,
                                                            0,
                                                            4095)),
                                        4095)

            on_count = 0
            self._set_servo_pwm(servo.channel, on_count, off_count)

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

            if servo.name != 'undefined':
                self._servos_state[channel]['enabled'] = True
                self.set_servo_angle(channel, servo.angle_init_deg)
                sleep(servo.init_delay_s)
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

        self._servos_state[channel]['enabled'] = True
        self.set_servo_angle(channel)

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
