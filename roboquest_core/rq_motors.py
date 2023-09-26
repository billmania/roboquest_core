from struct import pack
from time import sleep

import smbus2

import RPi.GPIO as GPIO

MOTOR_ENABLE_PIN = 17
I2C_BUS_ID = 6
I2C_DEVICE_ID = 0x53

I2C_MOTOR_RIGHT_REGISTER = 3
I2C_MOTOR_LEFT_REGISTER = 4

WRITE_ERROR_WAIT_S = 0.01


class RQMotors(object):
    """
    Manages the operation of the two drive motors connected to the I2C
    bus.
    """

    def __init__(self):
        """
        Configure the motor control sub-system for use.
        """

        self._write_errors = 0
        self.set_motor_max_speed(100)

        self._setup_gpio()
        self._setup_i2c()

    def set_motor_max_speed(self, max_speed: int) -> None:
        """
        Set the maximum speed of the motors as a positive percentage.
        """

        self._motor_max_speed = max_speed

    def _setup_gpio(self) -> None:
        """
        Initialize the GPIO subsystem.
        """

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_ENABLE_PIN, GPIO.OUT)
        GPIO.output(MOTOR_ENABLE_PIN, GPIO.LOW)
        self._motors_enabled = False

    def _setup_i2c(self) -> None:
        """
        Initialize the I2C bus.
        """

        self._bus = smbus2.SMBus(I2C_BUS_ID)

    def motors_are_enabled(self) -> bool:
        """
        Returns True when the motors are enabled.
        """

        return self._motors_enabled

    def enable_motors(self, enable: bool = False) -> None:
        """
        Enable or disable the motors.
        """

        if (enable and not self._motors_enabled):
            GPIO.output(MOTOR_ENABLE_PIN, GPIO.HIGH)
            sleep(0.2)
            self._motors_enabled = True

        if (not enable and self._motors_enabled):
            GPIO.output(MOTOR_ENABLE_PIN, GPIO.LOW)
            self._motors_enabled = False

    def _pack_speed(self, speed: int) -> bytes:
        """
        Pack a 4 byte signed integer into 4 bytes, little-endian.
        """

        return pack('<i', speed)

    def _constrain_speed(self, speed: int) -> int:
        """
        Constrain speed to be in
        [-self._motor_max_speed, self._motor_max_speed], clipping speed
        if it is not.
        """

        return max(min(int(speed),
                       self._motor_max_speed),
                   -self._motor_max_speed)

    def set_motors_velocity(self, right: int = 0, left: int = 0) -> bool:
        """
        An open loop control for the velocity of each drive motor.
        right and left are the percentage of full power to apply
        to each motor. A positive percentage indicates the forward
        rotation. Values outside the range [-100, 100] will be
        clipped.

        The velocity of the right motor is always set first, which
        will likely have an impact on the robot's motion.
        """

        if not self._motors_enabled:
            return False

        tries = 3
        while tries:
            tries -= 1

            try:
                self._bus.write_i2c_block_data(
                    I2C_DEVICE_ID,
                    I2C_MOTOR_RIGHT_REGISTER,
                    list(self._pack_speed(self._constrain_speed(right)))
                )
                self._bus.write_i2c_block_data(
                    I2C_DEVICE_ID,
                    I2C_MOTOR_LEFT_REGISTER,
                    list(self._pack_speed(self._constrain_speed(left)))
                )

                return True

            except Exception:
                # TODO: What are the possible Exceptions?
                self._write_errors += 1
                sleep(WRITE_ERROR_WAIT_S)

        self.enable_motors(False)

        return False
