"""Use the Raspberry Pi GPIO and I2C devices to control the motors."""

from struct import pack
from time import sleep

import RPi.GPIO as GPIO

import smbus2

MAX_MOTOR_RPM = 300
MOTOR_ENABLE_PIN = 17
I2C_BUS_ID = 6
I2C_DEVICE_ID = 0x53

I2C_MOTOR_RIGHT_REGISTER = 3
I2C_MOTOR_LEFT_REGISTER = 4

WRITE_ERROR_WAIT_S = 0.01


class RQMotors(object):
    """
    Motor control.

    Manages the operation of the two drive motors connected to the I2C
    bus.
    """

    def __init__(self):
        """Configure the motor control sub-system for use."""
        self._write_errors = 0
        self.set_motor_max_rpm(MAX_MOTOR_RPM)

        self._setup_gpio()
        self._setup_i2c()

    def set_motor_max_rpm(self, max_rpm: int) -> None:
        """Set the maximum RPM of the motors as a positive value."""
        self._motor_max_rpm = max_rpm

    def _setup_gpio(self) -> None:
        """Initialize the GPIO subsystem.

        This class does not have exclusive control of the GPIO sub-system.
        """
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_ENABLE_PIN, GPIO.OUT)
        GPIO.output(MOTOR_ENABLE_PIN, GPIO.LOW)
        self._motors_enabled = False

    def _setup_i2c(self) -> None:
        """Initialize the I2C bus."""
        self._bus = smbus2.SMBus(I2C_BUS_ID)

    def motors_are_enabled(self) -> bool:
        """Return True when the motors are enabled."""
        return self._motors_enabled

    def enable_motors(self, enable: bool = False) -> None:
        """Enable or disable the motors."""
        if (enable and not self._motors_enabled):
            GPIO.output(MOTOR_ENABLE_PIN, GPIO.HIGH)
            sleep(0.2)
            self._motors_enabled = True

        if (not enable and self._motors_enabled):
            GPIO.output(MOTOR_ENABLE_PIN, GPIO.LOW)
            self._motors_enabled = False

    def _pack_rpm(self, rpm: int) -> bytes:
        """Pack a 4 byte signed integer into 4 bytes, little-endian."""
        return pack('<i', rpm)

    def _constrain_rpm(self, rpm: int) -> int:
        """Constrain range of RPM.

        Constrain RPM to be in
        [-self._motor_max_rpm, self._motor_max_rpm], clipping rpm
        if it is not.
        """
        return max(min(int(rpm),
                       self._motor_max_rpm),
                   -self._motor_max_rpm)

    def set_motors_rpm(self, right: int = 0, left: int = 0) -> bool:
        """Set RPM of motors.

        An open loop control for the velocity of each drive motor.
        right and left are the RPM to set for each motor. A positive
        value indicates the forward rotation.

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
                    list(self._pack_rpm(self._constrain_rpm(right)))
                )
                self._bus.write_i2c_block_data(
                    I2C_DEVICE_ID,
                    I2C_MOTOR_LEFT_REGISTER,
                    list(self._pack_rpm(self._constrain_rpm(left)))
                )

                return True

            except Exception:
                # TODO: What are the possible Exceptions?
                self._write_errors += 1
                sleep(WRITE_ERROR_WAIT_S)

        self.enable_motors(False)

        return False
