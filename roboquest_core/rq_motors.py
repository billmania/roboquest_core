"""Use the Raspberry Pi GPIO and I2C devices to control the motors."""

from struct import pack
from time import sleep

import RPi.GPIO as GPIO

from roboquest_core.rq_i2c import BusError, DeviceError
from roboquest_core.rq_i2c import RQI2CComms

MAX_MOTOR_RPM = 300
MOTOR_ENABLE_PIN = 17
#
# This constant is only a default and can be overridden by i2c.yaml.
#
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

    def __init__(self, i2c_bus_id: int = I2C_BUS_ID, ros_logger=None):
        """Configure the motor control sub-system for use."""
        self._i2c_bus_id = i2c_bus_id
        self._ros_logger = ros_logger
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
        """Initialize use of the I2C bus."""
        try:
            self._i2c = RQI2CComms()
            self._i2c.add_device(self._i2c_bus_id, I2C_DEVICE_ID)

        except BusError as e:
            self._ros_logger().warn(f'_setup_i2c: BusError({e})')
            raise e

        except DeviceError as e:
            self._ros_logger().warn(f'_setup_i2c: DeviceError({e})')
            raise e

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
                self._i2c.write_block_payload(
                    self._i2c_bus_id,
                    I2C_DEVICE_ID,
                    I2C_MOTOR_RIGHT_REGISTER,
                    list(self._pack_rpm(self._constrain_rpm(right)))
                )
                self._i2c.write_block_payload(
                    self._i2c_bus_id,
                    I2C_DEVICE_ID,
                    I2C_MOTOR_LEFT_REGISTER,
                    list(self._pack_rpm(self._constrain_rpm(left)))
                )

                return True

            except Exception as e:
                # TODO: What are the possible Exceptions?
                self._ros_logger().warn(f'set_motors_rpm: Exception({e})')
                self._write_errors += 1
                sleep(WRITE_ERROR_WAIT_S)

        self.enable_motors(False)

        return False
