"""Template for a class which manages an I2C device."""


class RQI2CDevice(object):
    """Control an I2C device."""

    def __init__(self, device_name: str):
        """Initialize the object.

        Handle everything needed to setup the object, but
        don't do anything with the I2C bus.
        """
        self._device_name = device_name
        print(f'Initialized object for {self._device_name}')

        pass

    def setup(self) -> None:
        """Initialize the device.

        The I2C bus is available when this method is called.
        """
        print(f'Setup device {self._device_name}')

    def run_once(self) -> None:
        """Run once each time through the RQManage loop.

        Communicate with the I2C device in a non-blocking manner.
        """
        print(f'Ran device {self._device_name} once')

    def cleanup(self) -> None:
        """Stop the device.

        The I2C bus is still available when this method is called.
        """
        print(f'Cleanup for device {self._device_name}')
