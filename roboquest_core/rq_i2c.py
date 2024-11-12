"""Use the Raspberry Pi I2C buses."""

import smbus2


class BusError(Exception):
    """Describe an error with a bus."""

    pass


class DeviceError(Exception):
    """Describe an error with a device."""

    pass


class RQI2CComms(object):
    """
    Communicate with I2C buses and devices.

    Manages the operation of I2C bus devices as a singleton
    object.
    """

    def __new__(cls):
        """Ensure a singlton."""
        if not hasattr(cls, 'instance'):
            cls.instance = super(RQI2CComms, cls).__new__(cls)
        return cls.instance

    def __init__(self):
        """Initialize the data structure."""
        self._buses = {
        }

    def add_device(self, bus_ID: int, device_ID: int) -> None:
        """Initialize communication with a device.

        If bus_ID isn't already initialized, do so now. Record
        the relationship between the device and bus.
        """
        if bus_ID not in self._buses:
            try:
                self._buses[bus_ID] = {
                    'bus': smbus2.SMBus(bus_ID),
                    'devices':  [device_ID]
                }

                return

            except Exception:
                raise BusError(f'Bus {bus_ID} excepted')

        if device_ID not in self._buses[bus_ID]['devices']:
            self._buses[bus_ID]['devices'].append(device_ID)

            return

        raise DeviceError(f'Device {device_ID} already added')

    def write_block_data(
         self,
         bus_ID: int,
         device_ID: int,
         register_ID: int,
         payload: list) -> None:
        """Write the payload.

        Write the bytes in payload to the bus_ID bus, for
        device_ID device, to register_ID register.
        """
        if bus_ID not in self._buses:
            raise BusError(f'Bus {bus_ID} not setup')
        if device_ID not in self._buses[bus_ID]['devices']:
            raise DeviceError(f'Device {device_ID} not setup')

        try:
            self._buses[bus_ID]['bus'].write_i2c_block_data(
                device_ID,
                register_ID,
                payload
            )

        except Exception as e:
            raise BusError(
                f'Bus {bus_ID}'
                f', device {device_ID}'
                f', register {register_ID}'
                f' exception {e.msg}'
            )
