"""gpio utility functions.

Utility functions and constants for manipulating and
monitoring GPIO pins.
"""

from enum import Enum

import gpiod
from gpiod.line import Direction, Value


GPIO_DEVICE = '/dev/gpiochip0'


class RQ_GPIO(Enum):
    """Pins used by the Roboquest application."""

    COMMS_ENABLE = 'GPIO22'
    FET_1_ENABLE = 'GPIO24'
    FET_2_ENABLE = 'GPIO25'
    CHARGE_BATTERY = 'GPIO21'
    CHARGER_POWERED = 'GPIO7'


def check_device():
    """Get information about a GPIO device."""
    is_gpio_device = gpiod.is_gpiochip_device(GPIO_DEVICE)
    if is_gpio_device:
        print(
            f'{GPIO_DEVICE} is an actual GPIO device'
        )

        with gpiod.Chip(GPIO_DEVICE) as chip:
            info = chip.get_info()
            print(
                f'{info.name} [{info.label}] ({info.num_lines} lines)'
            )


def set_pin(pin_name: RQ_GPIO = None, pin_state: str = None) -> str:
    """Set the state of a pin."""
    if pin_name:
        pin = pin_name.value
    else:
        return None

    if pin_state in ['on', 'active', 'high', '1']:
        pin_value = Value.ACTIVE
    else:
        pin_value = Value.INACTIVE
    with gpiod.request_lines(
        GPIO_DEVICE,
        consumer='rq_gpio_utils',
        config={
            pin: gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=pin_value
            )
        },
    ) as request:
        request.set_value(pin, pin_value)

    return pin_state


def get_pin(pin: RQ_GPIO = None) -> str:
    """Read the state of a pin."""
    return 'Not implemented'


def detect_change(pin: RQ_GPIO = None, change: str = None):
    """Watch for a specific state change on a pin."""
    pass


if __name__ == '__main__':
    import argparse

    def _parse_args() -> argparse.Namespace:
        """Get the arguments from the command line."""
        parser = argparse.ArgumentParser(
            prog='gpio_utils',
            description='Use the rq_gpio_utils functions',
            epilog=''
        )

        parser.add_argument(
            '--check_device',
            dest='check_device',
            action='store_true',
            help='Check the GPIO device'
        )
        parser.add_argument(
            '--pin_name',
            dest='pin_name',
            default=None,
            help='Name of the pin to control',
            type=str
        )
        parser.add_argument(
            '--pin_value',
            dest='pin_value',
            default=None,
            help='Set the pin value',
            type=str
        )

        return parser.parse_args()

    parsed_args = _parse_args()

    if parsed_args.check_device:
        check_device()

    if parsed_args.pin_name:
        try:
            pin = RQ_GPIO[parsed_args.pin_name]

        except KeyError:
            raise Exception(
                f'{parsed_args.pin_name} is not known'
            )

        if parsed_args.pin_value:
            print(
                f'Set {pin.value}: {set_pin(pin, parsed_args.pin_value)}'
            )
        else:
            print(
                f'Get {pin.value}: {get_pin(pin)}'
            )
