"""Control of the user GPIO pins."""
from enum import Enum
from typing import List, Tuple

import RPi.GPIO as GPIO


class USER_GPIO_PIN(Enum):
    """GPIO pins available to the user.

    These are BCM numbers, not board numbers.
    """

    GPIO6 = 6
    GPIO16 = 16
    GPIO19 = 19
    GPIO20 = 20
    GPIO26 = 26


class PinError(Exception):
    """Describe error in pin configuration."""

    pass


class UserGPIO(object):
    """Control user GPIO pins.

    Provide user control of the GPIO pins.
    """

    def __init__(self):
        """Initialize the structures."""
        self._unused_pins = set()
        self._output_pins = set()
        self._input_pins = set()

        for pin in USER_GPIO_PIN:
            self._unused_pins.add(pin)

    def _invalid_pin(self, pin: USER_GPIO_PIN) -> bool:
        """Check the validity of a pin."""
        return False if pin in USER_GPIO_PIN else True

    def clear_pin(self, pin: USER_GPIO_PIN) -> None:
        """Make a pin unused.

        Ensure a pin is marked as unused. Calling this method on an already
        unused pin is not an error.
        """
        if self._invalid_pin(pin):
            raise PinError(f'{pin} not a valid pin')

        if pin in self._unused_pins:
            return

        if pin in self._output_pins:
            self._output_pins.remove()
        else:
            self._input_pins.remove()

        GPIO.cleanup(pin.value)
        self._unused_pins.add(pin)

    def make_output(
         self,
         pin: USER_GPIO_PIN) -> Exception:
        """Set a GPIO pin for output."""
        if self._invalid_pin(pin):
            raise PinError(f'{pin} not a valid pin')

        if pin in self._unused_pins:
            self._unused_pins.remove(pin)
            self._output_pins.add(pin)
            GPIO.setup(pin.value, GPIO.OUT)
        else:
            if pin in self._input_pins:
                error = f'{pin.name} already set as input'
            else:
                error = f'{pin.name} already set as output'

            raise PinError(error)

    def make_input(
         self,
         pin: USER_GPIO_PIN) -> Exception:
        """Set a GPIO pin for input."""
        if self._invalid_pin(pin):
            raise PinError(f'{pin} not a valid pin')

        if pin in self._unused_pins:
            self._unused_pins.remove(pin)
            self._input_pins.add(pin)
            GPIO.setup(pin.value, GPIO.IN)
        else:
            if pin in self._input_pins:
                error = f'{pin.name} already set as input'
            else:
                error = f'{pin.name} already set as output'

            raise PinError(error)

    def set_pin(self, pin: USER_GPIO_PIN, state: int = GPIO.LOW) -> Exception:
        """Set the value of an output pin.

        Raise an exception if the pin isn't configured for output.
        """
        if self._invalid_pin(pin):
            raise PinError(f'{pin} not a valid pin')

        if pin not in self._output_pins:
            raise PinError(f'{pin.name} not an output pin')

        GPIO.output(pin.value, state)

    def read_input_pins(self) -> List[Tuple]:
        """Read the input pins.

        Read the current value of each pin already configured as an input.
        Return a list of tuples containing the pin and its value.
        """
        input_pin_values = []
        for pin in self._input_pins:
            input_pin_values.append((pin, GPIO.input(pin.value)))

        return input_pin_values
