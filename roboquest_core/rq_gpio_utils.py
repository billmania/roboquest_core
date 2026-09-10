"""gpio utility functions.

Utility functions and constants for manipulating and
monitoring GPIO pins.
"""

import threading
from datetime import timedelta
from enum import Enum
from sys import exit as sys_exit
from typing import Callable


import gpiod
from gpiod.line import Bias, Direction, Edge, Value


GPIO_DEVICE = '/dev/gpiochip0'
VERSION = 1

detector = None


class RQ_GPIO(Enum):
    """Pins used by the Roboquest application.

    Pin names beginning with a 'U' are directly accessible to the
    user.
    """

    COMMS_ENABLE = 'GPIO22'
    FET_1_ENABLE = 'GPIO24'
    FET_2_ENABLE = 'GPIO25'
    CHARGE_BATTERY = 'GPIO21'
    CHARGER_POWERED = 'GPIO7'
    MOTOR_ENABLE = 'GPIO17'
    SHUTDOWN = 'GPIO27'
    UGPIO6 = 'GPIO6'
    UGPIO16 = 'GPIO16'
    UGPIO19 = 'GPIO19'
    UGPIO20 = 'GPIO20'
    UGPIO26 = 'GPIO26'


class USER_GPIO_PIN(Enum):
    """GPIO pins available to the user.

    The names of each member of the Enum match the attribute
    names in the GPIOOutput and GPIOInput ROS interface names,
    for historical reasons.
    """

    gpio6 = RQ_GPIO['UGPIO6']
    gpio16 = RQ_GPIO['UGPIO16']
    gpio19 = RQ_GPIO['UGPIO19']
    gpio20 = RQ_GPIO['UGPIO20']
    gpio26 = RQ_GPIO['UGPIO26']


class GPIOEdgeDetector:
    """Detect a rising edge.

    Watches pin_name for a rising-edge transition.
    This class uses a threading.Thread to busy-loop check
    the state of the pin. When the rising edge is detected,
    the callback function is called. If the callback returns,
    the thread will exit.
    """

    def __init__(
        self,
        pin_name: RQ_GPIO = None,
        callback: Callable[[gpiod.EdgeEvent], None] = None,
        bias: Bias = Bias.AS_IS,
        debounce_us: int = 0
    ):
        """Create the detector."""
        if pin_name and callback:
            self._pin = pin_name.value
        else:
            return None

        self._callback = callback
        self._stop_event = threading.Event()
        self.done = False

        self._request = gpiod.request_lines(
            GPIO_DEVICE,
            consumer='rq_gpio_utils',
            config={
                self._pin: gpiod.LineSettings(
                    direction=Direction.INPUT,
                    edge_detection=Edge.RISING,
                    bias=bias,
                    debounce_period=timedelta(microseconds=debounce_us)
                )
            },
        )
        self._offset = self._request.offsets[0]

        self._thread = threading.Thread(target=self._watch_loop, daemon=True)

    def start(self) -> None:
        """Start the detector thread."""
        self._thread.start()

    def stop(self) -> None:
        """Stop the detector thread."""
        self._stop_event.set()
        self._thread.join()
        self._request.release()

    def _watch_loop(self) -> None:
        while not self._stop_event.is_set():
            #
            # Poll with a timeout so the stop flag gets checked periodically
            # rather than blocking forever on wait_edge_events().
            #
            if self._request.wait_edge_events(
                timeout=timedelta(milliseconds=200)
            ):
                for event in self._request.read_edge_events():
                    if event.line_offset == self._offset:
                        self._callback(event)
                sys_exit(0)


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


def get_pin(pin_name: RQ_GPIO = None) -> str:
    """Read the state of a pin."""
    if pin_name:
        pin = pin_name.value
    else:
        return None

    with gpiod.request_lines(
        GPIO_DEVICE,
        consumer='rq_gpio_utils',
        config={
            pin: gpiod.LineSettings(
                direction=Direction.INPUT
            )
        },
    ) as request:
        return (
            'high'
            if request.get_value(pin) == Value.ACTIVE
            else 'low'
        )


if __name__ == '__main__':
    import argparse
    from time import sleep

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
            '--edge',
            dest='edge',
            action='store_true',
            help='Detect a rising edge'
        )
        parser.add_argument(
            '--pin_value',
            dest='pin_value',
            default=None,
            help='Set the pin value',
            type=str
        )

        return parser.parse_args()

    def edge_detected(event: gpiod.EdgeEvent) -> None:
        """Show the edge detection details."""
        print(
            f'Rising edge detected on {event.line_offset}'
            f' at {event.timestamp_ns} ns'
            f' and sequence {event.line_seqno}'
        )
        detector.done = True
        sys_exit(0)

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

        if parsed_args.edge:
            detector = GPIOEdgeDetector(
                pin,
                edge_detected,
                debounce_us=2000
            )
            detector.start()
            while not detector.done:
                sleep(1.0)
            detector.stop()

        if parsed_args.pin_value:
            print(
                f'Set {pin.value}: {set_pin(pin, parsed_args.pin_value)}'
            )
        else:
            print(
                f'Get {pin.value}: {get_pin(pin)}'
            )
