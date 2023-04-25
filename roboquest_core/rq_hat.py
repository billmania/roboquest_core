from enum import Enum
import serial
import RPi.GPIO as GPIO

ENABLE = GPIO.HIGH
DISABLE = GPIO.LOW


class HAT_GPIO_PIN(Enum):
    """
    Pins for controlling features of the HAT.
    """

    COMMS_ENABLE = 22
    FET_1_ENABLE = 24
    FET_2_ENABLE = 25
    CHARGER_DISABLE = 21
    CHARGER_DETECT = 7


class HAT_SCREEN(Enum):
    HOME = '1'
    DEVICES = '2'
    CONNECTIONS = '3'
    MISC = '4'
    ABOUT = '5'


class HAT_BUTTON(Enum):
    DOWN = '-1'
    UP = '1'
    ENTER = '2'
    NO_BUTTON = '0'


class RQHAT(object):
    """
    Manages the operation of the HAT, via a serial port. Even
    though motor and servo control is implemented on the HAT,
    they're managed by other classes.

    It manages:
    - GPIO
    - display screen
    - buttons
    - two-way serial port communication
    - battery charger
    - 2 current drivers
    - telemetry data
        - battery voltage
        - battery current
        - five ADC values
        - battery charger state
    - sleep mode

    - network configuration
        - devices
        - connections
    """

    def __init__(self, parameters: dict):
        self.hat = serial.Serial
        self._screen_page = 0
        self._controls_state = {
            'charger': 'UNKNOWN',
            'fet_1': 'UNKNOWN',
            'fet_2': 'UNKNOWN',
            'comms': 'UNKNOWN'
        }

        self._setup_gpio()

    def cleanup_gpio(self):
        if self._controls_state['charger'] == 'ENABLED':
            GPIO.output(HAT_GPIO_PIN.CHARGER_DISABLE.value, GPIO.HIGH)
            self._controls_state['charger'] = 'DISABLED'

        if self._controls_state['fet_1'] == 'ENABLED':
            GPIO.output(HAT_GPIO_PIN.FET_1_ENABLE.value, GPIO.LOW)

        if self._controls_state['fet_2'] == 'ENABLED':
            GPIO.output(HAT_GPIO_PIN.FET_2_ENABLE.value, GPIO.LOW)

        if self._controls_state['comms'] == 'ENABLED':
            GPIO.output(HAT_GPIO_PIN.COMMS_ENABLE.value, GPIO.LOW)

        GPIO.cleanup()

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(HAT_GPIO_PIN.CHARGER_DISABLE.value, GPIO.OUT)
        GPIO.output(HAT_GPIO_PIN.CHARGER_DISABLE.value, GPIO.LOW)
        self._controls_state['charger'] = 'ENABLED'

        GPIO.setup(HAT_GPIO_PIN.FET_1_ENABLE.value, GPIO.OUT)
        GPIO.output(HAT_GPIO_PIN.FET_1_ENABLE.value, GPIO.LOW)
        self._controls_state['fet_1'] = 'DISABLED'

        GPIO.setup(HAT_GPIO_PIN.FET_2_ENABLE.value, GPIO.OUT)
        GPIO.output(HAT_GPIO_PIN.FET_2_ENABLE.value, GPIO.LOW)
        self._controls_state['fet_2'] = 'DISABLED'

        GPIO.setup(HAT_GPIO_PIN.COMMS_ENABLE.value, GPIO.OUT)
        GPIO.output(HAT_GPIO_PIN.COMMS_ENABLE.value, GPIO.HIGH)
        self._controls_state['comms'] = 'ENABLED'

        GPIO.setup(HAT_GPIO_PIN.CHARGER_DETECT.value, GPIO.IN)

    def _charger_control(self, enable: bool = False) -> None:
        # TODO: implement
        pass

    def _current_control(self,
                         enable_driver1: bool = False,
                         enable_driver2: bool = False) -> None:
        # TODO: Implement
        pass

    def _set_sleep_duration(self, sleep_secs: int = 0) -> None:
        # TODO: Implement
        pass

    def read_next_line(self) -> str:
        """
        Read the next line of output from the HAT, which will be either
        SCREEN or TELEM.
        """

        # TODO: Implement
        pass

    def _get_network_device(self):
        """
        Populates the Devices screen of the UI.
        Get the NM Connections which are currently active, ie. applied to a
        NM Device (interface). Collect the details about each NM Device and
        return it to the display, so it can be used to connect with a specific
        network interface.
        """

        # TODO: Implement
        pass

    def _get_network_connections(self):
        """
        Populates the Connections screen of the UI.

        Parse the list of all defined NM Connections.
        """

        # TODO: Implement
        pass
