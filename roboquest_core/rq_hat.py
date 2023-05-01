from enum import Enum
from typing import Callable
import serial
import RPi.GPIO as GPIO

READ_EOL = b'\r\n'
WRITE_EOL = b'\r'
READ_TIMEOUT_SEC = 0.2


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

    UNKNOWN = '0'


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

    def __init__(self,
                 port: str,
                 data_rate: int,
                 data_bits: int,
                 parity: str,
                 stop_bits: int,
                 read_timeout_sec: float,
                 serial_errors_cb: Callable = None):
        self._serial_errors_cb = serial_errors_cb

        self._hat = None
        self._controls_state = {
            'charger': 'UNKNOWN',
            'fet_1': 'UNKNOWN',
            'fet_2': 'UNKNOWN',
            'comms': 'UNKNOWN'
        }
        self._setup_gpio()
        self._open_hat_serial(port,
                              data_rate,
                              data_bits,
                              parity,
                              stop_bits,
                              read_timeout_sec)
        self._screen_page = 0

    def _open_hat_serial(self, port: str,
                         data_rate: int,
                         data_bits: int,
                         parity: str,
                         stop_bits: int,
                         read_timeout_sec: float):
        if self._hat:
            try:
                self._hat.close()
            except serial.SerialException:
                pass

            self._hat = None

        try:
            self._hat = serial.Serial(port=port,
                                      baudrate=data_rate,
                                      bytesize=data_bits,
                                      parity=parity,
                                      stopbits=stop_bits,
                                      timeout=read_timeout_sec)
            self._hat.reset_input_buffer()
            self._hat.reset_output_buffer()

        except serial.SerialException:
            self._hat = None

    def _write_sentence(self, sentence: str) -> None:
        """
        Append the WRITE_EOL and then write the sentence.
        """

        if 0 < len(sentence):
            bytes_to_write = bytearray(sentence + WRITE_EOL, 'ascii')
            try:
                bytes_written = self._hat.write(bytes_to_write)
                if bytes_written != len(bytes_to_write):
                    self._open_hat_serial()

            except serial.SerialException:
                self._open_hat_serial()

        return None

    def _read_sentence(self) -> str:
        """
        Read one sentence from the serial port or timeout. Return
        the sentence after stripping the READ_EOL.

        One second's worth of data from the HAT looks like
        the following. Every line is terminated with \r\n:

$$TELEM 12.39 0.13 0.05 1.67 1.68 1.69 1.70 0.00 1
$$TELEM 12.37 0.12 0.09 1.67 1.68 1.69 1.70 0.00 1
$$TELEM 12.38 0.12 0.05 1.68 1.69 1.70 1.70 0.00 1
$$TELEM 12.39 0.12 0.05 1.67 1.69 1.70 1.70 0.00 1
$$TELEM 12.38 0.12 0.05 1.67 1.69 1.69 1.70 0.00 1
$$TELEM 12.37 0.23 0.01 1.68 1.69 1.70 1.71 0.00 1
$$TELEM 12.39 0.18 0.05 1.68 1.69 1.70 1.71 0.00 1
$$TELEM 12.37 0.16 -0.02 1.66 1.68 1.69 1.70 0.00 1
$$TELEM 12.37 0.17 -0.02 1.66 1.67 1.68 1.70 0.00 1
$$TELEM 12.38 0.17 0.05 1.67 1.69 1.69 1.69 0.00 1
$$TELEM 12.37 0.16 0.05 1.66 1.67 1.68 1.69 0.00 1
$$TELEM 12.38 0.17 0.01 1.65 1.66 1.68 1.69 0.00 1
$$TELEM 12.38 0.17 0.01 1.65 1.66 1.67 1.69 0.00 1
$$TELEM 12.39 0.20 -0.02 1.66 1.68 1.68 1.69 0.00 1
$$TELEM 12.38 0.21 -0.06 1.67 1.68 1.69 1.70 0.00 1
$$TELEM 12.37 0.21 0.01 1.66 1.68 1.68 1.70 0.00 1
$$TELEM 12.39 0.20 -0.02 1.65 1.67 1.68 1.69 0.00 1
$$TELEM 12.38 0.20 0.01 1.66 1.67 1.67 1.69 0.00 1
$$TELEM 12.38 0.21 -0.02 1.66 1.68 1.68 1.70 0.00 1
$$TELEM 12.37 0.20 -0.06 1.66 1.67 1.69 1.70 0.00 1
$$SCREEN 1 0
        """

        try:
            sentence = self._hat.read_until(
                expected=READ_EOL)

            if sentence:
                return sentence[:-len(READ_EOL)].decode('ascii')
            else:
                self._hat.reset_input_buffer()

        except serial.SerialException:
            self._open_hat_serial()

        return None

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

    def control_comms(self, enable: bool = False):
        """
        Used to enable and disable the flow of serial data from the HAT.
        Disabling the communications causes the input buffer to be reset.
        """

        new_state = 'ENABLED' if enable else 'DISABLED'
        if self._controls_state['comms'] != new_state:
            new_pin_output = GPIO.HIGH if enable else GPIO.LOW
            GPIO.output(HAT_GPIO_PIN.COMMS_ENABLE.value, new_pin_output)
            if new_state == 'DISABLED':
                self._hat.reset_input_buffer()

    def _setup_gpio(self):
        GPIO.setwarnings(False)
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
        GPIO.output(HAT_GPIO_PIN.COMMS_ENABLE.value, GPIO.LOW)
        self._controls_state['comms'] = 'DISABLED'

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
