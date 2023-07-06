from enum import Enum
from typing import Callable, Tuple
import serial
import RPi.GPIO as GPIO

READ_EOL = b'\r\n'
READ_TIMEOUT_SEC = 0.2

TELEM_HEADER = '$$TELEM'
SCREEN_HEADER = '$$SCREEN'

#
# Constants for managing the OLED display used as the
# RoboQuest HAT UI.
#
PAD_CHAR = ' '
LINE_LENGTH = 23
LINES = 6

EOL = '\n'
EOB = '\r'
DISPLAY_LENGTH = (LINE_LENGTH * LINES) - len(EOB)
COLUMNS = LINE_LENGTH - len(EOL)


class HAT_GPIO_PIN(Enum):
    """
    Pins for controlling features of the HAT.
    """

    COMMS_ENABLE = 22
    FET_1_ENABLE = 24
    FET_2_ENABLE = 25
    CHARGE_BATTERY = 21
    CHARGER_POWERED = 7


class HAT_SCREEN(Enum):
    """
    There are a total of five screens available from the HAT. Two of
    the screens can have multiple pages.
    The information on screens HOME and ABOUT comes directly from the HAT.
    The DEVICES AND CONNECTIONS screens are populated with information
    off the HAT. The STATUS screen shows status messages.

    SCREEN_HEADER messages are written by the HAT to the serial port on each
    of three conditions:
        1. an update cycle has arrived
        2. a different screen is now displayed
        3. the PREVIOUS_PAGE, NEXT_PAGE, or ENTER button was pressed

    The value of each member is left as the string representation of the
    number because the integer isn't needed and a cast can be avoided.
    """

    HOME = '1'
    DEVICES = '2'
    CONNECTIONS = '3'
    STATUS = '4'
    ABOUT = '5'

    UNKNOWN = '0'


class HAT_BUTTON(Enum):
    """
    The PREVIOUS_PAGE and NEXT_PAGE buttons cycle through the pages of
    a screen. The ENTER button is the command to select or enable the
    item currently displayed on the screen. For the update cycle SCREEN
    message, NO_BUTTON is sent.

    NEXT_PAGE and PREVIOUS_PAGE are used as integers.
    """

    NEXT_PAGE = -1
    PREVIOUS_PAGE = 1
    ENTER = 2
    NO_BUTTON = 0


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
        self._status_lines = list()

        self.status_msg('HAT setup')

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

    def write_sentence(self, sentence: str) -> None:
        """
        Encode from ASCII and then write the sentence.
        """

        if 0 < len(sentence):
            bytes_to_write = bytearray(sentence, 'ascii')
            try:
                bytes_written = self._hat.write(bytes_to_write)
                if bytes_written != len(bytes_to_write):
                    self._open_hat_serial()

            except serial.SerialException:
                self._open_hat_serial()

        return None

    def read_sentence(self) -> str:
        """
        Read one sentence from the serial port or timeout. Return
        the sentence after stripping the READ_EOL.

        One second's worth of data from the HAT looks like
        the following. Every line is terminated with \r\n:

$$TELEM 12.39 0.13 0.05 1.67 1.68 1.69 1.70 0.00 1
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
            GPIO.output(HAT_GPIO_PIN.CHARGE_BATTERY.value, GPIO.HIGH)
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

        #
        # Subsequent control of the battery charger should use
        # charger_control().
        #
        GPIO.setup(HAT_GPIO_PIN.CHARGE_BATTERY.value, GPIO.OUT)
        GPIO.output(HAT_GPIO_PIN.CHARGE_BATTERY.value, GPIO.LOW)
        self._controls_state['charger'] = 'ENABLED'

        GPIO.setup(HAT_GPIO_PIN.FET_1_ENABLE.value, GPIO.OUT)
        GPIO.output(HAT_GPIO_PIN.FET_1_ENABLE.value, GPIO.LOW)
        self._controls_state['fet_1'] = 'DISABLED'

        GPIO.setup(HAT_GPIO_PIN.FET_2_ENABLE.value, GPIO.OUT)
        GPIO.output(HAT_GPIO_PIN.FET_2_ENABLE.value, GPIO.LOW)
        self._controls_state['fet_2'] = 'DISABLED'

        #
        # Subsequent control of the communications channel should use
        # control_comms().
        #
        GPIO.setup(HAT_GPIO_PIN.COMMS_ENABLE.value, GPIO.OUT)
        GPIO.output(HAT_GPIO_PIN.COMMS_ENABLE.value, GPIO.LOW)
        self._controls_state['comms'] = 'DISABLED'

        GPIO.setup(HAT_GPIO_PIN.CHARGER_POWERED.value, GPIO.IN)

    def charger_state(self) -> Tuple[bool, bool]:
        """
        Return two booleans indicating if the battery is being charged and
        if the charger is powered.
        """

        power_pin = GPIO.input(HAT_GPIO_PIN.CHARGER_POWERED.value)
        charger_has_power = True if power_pin == GPIO.HIGH else False

        if not charger_has_power:
            #
            # If the charger doesn't have power, it can't charge the battery.
            #
            battery_charging = False
            self.charger_control(on=False)
        else:
            state_of_charger = self._controls_state['charger']
            battery_charging = True if state_of_charger == 'ENABLED' else False

        return battery_charging, charger_has_power

    def charger_control(self, on: bool = False) -> None:
        new_state = 'ENABLED' if on else 'DISABLED'
        if self._controls_state['charger'] != new_state:
            #
            # This pin is non-standard. Setting it LOW enables the charger.
            #
            new_pin_output = GPIO.LOW if on else GPIO.HIGH
            GPIO.output(HAT_GPIO_PIN.CHARGE_BATTERY.value, new_pin_output)
            self._controls_state['charger'] = new_state

    def fet1_control(self, on: bool = False) -> None:
        new_state = 'ENABLED' if on else 'DISABLED'
        if self._controls_state['fet_1'] != new_state:
            new_pin_output = GPIO.HIGH if on else GPIO.LOW
            GPIO.output(HAT_GPIO_PIN.FET_1_ENABLE.value, new_pin_output)
            self._controls_state['fet_1'] = new_state

    def fet2_control(self, on: bool = False) -> None:
        new_state = 'ENABLED' if on else 'DISABLED'
        if self._controls_state['fet_2'] != new_state:
            new_pin_output = GPIO.HIGH if on else GPIO.LOW
            GPIO.output(HAT_GPIO_PIN.FET_2_ENABLE.value, new_pin_output)
            self._controls_state['fet_2'] = new_state

    def status_msg(self, message: str) -> None:
        """
        Show a status message on the HAT OLED screen 4. message cannot contain
        EOL characters and cannot be longer than LINE_LENGTH characters. The
        most recent LINES messages will be shown on screen 4.
        """

        status = self.pad_line(message.replace(EOL, ' ')[:LINE_LENGTH])
        self._status_lines.append(status)
        if len(self._status_lines) > LINES:
            self._status_lines.pop(0)

        self.show_status_msgs()

    def show_status_msgs(self) -> None:
        """
        Display the status messages collected by status_msg().
        """

        status_text = ''
        for line in self._status_lines:
            status_text += line

        status_output = SCREEN_HEADER.lower()
        status_output += '='
        status_output += HAT_SCREEN.STATUS.value
        status_output += '='
        status_output += self.pad_text(status_text)
        self.write_sentence(status_output)

    def pad_line(self, text: str) -> str:
        """
        Return a string which contains text, left-justified, and
        then right-padded with enough PAD_CHARs and one trailing
        EOL to make the string length equal to COLUMNS.
        """

        return text.ljust(COLUMNS, PAD_CHAR) + EOL

    def pad_text(self, text) -> str:
        """
        Pad text to a total of LINES lines of text by adding
        four blank lines to the end of text and then truncating
        the result to DISPLAY_LENGTH characters.

        This is done primarily to ensure a shorter new text completely
        overwrites a longer, older text.
        """

        return (text + self.pad_line('') * LINES)[:DISPLAY_LENGTH] + EOB
