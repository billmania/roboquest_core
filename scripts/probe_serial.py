#!/usr/bin/env python3

from typing import Tuple
import serial
import RPi.GPIO as GPIO

READ_TIMEOUT_SEC = 0.2

COMMS_ENABLE = 22

PARITYS = ['N', 'E', 'O']
DATA_BITS = [8, 7]
DATA_RATES = [115200, 57600, 9600, 19200, 38400]
STOP_BITS = [1, 2]

TEST_SEQUENCE = bytearray('\r\n$$', 'ascii')
PROBE_SIZE = 100


def probe_serial(port_name: str = '/dev/ttyS0') -> Tuple[int, int, str, int]:
    """
    Probe the port_name serial port. If the protocol can be determined,
    return its data_rate, data_bits, parity, and stop bits.
    """

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(COMMS_ENABLE, GPIO.OUT)
    GPIO.output(COMMS_ENABLE, GPIO.HIGH)

    for parity in PARITYS:
        for data_bits in DATA_BITS:
            for data_rate in DATA_RATES:
                for stop_bits in STOP_BITS:
                    print(f"{data_rate} {data_bits} {parity} {stop_bits}")

                    port = serial.Serial(port=port_name,
                                         baudrate=data_rate,
                                         stopbits=stop_bits,
                                         bytesize=data_bits,
                                         parity=parity,
                                         timeout=1.0)
                    port.reset_input_buffer()

                    buffer = port.read(size=PROBE_SIZE)
                    try:
                        index = buffer.index(TEST_SEQUENCE)
                        print(f"  {TEST_SEQUENCE} at {index} in {buffer}")
                        port.close()
                        GPIO.cleanup()
                        return (data_rate, data_bits, parity, stop_bits)

                    except ValueError:
                        port.close()
                        continue

    GPIO.cleanup()
    raise Exception('Unable to determine protocol')


print(f"{probe_serial()}")
