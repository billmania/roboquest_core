#!/usr/bin/env python3

from typing import Tuple
import time
import serial
import RPi.GPIO as GPIO

READ_TIMEOUT_SEC = 0.2

COMMS_ENABLE = 22

PARITYS = ['N', 'E', 'O']
DATA_BITS = [7, 8]
DATA_RATES = [38400, 115200, 57600, 9600, 19200]
STOP_BITS = [1, 2]

TEST_SEQUENCE = bytearray('\r\n$$', 'ascii')
PROBE_SIZE = 100
TIMING_SENTENCES = 100
EOL = b'\r\n'
TELEM = b'$$T'
SCREEN = b'$$S'

def measure_serial(data_rate, data_bits, parity, stop_bits):
    countdown = TIMING_SENTENCES

    port = serial.Serial(port='/dev/ttyS0',
                         baudrate=data_rate,
                         stopbits=stop_bits,
                         bytesize=data_bits,
                         parity=parity,
                         timeout=1.0)
    port.reset_input_buffer()
    GPIO.output(COMMS_ENABLE, GPIO.HIGH)

    start = time.time()
    telem_count = 0
    screen_count = 0
    while countdown:
        countdown -= 1
        buffer = port.read_until(expected=EOL)
        read_time = time.time()

        sentence = buffer[:3]
        if sentence == TELEM:
            telem_count += 1
        elif sentence == SCREEN:
            screen_count += 1

    print(f"Tps: {telem_count / (read_time - start)}")
    print(f"Sps: {screen_count / (read_time - start)}")

    port.close()

def probe_serial(port_name: str = '/dev/ttyS0') -> Tuple[int, int, str, int]:
    """
    Probe the port_name serial port. If the protocol can be determined,
    return its data_rate, data_bits, parity, and stop bits.
    """

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
                        GPIO.output(COMMS_ENABLE, GPIO.LOW)
                        port.close()
                        return (data_rate, data_bits, parity, stop_bits)

                    except ValueError:
                        port.close()
                        continue

    GPIO.output(COMMS_ENABLE, GPIO.LOW)
    port.close()
    raise Exception('Unable to determine protocol')


GPIO.setmode(GPIO.BCM)
GPIO.setup(COMMS_ENABLE, GPIO.OUT)

data_rate, data_bits, parity, stop_bits = probe_serial()
print(f"{data_rate}, {data_bits}, {parity}, {stop_bits}")
measure_serial(data_rate, data_bits, parity, stop_bits)

GPIO.cleanup()
