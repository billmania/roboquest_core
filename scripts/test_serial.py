#!/usr/bin/env python3

import serial
import RPi.GPIO as GPIO

READ_EOL = '\n'
WRITE_EOL = '\r'
MAX_SENTENCE_LENGTH = 80
READ_TIMEOUT_SEC = 0.2

COMMS_ENABLE = 22

hat = serial.Serial(port='/dev/ttyS0',
                    baudrate=57600,
                    timeout=0.2)
hat.reset_input_buffer()
hat.reset_output_buffer()

GPIO.setmode(GPIO.BCM)
GPIO.setup(COMMS_ENABLE, GPIO.OUT)
GPIO.output(COMMS_ENABLE, GPIO.HIGH)

for i in range(10):
    print(f"{hat.read_until(READ_EOL)}")

GPIO.cleanup()
