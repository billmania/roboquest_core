#!/usr/bin/env python3

FRAMES = 100
FORMAT = 'jpeg'
#WIDTH = 3280
#HEIGHT = 2464
WIDTH = 640
HEIGHT = 480

import io
import time
from picamera2 import Picamera2

picam2 = Picamera2()

"""
{'use_case': 'still', 'transform': <libcamera.Transform 'identity'>, 'colour_space':
<libcamera.Colo
rSpace 'sYCC'>, 'buffer_count': 1, 'queue': True, 'main': {'format': 'BGR888',
'size': (3280, 2464)}
, 'lores': None, 'raw': {'format': 'SBGGR10_CSI2P', 'size': (3280, 2464)},
'controls': {'NoiseReduct
ionMode': <NoiseReductionModeEnum.HighQuality: 2>, 'FrameDurationLimits': (100,
1000000000)}, 'displ
ay': None, 'encode': None}
"""

capture_config = picam2.create_still_configuration()
capture_config['main']['size'] = (WIDTH, HEIGHT)
capture_config['raw']['size'] = (WIDTH, HEIGHT)
picam2.configure(capture_config)
picam2.start()

start = time.time()
print(f"Start time {start}")

data = io.BytesIO()
bytes_collected = 0
for frame in range(FRAMES):
    picam2.capture_file(data, format=FORMAT)
    bytes_collected += data.getbuffer().nbytes

end = time.time()

print(f"{FORMAT} Frames per second = {FRAMES / (end - start)}")
print(f"Frame average size {bytes_collected / FRAMES}")

metadata_jpg = picam2.capture_file(f"{WIDTH}x{HEIGHT}.jpg")
metadata_png = picam2.capture_file(f"{WIDTH}x{HEIGHT}.png")
