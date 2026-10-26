#!/usr/bin/env python3
"""Show an example Roboquest user script."""

import logging
import os
import sys

logging.basicConfig(
    filename='sample_user_script.log',
    format='%(asctime)s %(levelname)s %(message)s',
    level=logging.INFO
)
logging.info('sample_user_script started')

logging.info(
    f"_ = {os.environ['_']}"
)
logging.info(
    f"PWD = {os.environ['PWD']}"
)

sys.exit(0)
