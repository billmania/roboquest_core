#!/usr/bin/env python3

"""
Adjust the camera config.

Executed via an ssh terminal session as root on the robot.
"""

import argparse
import logging
from glob import glob
from pathlib import Path
from re import sub
from subprocess import run
from sys import exit as sys_exit

from yaml import dump, safe_load


VERSION = '1'
RESOLUTIONS = [
    '640x480',
    '1296x972',
    '1920x1080',
    '2592x1944'
]
MIN_FRAME_RATE = 1
DEFAULT_FRAME_RATE = 10
MAX_FRAME_RATE = 25
MICROSECS_PER_SEC = 1000000
CAM_PARAMS_FILE = '/opt/persist/rq_camera0.yaml'
CALIBRATION_DIR = '/opt/persist/calibration'


class CamTool(object):
    """Demonstrate APRIL tags and LiDAR."""

    def __init__(self):
        """Prepare the tool."""
        logging.basicConfig(
            format='%(asctime)s %(levelname)s %(message)s',
            level=logging.DEBUG
        )
        logging.info(f'cam_tool.py version {VERSION} started')

        self._parsed_args = self._parse_args()

    def _parse_args(self) -> argparse.Namespace:
        """Get the arguments from the command line."""
        logging.debug(
            '_parse_args called'
        )
        parser = argparse.ArgumentParser(
            prog='CamTool',
            description='Adjust the resolution and frame rate',
            epilog=''
        )

        resolution_help = ''
        for i, resolution in enumerate(RESOLUTIONS):
            resolution_help += f'{i}. {resolution}, '
        resolution_help = resolution_help[:-2]
        parser.add_argument(
            '--resolution',
            dest='resolution',
            default=0,
            help=resolution_help,
            type=int
        )
        parser.add_argument(
            '--frame_rate',
            dest='frame_rate',
            default=DEFAULT_FRAME_RATE,
            help=f'[{MIN_FRAME_RATE}, {MAX_FRAME_RATE}]',
            type=int
        )

        return parser.parse_args()

    def _calculate_frame_rate(self, frame_rate: int) -> int:
        """Calculate the min and max frame duration limits."""
        frame_rate = max(frame_rate, MIN_FRAME_RATE)
        frame_rate = min(frame_rate, MAX_FRAME_RATE)

        frame_limit = int(MICROSECS_PER_SEC / frame_rate)
        logging.debug(
            '_calculate_frame_rate:'
            f' {frame_limit}'
        )
        return frame_limit

    def _get_device_name(self, resolution: int) -> str:
        """Get the device name."""
        output = run(
            [
                '/usr/bin/rpicam-hello',
                '--list-cameras'
            ],
            capture_output=True
        )
        for line in output.stdout.decode().split('\n'):
            if line.find(': ov5647 ') != -1:
                break

        device_path = sub(r'\)$', '', sub(r'^.* \(', '', line))
        device_name = (
            'ov5647_' +
            device_path.replace('/', '_').replace('@', '_') +
            '_'
        )
        logging.debug(
            '_get_device_name:'
            f' {device_name}'
        )
        return device_name

    def _form_camera_name(
         self,
         resolution: str) -> str:
        """Make a camera name string."""
        device_name = self._get_device_name(resolution)

        camera_name = device_name + RESOLUTIONS[resolution]
        logging.debug(
            '_form_camera_name:'
            f' {camera_name}'
        )
        return camera_name

    def _adjust_params(self, resolution: int):
        """Adjust the params file.

        Adjust the height, width, and FrameDurationLimits.
        """
        frame_rate = self._calculate_frame_rate(self._parsed_args.frame_rate)

        with open(CAM_PARAMS_FILE, 'r') as f:
            params = safe_load(f)
#         logging.debug(
#             '_adjust_params:'
#             f' Old parameters: {dump(params)}'
#         )

        self._old_resolution = (
            f"{params['/**']['ros__parameters']['width']}"
            'x'
            f"{params['/**']['ros__parameters']['height']}"
        )
        logging.debug(
            '_adjust_params:'
            f' Old resolution: {self._old_resolution}'
        )

        params['/**']['ros__parameters']['height'] = (
            int(RESOLUTIONS[resolution].split('x')[1])
        )
        params['/**']['ros__parameters']['width'] = (
            int(RESOLUTIONS[resolution].split('x')[0])
        )
        params['/**']['ros__parameters']['FrameDurationLimits'] = [
            frame_rate,
            frame_rate
        ]

        with open(CAM_PARAMS_FILE, 'w') as f:
            dump(params, f)

#         logging.debug(
#             '_adjust_params'
#             f' New parameters: {dump(params)}'
#         )

    def _adjust_calibration(
         self,
         resolution: str,
         camera_name: str):
        """Adjust the calibration file.

        The calibration yaml file is NOT expected to be a complete
        ROS parameter file. It's missing the /** and ros__parameters
        keys.

        Find the ov5647 calibration file in CALIBRATION_DIR.
        Parse its YAML contents, update them, write them back
        to the file. Then rename the file with the new resolution.
        """
        calibration_glob = (
            CALIBRATION_DIR +
            '/ov5647__*' +
            self._old_resolution +
            '.yaml'
        )
        calibration_file = glob(calibration_glob)
        if not calibration_file:
            logging.error(
                '_adjust_calibration:'
                ' No calibration file found with'
                f' {calibration_glob}'
            )
            logging.error(
                'See'
                ' https://github.com/billmania/roboquest_addons/wiki'
                '/Calibrate-CSI-camera-for-APRIL-tags'
            )
            sys_exit(1)

        calibration_file = calibration_file[0]
        logging.debug(
            '_adjust_calibration:'
            f' Calibration file: {calibration_file}'
        )

        with open(calibration_file, 'r') as f:
            params = safe_load(f)

        params['image_height'] = (
            int(RESOLUTIONS[resolution].split('x')[1])
        )
        params['image_width'] = (
            int(RESOLUTIONS[resolution].split('x')[0])
        )
        params['camera_name'] = camera_name

        with open(calibration_file, 'w') as f:
            dump(params, f)

#         logging.debug(
#             '_adjust_calibration'
#             f' {dump(params)}'
#         )

        new_calibration_file = (
            CALIBRATION_DIR +
            '/' +
            camera_name +
            '.yaml'
        )
        Path(calibration_file).rename(new_calibration_file)

        logging.debug(
            '_adjust_calibration:'
            f' Renamed {calibration_file}'
            f' to {new_calibration_file}'
        )

    def main(self):
        """Execute the steps in the tool."""
        logging.debug(
            'main called'
        )

        self._parsed_args = self._parse_args()

        camera_name = self._form_camera_name(
            self._parsed_args.resolution
        )

        self._adjust_params(self._parsed_args.resolution)

        self._adjust_calibration(
            self._parsed_args.resolution,
            camera_name
        )

        logging.debug(
            'main completed'
        )


if __name__ == '__main__':
    CamTool().main()
