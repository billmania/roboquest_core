#!/usr/bin/env python3

"""
Adjust the camera config.

Executed via an ssh terminal session as root on the robot.
"""

import argparse
import logging
from fcntl import ioctl
from glob import glob
from re import sub
from socket import AF_INET, SOCK_DGRAM, socket
from struct import pack
from subprocess import run
from urllib.request import urlopen

from yaml import dump, safe_load


VERSION = '2'
RESOLUTIONS = [
    '640x480',
    '1296x972',
    '1920x1080',
    '2592x1944'
]
MIN_FRAME_RATE = 1
MAX_FRAME_RATE = 25
MICROSECS_PER_SEC = 1000000
CAM_PARAMS_FILE = '/opt/persist/rq_camera0.yaml'
CALIBRATION_DIR = '/opt/persist/calibration'
INTERFACE = 'eth0'
CALIBRATION_URL_BASE = 'http://registry.q4excellence.com:8079/calibration/'
DEFAULT_CALIBRATION_ROBOT = 'rq-23eb'


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
            default=None,
            help=resolution_help,
            type=int
        )
        parser.add_argument(
            '--frame_rate',
            dest='frame_rate',
            default=None,
            help=f'[{MIN_FRAME_RATE}, {MAX_FRAME_RATE}]',
            type=int
        )
        parser.add_argument(
            '--robot_name',
            dest='robot_name',
            default=None,
            help='Calibration files for which robot hostname',
            type=str
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

    def _get_camera_name(self) -> str:
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
            '_get_camera_name:'
            f' {device_name}'
        )
        return device_name

    def _form_camera_name(self) -> str:
        """Make a camera name string.

        It includes only the module and the device path. The
        resolution is not included.
        """
        device_name = self._get_device_name()

        camera_name = device_name
        logging.debug(
            '_form_camera_name:'
            f' {camera_name}'
        )
        return camera_name

    def _get_camera_parameters(self):
        """Get the old camera parameters."""
        with open(CAM_PARAMS_FILE, 'r') as f:
            self._old_camera_parameters = safe_load(f)

        self._old_resolution = (
            f"{(
                self._old_camera_parameters
                ['/**']
                ['ros__parameters']
                ['width']
               )}"
            'x'
            f"{(
                self._old_camera_parameters
                ['/**']
                ['ros__parameters']
                ['height']
               )}"
        )

    def _adjust_params(self):
        """Adjust the params file.

        Adjust the height, width, and FrameDurationLimits.
        """
        if (
             not self._parsed_args.frame_rate
             and not self._parsed_args.resolution):
            logging.info(
                'Parameters not changed.'
            )
            return

        if self._parsed_args.resolution is not None:
            logging.debug(
                '_adjust_params:'
                f' Old resolution: {self._old_resolution}'
            )
            self._old_camera_parameters['/**']['ros__parameters']['height'] = (
                int(RESOLUTIONS[self._parsed_args.resolution].split('x')[1])
            )
            self._old_camera_parameters['/**']['ros__parameters']['width'] = (
                int(RESOLUTIONS[self._parsed_args.resolution].split('x')[0])
            )

        if self._parsed_args.frame_rate is not None:
            old_frame_rate = (
                MICROSECS_PER_SEC
                / int(
                    self._old_camera_parameters
                    ['/**']
                    ['ros__parameters']
                    ['FrameDurationLimits']
                    [0]
                  )
            )
            logging.debug(
                '_adjust_params:'
                f' Old frame rate: {int(old_frame_rate)}'
            )
            frame_limit = self._calculate_frame_rate(
                self._parsed_args.frame_rate
            )
            (self._old_camera_parameters
             ['/**']
             ['ros__parameters']
             ['FrameDurationLimits']) = [frame_limit, frame_limit]

        with open(CAM_PARAMS_FILE, 'w') as f:
            dump(self._old_camera_parameters, f)

    def _get_robot_name(self, interface: str) -> str:
        """Get the robot's hostname."""
        if self._parsed_args.robot_name:
            return self._parsed_args.robot_name

        s = socket(AF_INET, SOCK_DGRAM)
        info = ioctl(
            s.fileno(),
            0x8927,
            pack('256s', bytes(interface, 'utf-8')[:15])
        )

        mac_address = ''
        try:
            for octet in info[18:24]:
                mac_address += f'{octet:02x}'
            robot_name = 'rq-' + mac_address[-4:]

        except TypeError as e:
            logging.warning(
                '_get_robot_name:'
                f' Excepted {e}'
                f', info=<{info[18:24]}>'
                f', octet=<{octet}>'
                f', robot_name=<{robot_name}>'
            )

        return robot_name

    def _retrieve_calibration_file(
         self,
         calibration_file_path: str,
         camera_name: str,
         resolution: str,
         robot_name: str):
        """Retrieve and install a calibration file."""
        calibration_file_url = (
            CALIBRATION_URL_BASE
            + robot_name
            + '_ov5647_'
            + resolution
            + '.yaml'
        )
        logging.debug(
            '_retrieve_calibration_file:'
            f' Retrieving {calibration_file_url}'
        )
        try:
            with urlopen(calibration_file_url) as f:
                calibration_file_content = f.read().decode('utf-8')

        except Exception:
            logging.warning(
                '_retrieve_calibration_file:'
                f' {calibration_file_url} not found'
            )

            logging.warning(
                '_retrieve_calibration_file:'
                ' Retrieving default calibration file.'
            )
            calibration_file_url = (
                CALIBRATION_URL_BASE
                + DEFAULT_CALIBRATION_ROBOT
                + '_ov5647_'
                + resolution
                + '.yaml'
            )
            with urlopen(calibration_file_url) as f:
                calibration_file_content = f.read().decode('ascii')

        finally:
            with open(calibration_file_path, 'w', encoding='ascii') as f:
                f.write(calibration_file_content)

    def _check_calibration_files(
         self,
         camera_name: str,
         robot_name: str):
        """Ensure calibration file exists.

        Ensure a calibration file, for the robot's camera and the selected
        resolution exists in the filesystem. If the required isn't present,
        attempt to retrieve it.
        """
        resolution = (
            self._old_resolution
            if self._parsed_args.resolution is None
            else RESOLUTIONS[self._parsed_args.resolution]
        )
        calibration_file_glob = (
            CALIBRATION_DIR +
            '/' +
            camera_name +
            resolution +
            '.yaml'
        )
        logging.debug(
            '_check_calibration_files:'
            f' Checking for {calibration_file_glob}'
        )
        calibration_file = glob(calibration_file_glob)
        if calibration_file:
            logging.info(
                '_check_calibration_files:'
                ' Calibration file already exists.'
            )
            return

        self._retrieve_calibration_file(
            calibration_file_glob,
            camera_name,
            resolution,
            robot_name
        )

    def main(self):
        """Execute the steps in the tool."""
        logging.debug(
            'main called'
        )

        self._parsed_args = self._parse_args()

        robot_name = self._get_robot_name(INTERFACE)
        logging.debug(
            'main:'
            f' robot_name: {robot_name}'
        )

        self._get_camera_parameters()
        camera_name = self._get_camera_name()

        if (
             self._parsed_args.resolution is not None
             or self._parsed_args.frame_rate is not None):
            self._adjust_params()

        self._check_calibration_files(
            camera_name,
            robot_name
        )

        logging.debug(
            'main completed'
        )


if __name__ == '__main__':
    CamTool().main()
