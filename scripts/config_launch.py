#!/usr/bin/env python3

from pathlib import Path
from find_cameras import write_cameras_file, find_cameras

MAX_CSI = 1
MAX_USB = 3
PERSIST_DIR = '/tmp'
# INSTALL_DIR = '/usr/src/ros2ws/install/roboquest_core/share/roboquest_core'
INSTALL_DIR = '/home/bill/projects/roboquest/ros2ws/src/roboquest_core'
CONFIG_DIR = INSTALL_DIR + '/config'
CSI_TEMPLATE = 'csi_camera_params.template'
CSI_YAML = 'csi_camera.yaml'
USB_TEMPLATE = 'usb_camera_params.template'
LAUNCH_DIR = INSTALL_DIR + '/launch'
LAUNCH_TEMPLATE = 'roboquest_core_launch.template'
LAUNCH_FILE = 'roboquest_core.launch.py'


class CameraConfig(object):
    """
    Probe the hardware to find the currently connected CSI and USB cameras.
    Use the config template files to create a ROS parameter file for each
    camera. Use the launch template file to adjust the launch file.

    This script is to be used BEFORE calling ros2 launch.
    """

    def __init__(self):
        self._cameras_list = None
        self._ids_to_launch = []

        self._create_cameras_file()

    def _create_cameras_file(self) -> None:
        """
        Use find_cameras to get the list of cameras. Use write_cameras_file
        to write that list to a persisent file, so the browser UI will have
        access to it.
        """

        self._cameras_list = find_cameras()
        write_cameras_file(self._cameras_list, PERSIST_DIR)

    def _create_csi_params(self) -> None:
        """
        Create an updated CSI parameters file and put it in place.
        At present, the CSI parameter file is static.
        """

        template = Path(CONFIG_DIR) / CSI_TEMPLATE
        with open(template, 'r') as f:
            self._csi_template = f.read()
        param_file = Path(CONFIG_DIR) / CSI_YAML
        param_file.unlink(missing_ok=True)
        with open(param_file, 'w') as f:
            f.write(self._csi_template)

        self._ids_to_launch.append('CSI')

        return

    def _create_usb_params(self, id: str):
        """
        Create updated USB parameter file for id and put it in
        place.
        The ID numbers for the /dev/video files are not expected to
        be contiguous. However, the usb camera parameter files
        included in the launch file are contiguously named.
        """

        param_files = []
        param_files.append('usb_camera_A.yaml')
        param_files.append('usb_camera_B.yaml')
        param_files.append('usb_camera_C.yaml')

        template = Path(CONFIG_DIR) / USB_TEMPLATE
        with open(template, 'r') as f:
            self._usb_template = f.read()
        usb_yaml = param_files.pop(0)
        param_file = Path(CONFIG_DIR) / usb_yaml
        self._usb_template = self._usb_template.replace('%VIDEO_ID%', id)
        param_file.unlink(missing_ok=True)
        with open(param_file, 'w') as f:
            f.write(self._usb_template)

        self._ids_to_launch.append(id)

        return

    def _create_launch_file(self):
        """
        Create the launch file to start the appropriate collection of
        camera nodes.
        """

        usb_node_names = []
        usb_node_names.append('usb_camera_node_A')
        usb_node_names.append('usb_camera_node_B')
        usb_node_names.append('usb_camera_node_C')
        nodes = ''

        csi_nodes = 0
        usb_nodes = 0
        for node in self._ids_to_launch:
            if node == 'CSI' and csi_nodes < MAX_CSI:
                nodes += ', csi_camera_node'
                csi_nodes += 1
            elif usb_nodes < MAX_USB:
                nodes += ', ' + usb_node_names.pop(0)
                usb_nodes += 1

        template = Path(LAUNCH_DIR) / LAUNCH_TEMPLATE
        with open(template, 'r') as f:
            self._launch_template = f.read()
        launch_py = self._launch_template.replace('%CAMERA_NODES%', nodes)
        launch_file = Path(LAUNCH_DIR) / LAUNCH_FILE
        launch_file.unlink(missing_ok=True)
        with open(launch_file, 'w') as f:
            f.write(launch_py)

    def _process_camera_list(self) -> None:
        """
        Read the contents of the camera list, one entry at a time.
        Call the CSI or the USB setup as appropriate, keeping track
        of which nodes must be started in the launch file.
        Finally, create an appropriate launch file.
        """

        for camera in self._cameras_list:
            details = camera.split(',')
            id = details[0]
            name = details[1]
            print(f"camera id {id}, name {name}")

            if id == 'CSI':
                self._create_csi_params()
            else:
                self._create_usb_params(id)

        self._create_launch_file()

    def run(self):
        """
        Create the parameter files and then the launch file.
        """

        self._process_camera_list()


if __name__ == '__main__':
    camera_config = CameraConfig()

    camera_config.run()
