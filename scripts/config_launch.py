#!/usr/bin/env python3

from pathlib import Path

MAX_CSI = 1
MAX_USB = 3
PERSIST_DIR = '/tmp'
INSTALL_DIR = '/usr/src/ros2ws/install/roboquest_core/share/roboquest_core'
# INSTALL_DIR = '/home/bill/projects/roboquest/ros2ws/src/roboquest_core'
PERSIST_DIR = '/usr/src/ros2ws/install/roboquest_core/share/roboquest_core/persist'
CAMERAS_FILE = 'cameras_info'
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

        #
        # These two lists must have a length <= MAX_USB.
        #
        self._param_files = []
        self._param_files.append('usb_camera_A.yaml')
        self._param_files.append('usb_camera_B.yaml')
        self._param_files.append('usb_camera_C.yaml')
        self._usb_node_names = []
        self._usb_node_names.append('usb_camera_node_A')
        self._usb_node_names.append('usb_camera_node_B')
        self._usb_node_names.append('usb_camera_node_C')

        self._get_cameras_file()

    def _get_cameras_file(self) -> None:
        """
        Get the list of cameras from the file.
        """

        cameras_file = Path(PERSIST_DIR) / CAMERAS_FILE
        with open(cameras_file, 'r') as f:
            self._cameras_list = f.read()

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

        if len(self._param_files) == 0:
            return

        template = Path(CONFIG_DIR) / USB_TEMPLATE
        with open(template, 'r') as f:
            self._usb_template = f.read()
        usb_yaml = self._param_files.pop(0)
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

        nodes = ''

        csi_nodes = 0
        usb_nodes = 0
        for node in self._ids_to_launch:
            if node == 'CSI' and csi_nodes < MAX_CSI:
                nodes += ', csi_camera_node'
                csi_nodes += 1
            elif usb_nodes < MAX_USB:
                nodes += ', ' + self._usb_node_names.pop(0)
                usb_nodes += 1

        template = Path(LAUNCH_DIR) / LAUNCH_TEMPLATE
        with open(template, 'r') as f:
            self._launch_template = f.read()
        launch_py = self._launch_template.replace('%CAMERA_NODES%', nodes)
        launch_file = Path(LAUNCH_DIR) / LAUNCH_FILE
        launch_file.unlink(missing_ok=True)
        with open(launch_file, 'w') as f:
            f.write(launch_py)

    def run(self):
        """
        Read the contents of the camera list, one entry at a time.
        Call the CSI or the USB setup as appropriate, keeping track
        of which nodes must be started in the launch file.
        Finally, create an appropriate launch file.
        """


        for camera in self._cameras_list.split('\n'):
            if camera == '':
                break

            details = camera.split(',')
            id = details[0]
            name = details[1]
            print(f"camera id {id}, name {name}")

            if id == 'CSI':
                self._create_csi_params()
            else:
                self._create_usb_params(id)

        self._create_launch_file()


if __name__ == '__main__':
    camera_config = CameraConfig()

    camera_config.run()
