"""
Utility function to find connected cameras.
"""

from pathlib import Path
from typing import List
from v4l2py.device import Device, Capability
have_csi = True
try:
    from picamera2 import Picamera2
except ModuleNotFoundError:
    have_csi = False

CSI_TYPE = 'ov5647'


def _list_video_devices() -> List[int]:
    """
    Return a list of integers corresponding to the /dev/video* device
    files.
    """

    video_indices = []
    dev_dir = Path('/dev')

    for video_file in dev_dir.glob('video*'):
        if video_file.is_char_device():
            video_indices.append(video_file.name.replace('video', ''))

    return video_indices


def _csi_device_found() -> bool:
    """
    Probe for a CSI camera. If one exists, return True,
    otherwise return False.

    https://www.tomshardware.com/how-to/raspberry-pi-camera-module-3-python-picamera-2
    """

    try:
        csi_camera = Picamera2()
        if csi_camera.camera_properties['Model'] == CSI_TYPE:
            return True

    except NameError as e:
        #
        # Likely on a host without the CSI, ie. not a Raspberry Pi.
        #
        if have_csi:
            raise e

    except Exception:
        pass

    return False


def find_cameras() -> List:
    """
    Use the list of integers from _list_video_devices() and attempt
    to open each of the corresponding /dev/video device files. If the
    device is a responsive character device and reports both Streaming
    and VideoCapture capability, add it to the list of likely usable
    cameras.
    """

    cameras_info = []

    for camera_id in _list_video_devices():
        try:
            camera_device = Device.from_id(camera_id)
            camera_device.open()

        except OSError:
            continue

        name = camera_device.info.card
        capabilities = camera_device.info.capabilities

        if (Capability.STREAMING in capabilities and
                Capability.VIDEO_CAPTURE in capabilities):
            cameras_info.append(f'{camera_id},{name}')

        camera_device.close()

    if _csi_device_found():
        cameras_info.append(f'CSI,{CSI_TYPE}')

    return cameras_info


def write_cameras_file(
        cameras_info: list,
        persist_dir='/opt/persist') -> None:
    """
    Write the cameras_info to a file, suitable for parsing by
    a script which creates ROS parameter files for ROS launch to use.
    """

    cameras_info_file = Path(persist_dir) / 'cameras_info'
    cameras_info_file.unlink(missing_ok=True)
    with open(cameras_info_file, 'w') as f:
        for camera in cameras_info:
            fields = camera.split(',')
            f.write(f'{fields[0]},{fields[1]}\n')

    return


if __name__ == '__main__':
    write_cameras_file(find_cameras())
