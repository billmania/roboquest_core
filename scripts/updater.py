#!/usr/bin/env python3

import os
from sys import exit
from pathlib import Path
from requests import get
import logging
import json
import docker
from time import sleep
import RPi.GPIO as GPIO

from rq_hat import RQHAT

"""
A utility script to manage the updating of RoboQuest docker images.
It must be executing on the same host as the docker containers and
must execute as the superuser. Its curent working directory must be
the directory where it is installed and both the directory and script
must be writable and executable.

Required modules (on the host OS) are:
    - docker
    - requests

Files and directories:
    - UPDATER_DIR/updater.log
    - systemd service with auto restart
"""

VERSION = 14
HAT_SERIAL = '/dev/ttyAMA1'
SHUTDOWN_PIN = 27
SERIAL_NUMBER_FILE = '/sys/firmware/devicetree/base/serial-number'
RQ_CORE_PERSIST = (
    '/usr/src/ros2ws/install/roboquest_core/share/roboquest_core/persist'
)
RQ_UI_PERSIST = (
    '/usr/src/ros2ws/install/roboquest_ui/share/roboquest_ui/public/persist'
)
OS_PERSIST_DIR = '/opt/persist'
UPDATER_DIR = '/opt/updater'
DIRECTORIES = [OS_PERSIST_DIR, UPDATER_DIR]
CONFIG_FILES = ['configuration.json']
UPDATE_LOG = UPDATER_DIR + '/updater.log'
UPDATE_IN_PROGRESS = UPDATER_DIR + '/update_in_progress'
UPDATE_FIFO = '/tmp/update_fifo'
UPDATE_VERSION = 'http://registry.q4excellence.com:8079/updater_version.txt'
FIRMWARE_VERSION = 'http://registry.q4excellence.com:8079/firmware_version.txt'
IMAGE_VERSIONS = 'http://registry.q4excellence.com:8079/image_versions.json'
ALL_VERSIONS = OS_PERSIST_DIR + '/versions.json'
UPDATE_SCRIPT = 'updater.py'
UPDATE_URL = 'http://registry.q4excellence.com:8079/' + UPDATE_SCRIPT
LOOP_PERIOD_S = 10.0
LONG_TIME = 60
EOL = '\n'

#
# When this dictionary is modified, remember to update both dstart.sh
# scripts.
# Because the rq_core container is run in privileged mode, the inclusion
# of the devices list is superfluous.
#
CONTAINERS = {
    'rq_core': {
        'image_name': 'registry.q4excellence.com:5678/rq_core',
        'privileged': True,
        'devices': ['/dev/gpiomem:/dev/gpiomem:rwm',
                    '/dev/i2c-1:/dev/i2c-1:rwm',
                    '/dev/i2c-6:/dev/i2c-6:rwm',
                    HAT_SERIAL+':'+HAT_SERIAL+':rwm'],
        'volumes': ['/dev/shm:/dev/shm',
                    '/var/run/dbus:/var/run/dbus',
                    '/run/udev:/run/udev:ro',
                    OS_PERSIST_DIR+':'+RQ_CORE_PERSIST,
                    'ros_logs:/root/.ros/log']},
    'rq_ui': {
        'image_name': 'registry.q4excellence.com:5678/rq_ui',
        'privileged': False,
        'devices': [],
        'volumes': ['/dev/shm:/dev/shm',
                    UPDATE_FIFO+':'+UPDATE_FIFO,
                    OS_PERSIST_DIR+':'+RQ_UI_PERSIST,
                    'ros_logs:/root/.ros/log']}
}


class RQUpdate(object):
    """
    All of the methods to manage the update process.
    """

    def __init__(self, fifo_path):
        """
        Setup all of the stuff.
        """

        self._make_dirs(DIRECTORIES)
        logging.basicConfig(
            filename=UPDATE_LOG,
            format='%(asctime)s %(levelname)s %(message)s',
            level=logging.INFO)
        logging.info(f"updater.py version {VERSION} started")

        self._ros_domain_id = self._get_ros_domain_id()
        logging.info(f'Set ROS domain: {self._ros_domain_id}')

        self._messages = list()
        self._client = None
        self._fifo = None
        self._latest_updater_version = None
        self._latest_image_versions = None
        self._status_messages = list()

        self._fifo_path = fifo_path

        self._setup_shutdown()
        self._setup_docker()
        self._remove_old_images()
        self._setup_fifo()

    def _update_in_progress(self) -> bool:
        """
        Test whether an update was in progress before this script restarted.
        """

        if Path(UPDATE_IN_PROGRESS).exists():
            return True

        return False

    def _reset_update_in_progress(self) -> None:
        """
        Indicate there isn't an update in progress.
        """

        Path(UPDATE_IN_PROGRESS).unlink(missing_ok=True)

        return

    def _set_update_in_progress(self) -> None:
        """
        Indicate there is an update in progress.
        """

        Path(UPDATE_IN_PROGRESS).touch(mode=0o444)

        return

    def _setup_shutdown(self):
        """
        Setup the callback to call when the shutdown hardware
        signal is detected.
        """

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SHUTDOWN_PIN, GPIO.IN)
        GPIO.add_event_detect(
            SHUTDOWN_PIN,
            GPIO.BOTH,
            callback=self._shutdown_cb
        )

    def _setup_docker(self):
        """
        Setup the docker client.
        """

        self._client = docker.from_env()

    def _setup_fifo(self):
        """
        Ensure the FIFO rendezvous point exists and configure it for
        non-blocking reads. The non-existence of the FIFO can be interpreted
        by the UI as an update in progress.
        """

        try:
            os.mkfifo(self._fifo_path)

        except FileExistsError:
            pass

        fifo_fd = os.open(self._fifo_path, os.O_RDONLY | os.O_NONBLOCK)

        self._fifo = os.fdopen(fifo_fd)

    def close_fifo(self) -> None:
        """
        Close and remove the FIFO.
        """

        if self._fifo:
            self._fifo.close()
        Path(self._fifo_path).unlink(missing_ok=True)

        self._fifo = None

    def read_message(self) -> str:
        """
        If a message is available to read from the FIFO, read and return it.
        """

        if not self._messages:
            for message in self._fifo.readline().split(EOL):
                if message:
                    self._messages.append(message)

        if self._messages:
            message = self._messages.pop(0)
            return message

        return None

    def _requirements_met(self) -> bool:
        """
        Confirm network connectivity is available and appears stable.
        """
        result = True

        response = get(UPDATE_VERSION, timeout=10.0)
        if response.status_code == 200:
            self._latest_updater_version = int(response.text)
        else:
            logging.warning("Network connectivity requirements not met")
            result = False
        response = get(FIRMWARE_VERSION, timeout=10.0)
        if response.status_code == 200:
            self._latest_firmware_version = response.text[:-1]
        else:
            self._latest_firmware_version = 'Unknown'

        return result

    def _update_updater(self):
        """
        If a different version of this updater script is available, retrieve
        and install it. This provides both an upgrade and a rollback mechanism.
        Exit and allow systemd to restart it.
        """

        if VERSION == self._latest_updater_version:
            logging.info(
                f"_update_updater: version {VERSION} is already the latest"
            )
            return

        self.close_fifo()
        logging.info(
            f'Replacing {VERSION} with {self._latest_updater_version}'
        )
        try:
            Path(UPDATE_SCRIPT).replace(UPDATE_SCRIPT+'.old')

        except Exception:
            pass

        response = get(UPDATE_URL, timeout=10.0)
        if response.status_code == 200:
            with open(UPDATE_SCRIPT, 'w') as f:
                f.write(response.text)
            Path(UPDATE_SCRIPT).chmod(0o554)
            logging.info('updater.py replaced')
            logging.warning('updater.py exiting')
            self._set_update_in_progress()

            exit(0)
        else:
            logging.warning('Failed to retrieve updater.py,'
                            f' status code {response.status_code}')

    def _status_msg(self, msg: str) -> None:
        """
        Use the HAT screen 4 status message capability to display
        msg. The HAT is opened and closed every time this method is
        called, in order to minimize any conflict with rq_core.
        Incidentally, the HAT does not persist status messages, so
        this method persists them and overrides an internal RQHAT
        variable.
        """

        _hat = RQHAT(
            HAT_SERIAL,
            38400,
            7,
            'N',
            1,
            1.0)
        _hat._status_lines = self._status_messages
        _hat.status_msg(msg)
        self._status_messages = _hat._status_lines
        _hat.close()

    def _get_latest_images(self):
        """
        This method kills any running containers. Docker doesn't provide a
        mechanism to compare the version of a local image to the version on
        a registry.
        """

        for container_name in CONTAINERS:
            try:
                container = self._client.containers.get(container_name)

            except docker.errors.NotFound:
                pass

            else:
                container.kill()
        self._client.containers.prune()

        for container_name in CONTAINERS:
            self._status_msg(f'checking {container_name}')
            image_name = CONTAINERS[container_name]['image_name']
            try:
                image_local = self._client.images.get(image_name)
            except docker.errors.ImageNotFound:
                image_local = None

            try:
                logging.info(
                    f"Pulling {image_name}"
                    f" {self._latest_image_versions[image_name]['version']}"
                    " from the registry"
                )
                self._status_msg(
                    f" pull {container_name}"
                    f" {self._latest_image_versions[image_name]['version']}"
                )
                image_registry = self._client.images.pull(
                    image_name,
                    tag='latest')

            except docker.errors.ImageNotFound:
                image_registry = None

            if not image_local and not image_registry:
                logging.fatal(
                    f'Image {image_name}'
                    f", does not exist on the registry")
                self._status_msg(' not available')
                continue

            if not image_local:
                logging.info(
                    f'Image {image_name}'
                    ' No local image'
                    f", registry {image_registry.labels['version']}")
            elif not image_registry:
                logging.warning(
                    f'Image {image_name}'
                    f" local: {image_local.labels['version']}"
                    ', No registry image')
            else:
                logging.info(
                    f'Image {image_name}'
                    f" local: {image_local.labels['version']}"
                    f", registry {image_registry.labels['version']}")

        self._status_msg('image update complete')

    def _update_images(self) -> None:
        """
        Update the images if a newer version exists or
        there is no local copy.
        """

        if self._requirements_met():
            self.close_fifo()
            self._update_updater()
            self._reset_update_in_progress()
            self._get_latest_images()
            self._setup_fifo()

            logging.info('update complete')
            self._status_msg('update complete')

    def _process_message(self, message: str) -> None:
        """
        Process the message received from the UI. The defined
        actions are : UPDATE, SHUTDOWN, REBOOT.

        The defined message:
            {
                action: 'UPDATE',
                args: ''
            }
        """

        try:
            command = json.loads(message)

        except json.decoder.JSONDecodeError:
            logging.warning('Unparsable command message: %s', message)
            return

        if command['action'].upper() == 'UPDATE':
            logging.info('UPDATE command with args: <%s>', command['args'])
            self._status_msg('UPDATE')
            self._update_images()
        elif command['action'].upper() == 'SHUTDOWN':
            logging.info('SHUTDOWN command with args: <%s>', command['args'])
            self._status_msg('SHUTDOWN')
            self._shutdown_cb('SHUTDOWN')
        elif command['action'].upper() == 'REBOOT':
            logging.info('REBOOT command with args: <%s>', command['args'])
            self._status_msg('REBOOT')
            self._reboot_cb('REBOOT')
        else:
            logging.warning('Unrecognized command message: %s', message)

    def _get_ros_domain_id(self):
        """
        Calculate an integer, between 1 and 101 inclusive, which is
        unique across the universe of Raspberry Pi units. Return
        the string ROS_DOMAIN_ID={integer}.
        """
        try:
            with open(SERIAL_NUMBER_FILE, 'r') as serial_file:
                cpuserial = serial_file.read()

            logging.info(f'CPU serial {cpuserial[:-1]}')
            return f'ROS_DOMAIN_ID={(int(cpuserial[:-1], base=16) % 100) + 1}'

        except Exception:
            logging.error('Failed to get CPU serial')
            return 'ROS_DOMAIN_ID=0'

    def _start_containers(self, to_start: list) -> None:
        """
        Start the containers listed in to_start. If the image for any
        container isn't available locally begin the update process.
        """

        images_exist = True
        for container_name in to_start:
            image_name = CONTAINERS[container_name]['image_name']
            image = self._client.images.list(
                name=image_name)

            if not image:
                logging.warning(f'The image {image_name}'
                                f' for container {container_name}'
                                ' does not exist locally')
                images_exist = False

        if images_exist:
            for container_name in to_start:
                image_name = CONTAINERS[container_name]['image_name']
                try:
                    _ = self._client.containers.run(
                        image_name,
                        name=container_name,
                        detach=True,
                        environment=[self._ros_domain_id],
                        privileged=CONTAINERS[container_name]['privileged'],
                        auto_remove=True,
                        devices=CONTAINERS[container_name]['devices'],
                        ipc_mode='host',
                        network_mode='host',
                        volumes=CONTAINERS[container_name]['volumes']
                    )

                except docker.errors.APIError as e:
                    logging.warning(f"Exception starting {container_name}:"
                                    f" {e}")

                else:
                    logging.info(f"Container {container_name} started")
        else:
            self._update_images()

    def _check_running_containers(self):
        """
        Verify the containers are running, starting them if necessary.
        Intended for use in several scenarios:
            1. the images are up-to-date but a container stopped
            2. new images have been installed
            3. images are missing
        """

        containers_to_start = list()

        for container_name in CONTAINERS:
            containers = self._client.containers.list(
                filters={'name': container_name})

            if not containers:
                logging.warning(f'No {container_name} container found')
                containers_to_start.append(container_name)
            else:
                if containers[0].status != 'running':
                    logging.warning(f'Container {container_name} not running')
                    containers_to_start.append(container_name)

        if containers_to_start:
            self._start_containers(containers_to_start)

    def _make_dirs(self, directories=[]) -> None:
        """
        Ensure the listed directories exist.
        """

        for directory in directories:
            d = Path(directory)
            d.mkdir(mode=0o775, exist_ok=True)

    def _restore_config(self, config_file: str) -> None:
        """
        Restore config_file with its old version, if the old version
        exists.
        In the special case where neither the config file nor its
        old version exists, the condition will be logged but nothing
        else will be done. This case occurs once with a
        never-before-used RoboQuest SD image OR after a catastrophic
        failure. In either event, a restart of the robot will cause
        the rq_ui container to install a default config file.
        """

        logging.info(f"Attempting to restore {config_file}")
        config_file_path = Path(OS_PERSIST_DIR) / config_file
        old_config_file_path = Path(OS_PERSIST_DIR) / (config_file + '.old')

        if old_config_file_path.exists():
            try:
                config_file_path.unlink(missing_ok=True)
                old_config_file_path.rename(config_file_path)
                logging.info(f"{config_file} restored from old version")

            except Exception as e:
                logging.warn(f"Failed to restore {config_file}: {e}")
        else:
            logging.warn(f"Old version of {config_file} does not exist")

        return

    def _get_installed_image_versions(self) -> list[tuple[str, str]]:
        """
        Query the installed images and retrieve their "version"
        LABEL. Return a list of tuples containing the name and
        version for each image.
        """

        installed_images = list()
        for container_name in CONTAINERS:
            image = self._client.images.get(
                CONTAINERS[container_name]['image_name']
            )
            installed_images.append((container_name, image.labels['version']))

        return installed_images

    def _write_version_object(self) -> None:
        """
        Collect the versions of the installed images and other components.
        Create and write ALL_VERSIONS.
        """

        all_versions = {
            'installed': {
                'updater': VERSION
            },
            'latest': {
                'firmware': self._latest_firmware_version,
                'updater': self._latest_updater_version
            }
        }

        installed_image_versions = self._get_installed_image_versions()
        for image in installed_image_versions:
            all_versions['installed'][image[0]] = image[1]
        # TODO: Retrieve current firmware version from HAT
        all_versions['installed']['firmware'] = 'Unknown'
        all_versions['installed']['updater'] = VERSION
        for image in self._latest_image_versions:
            image_details = self._latest_image_versions[image]
            all_versions['latest'][image_details['name']] = (
                image_details['version']
            )

        with open(ALL_VERSIONS, 'w') as f:
            f.write(json.dumps(all_versions))
        return

    def _publish_versions(self):
        """
        Called once when this script starts, before any containers are
        started. It's  expected this method will usually fail, because the
        robot usually does NOT have Internet connectivity.

        It extracts the "version" label from the local Docker images, the
        files UPDATE_VERSION and IMAGE_VERSIONS from the registry, and
        the VERSION constant from this script. All of that information is used
        to create ALL_VERSIONS.
        When images are updated, it's expected that this script will be
        restarted after the updates complete.
        """

        if self._requirements_met():
            #
            # Network connectivity exists and the latest version
            # string for updater.py is in self._updater_version.
            #
            response = get(IMAGE_VERSIONS, timeout=10.0)
            if response.status_code == 200:
                self._latest_image_versions = json.loads(response.text)
            else:
                logging.warning("Failed to retrieve IMAGE_VERSIONS")

        self._write_version_object()

        return

    def _check_configs(self, restore: bool = False) -> None:
        """
        Monitor configuration files for:
            1. missing
            2. empty
            3. corrupt
        When any of those conditions exist, log the occurence.
        If restore is True, attempt to restore the config file
        with the old version.
        Uses CONFIG_FILES for a list of configuration file base
        names to be found in OS_PERSIST_DIR directory. The configuration
        files must be JSON strings and must have the "version"
        attribute at their top level.
        """

        for config_file in CONFIG_FILES:
            config_file_path = Path(OS_PERSIST_DIR) / config_file

            if config_file_path.exists():
                if config_file_path.stat().st_size > 0:
                    configuration = json.loads(config_file_path.read_text())
                    if 'version' in configuration:
                        continue
                    else:
                        logging.warning(f"{config_file}"
                                        " missing version attribute")
                else:
                    logging.warning(f"{config_file}"
                                    " is empty")
            else:
                logging.warning(f"{config_file}"
                                " does not exist")

            if restore:
                self._restore_config(config_file)

        return

    def run(self) -> None:

        """
        The loop for monitoring the containers and watching for
        a command to update the containers.
        """

        self._status_msg(f'updater v{VERSION}')
        self._publish_versions()
        self._check_configs(restore=True)

        if self._update_in_progress():
            self._status_msg('resuming update')
            self._update_images()

        self._status_msg('starting RoboQuest')
        while True:
            self._check_running_containers()
            self._check_configs(restore=False)

            message = self.read_message()
            if message:
                self._process_message(message)
            else:
                sleep(LOOP_PERIOD_S)

    def _shutdown_cb(self, arg):
        """
        Called when the shutdown control signal has been detected and
        when the SHUTDOWN command is received.
        """

        self._status_msg('shutdown start')
        logging.info("Shutdown triggered")
        os.system('systemctl halt')

        #
        # Pause here for a bit, so updater.py doesn't try
        # to restart the containers after the OS is trying
        # to stop the docker daemon.
        #
        sleep(LONG_TIME)
        logging.warn('Woke unexpectedly from sleep')

    def _reboot_cb(self, arg):
        """
        Called when the REBOOT command is received.
        """

        self._status_msg('reboot start')
        logging.info("Reboot triggered")
        os.system('systemctl reboot')

        #
        # Pause here for a bit, so updater.py doesn't try
        # to restart the containers after the OS is trying
        # to stop the docker daemon.
        #
        sleep(LONG_TIME)
        logging.warn('Woke unexpectedly from sleep')

    def _remove_old_images(self):
        """
        Intended for use only at startup, before any containers
        have been started. Removes all old images, selected
        with the "dangling" filter because they don't have a tag.
        """

        old_images = self._client.images.list(
            filters={
                'dangling': True
            })

        for image in old_images:
            try:
                self._client.images.remove(image=image.id)

            except Exception as e:
                logging.warning(f"Failed to remove old {image.id}: {e}")


if __name__ == "__main__":

    rq_update = RQUpdate(UPDATE_FIFO)
    try:
        rq_update.run()

    except KeyboardInterrupt:
        rq_update.close_fifo()

    logging.warning('Shutdown')
