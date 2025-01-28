#!/usr/bin/env python3

"""
Manage the RoboQuest application.

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

import json
import logging
import os
from pathlib import Path, PurePath
from sys import exit
from time import sleep

import RPi.GPIO as GPIO

import docker

from requests import get

from rq_hat import RQHAT

VERSION = 16
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
DOCKER_VOLUMES_DIR = '/opt/docker/volumes'
ROS_LOGS = 'ros_logs'
UPDATER_DIR = '/opt/updater'
DIRECTORIES = [OS_PERSIST_DIR, UPDATER_DIR]
CONFIG_FILES = ['configuration.json']
UPDATE_LOG = UPDATER_DIR + '/updater.log'
LOG_SERVER_PID_FILE = UPDATER_DIR + '/log_server_pid'
LOG_SERVER_PORT = 8444
LOG_LINES = 100
UPDATE_IN_PROGRESS = UPDATER_DIR + '/update_in_progress'
UPDATE_FIFO = '/tmp/update_fifo'
UPDATE_VERSION = 'http://registry.q4excellence.com:8079/updater_version.txt'
FIRMWARE_VERSION = 'http://registry.q4excellence.com:8079/firmware_version.txt'
IMAGE_VERSIONS = 'http://registry.q4excellence.com:8079/image_versions.json'
ALL_VERSIONS = OS_PERSIST_DIR + '/versions.json'
UPDATE_SCRIPT = 'updater.py'
URL_BASE = 'http://registry.q4excellence.com:8079/'
UPDATE_URL = URL_BASE + UPDATE_SCRIPT
CERT_FILE = 'cert.pem'
KEY_FILE = 'key.pem'
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
                    ROS_LOGS+':/root/.ros/log']},
    'rq_ui': {
        'image_name': 'registry.q4excellence.com:5678/rq_ui',
        'privileged': False,
        'devices': [],
        'volumes': ['/dev/shm:/dev/shm',
                    UPDATE_FIFO+':'+UPDATE_FIFO,
                    OS_PERSIST_DIR+':'+RQ_UI_PERSIST,
                    ROS_LOGS+':/root/.ros/log']}
}


class RQUpdate(object):
    """All of the methods to manage the update process."""

    def __init__(self, fifo_path):
        """Initialize the variables."""
        self._make_dirs(DIRECTORIES)
        # TODO: Preserve the last N lines of UPDATE_LOG
        logging.basicConfig(
            filename=UPDATE_LOG,
            format='%(asctime)s %(levelname)s %(message)s',
            level=logging.INFO)
        logging.info(f'updater.py version {VERSION} started')

        #
        # A safety flag, to ensure the HAT serial port isn't touched
        # when any containers are running.
        #
        self._containers_running = False

        self._status_messages = []
        self._status_msg(f'updater.py version {VERSION}')

        self._ros_domain_id = self._get_ros_domain_id()
        logging.info(f'{self._ros_domain_id}')

        self._messages = []
        self._client = None
        self._fifo = None

        self._latest_updater_version = None
        self._latest_image_versions = None

        self._fifo_path = fifo_path

        self._setup_shutdown()
        self._setup_docker()
        self._remove_old_images()
        self._setup_fifo()

    def _start_log_server(self):
        """
        start_log_server.

        Create a daemon process to serve the updater log file. A daemon
        process is used so it can outlive its parent.
        """
        from multiprocessing import Process
        from http.server import BaseHTTPRequestHandler, HTTPServer
        from ssl import SSLContext, PROTOCOL_TLSv1_2
        from os import kill, chdir
        from signal import SIGKILL

        logging.info('Configuring log server')

        def log_server_task(working_directory):
            chdir(working_directory)

            def get_ssl_context(certfile, keyfile):
                context = SSLContext(PROTOCOL_TLSv1_2)
                context.load_cert_chain(certfile, keyfile)
                context.set_ciphers('@SECLEVEL=1:ALL')
                return context

            class LogServer(BaseHTTPRequestHandler):
                def do_GET(self):
                    if not Path('.' + self.path).exists():
                        self.send_response(404)
                        self.end_headers()

                        return

                    self.send_response(200)
                    self.send_header('Content-type', 'text/plain')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()

                    with open('.' + self.path, 'r') as f:
                        for entry in (f.readlines()[-LOG_LINES:]):
                            self.wfile.write(
                                bytes(entry, 'utf-8')
                            )

            log_server = HTTPServer(
                ('', LOG_SERVER_PORT),
                LogServer
            )
            context = get_ssl_context(
                UPDATER_DIR + '/' + CERT_FILE,
                UPDATER_DIR + '/' + KEY_FILE
            )
            log_server.socket = context.wrap_socket(
                log_server.socket,
                server_side=True
            )
            log_server.serve_forever()

        if Path(LOG_SERVER_PID_FILE).exists():
            logging.warning('Trying to stop previous log server')
            with open(LOG_SERVER_PID_FILE, 'r') as f:
                log_server_pid = f.read()

            try:
                kill(int(log_server_pid), SIGKILL)
            except Exception:
                logging.warning(
                    f'Failed to stop previous log server {log_server_pid}'
                )
            Path(LOG_SERVER_PID_FILE).unlink(missing_ok=True)
            self._reboot_cb('_start_log_server')

        log_server = Process(
            target=log_server_task,
            daemon=True,
            args=(PurePath(UPDATE_LOG).parent,)
        )
        logging.info('Starting log server')
        log_server.start()

        with open(LOG_SERVER_PID_FILE, 'w+') as f:
            f.write(f'{log_server.pid}')
        logging.info(f'Log server running as {log_server.pid}')

    def _update_in_progress(self) -> bool:
        """
        update_in_progress.

        Test whether an update was in progress before this script restarted.
        """
        if Path(UPDATE_IN_PROGRESS).exists():
            return True

        return False

    def _reset_update_in_progress(self) -> None:
        """Indicate there isn't an update in progress."""
        Path(UPDATE_IN_PROGRESS).unlink(missing_ok=True)

        return

    def _set_update_in_progress(self) -> None:
        """Indicate there is an update in progress."""
        Path(UPDATE_IN_PROGRESS).touch(mode=0o444)

        return

    def _setup_shutdown(self):
        """
        setup_shutdown.

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
        """Create the docker client."""
        self._client = docker.from_env()

    def _setup_fifo(self):
        """
        setup_fifo.

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
        """Close and remove the FIFO."""
        if self._fifo:
            self._fifo.close()
        Path(self._fifo_path).unlink(missing_ok=True)

        self._fifo = None

    def read_message(self) -> str:
        """Read a message, if available, and then return it."""
        if not self._messages:
            for message in self._fifo.readline().split(EOL):
                if message:
                    self._messages.append(message)

        if self._messages:
            message = self._messages.pop(0)
            return message

        return None

    def _update_other_files(self):
        """
        Update with new files.

        Retrieve other files from the registry, which are required by
        updater.py.
        """
        success = True

        for other_file in [CERT_FILE, KEY_FILE]:
            other_file_path = UPDATER_DIR + '/' + other_file
            if not Path(other_file_path).exists():
                file_content = get(URL_BASE + other_file, timeout=10.0)
                if file_content.status_code == 200:
                    Path(other_file_path).touch(mode=0o440)
                    with open(other_file_path, 'w') as f:
                        f.write(file_content.text)
                    logging.info(f'Installed {other_file_path}')
                else:
                    logging.warning(f'Failed to install {other_file_path}')
                    success = False

        if not success:
            self._status_msg('Other files install failed')
            self._status_msg('Check Internet connection')

    def _requirements_met(self) -> bool:
        """
        Confirm requirements are met.

        Confirm network connectivity is available and appears stable.
        """
        result = True

        response = get(UPDATE_VERSION, timeout=10.0)
        if response.status_code == 200:
            self._latest_updater_version = int(response.text)
        else:
            logging.warning('Network connectivity requirements not met')
            result = False
        response = get(FIRMWARE_VERSION, timeout=10.0)
        if response.status_code == 200:
            self._latest_firmware_version = response.text[:-1]
        else:
            self._latest_firmware_version = 'Unknown'

        return result

    def _update_updater(self):
        """
        Update this script.

        If a different version of this updater script is available, retrieve
        and install it. This provides both an upgrade and a rollback
        mechanism.
        """
        if VERSION == self._latest_updater_version:
            return

        self.close_fifo()
        logging.info(
            f'Replacing updater {VERSION} with {self._latest_updater_version}'
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
            logging.info(
                f'updater.py upgraded to {self._latest_updater_version}'
            )
            logging.warning('updater.py exiting without reboot')
            self.stop_containers()
            self._status_msg('updater.py exiting')
            self._set_update_in_progress()

            Path(LOG_SERVER_PID_FILE).unlink(missing_ok=True)
            exit(0)
        else:
            logging.warning('Failed to retrieve updater.py,'
                            f' status code {response.status_code}')

    def _status_msg(self, msg: str) -> None:
        """
        Show a status message.

        Use the HAT screen 4 status message capability to display
        msg. The HAT is opened and closed every time this method is
        called, in order to minimize any conflict with rq_core.
        Incidentally, the HAT does not persist status messages, so
        this method persists them and overrides an internal RQHAT
        variable.
        """
        if self._containers_running:
            logging.warning('_status_msg: containers running')
            return

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

    def stop_containers(self):
        """Kill any running containers."""
        logging.info('Stopping containers')
        for container_name in CONTAINERS:
            try:
                container = self._client.containers.get(container_name)

            except docker.errors.NotFound:
                pass

            else:
                container.kill()
                logging.info(f'Stopped {container_name}')

        self._containers_running = False
        logging.info('Containers stopped')
        self._client.containers.prune()

    def _get_latest_images(self):
        """
        Get the latest images.

        Docker doesn't provide a mechanism to compare the version of a local
        image to the version on a registry.
        """
        self.stop_containers()

        for container_name in CONTAINERS:
            logging.info(f'checking {container_name}')
            image_name = CONTAINERS[container_name]['image_name']
            try:
                image_local = self._client.images.get(image_name)
            except docker.errors.ImageNotFound:
                image_local = None

            try:
                logging.info(
                    f'Pulling {image_name}'
                    f" {self._latest_image_versions[image_name]['version']}"
                    ' from the registry'
                )
                image_registry = self._client.images.pull(
                    image_name,
                    tag='latest')

            except docker.errors.ImageNotFound:
                image_registry = None

            if not image_local and not image_registry:
                logging.fatal(
                    f'Image {image_name}'
                    f', does not exist on the registry'
                )
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

        logging.info('image updates complete')

    def _update_images(self) -> None:
        """
        Update to latest images.

        Update the images if a newer version exists or
        there is no local copy.
        """
        if self._requirements_met():
            self.close_fifo()
            self._update_updater()
            self._reset_update_in_progress()
            self._get_latest_images()
            self._setup_fifo()

            logging.info('update process complete')
            self._status_msg('update process complete')

    def _process_message(self, message: str) -> None:
        """
        Process the message received from the UI.

        The defined actions are : UPDATE, SHUTDOWN, REBOOT.
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
        Calculate the ROS DOMAIN ID.

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
        Start all defined containers.

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
            self._containers_running = True

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
                    logging.warning(f'Exception starting {container_name}:'
                                    f' {e}')

                else:
                    logging.info(f'Container {container_name} started')
        else:
            self._update_images()

    def _check_running_containers(self):
        """
        Confirm containers are running.

        Verify the containers are running, starting them if necessary.
        Intended for use in several scenarios:
            1. the images are up-to-date but a container stopped
            2. new images have been installed
            3. images are missing
        """
        containers_to_start = []

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
        """Ensure the listed directories exist."""
        for directory in directories:
            d = Path(directory)
            d.mkdir(mode=0o775, exist_ok=True)

    def _restore_config(self, config_file: str) -> None:
        """
        Restore config_file with its old version.

        If the old version doesn't exist, the config_file will
        be removed. This scenario could occur when the config_file
        is empty or corrupt.
        """
        logging.warning(f'Attempting to restore {config_file}')
        config_file_path = Path(OS_PERSIST_DIR) / config_file
        old_config_file_path = Path(OS_PERSIST_DIR) / (config_file + '.old')

        if old_config_file_path.exists():
            try:
                config_file_path.unlink(missing_ok=True)
                old_config_file_path.rename(config_file_path)
                logging.info(f'{config_file} restored from old version')

            except Exception as e:
                logging.warning(f'Failed to restore {config_file}: {e}')
        else:
            logging.warning(f'Old version of {config_file} does not exist')
            logging.warning(f'Removing {config_file}')
            config_file_path.unlink(missing_ok=True)

        return

    def _get_installed_image_versions(self) -> list[tuple[str, str]]:
        """
        Get the image versions.

        Query the installed images and retrieve their "version"
        LABEL. Return a list of tuples containing the name and
        version for each image.
        """
        installed_images = []
        for container_name in CONTAINERS:
            image = self._client.images.get(
                CONTAINERS[container_name]['image_name']
            )
            installed_images.append((container_name, image.labels['version']))

        return installed_images

    def _write_version_object(self) -> None:
        """
        Write the version object to a file.

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
        Publish the software versions.

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
                logging.warning(f'Failed to retrieve {IMAGE_VERSIONS}')

        self._write_version_object()

        return

    def _check_ros_logs(self) -> None:
        """
        Check the ROS_LOGS directory exists.

        If the ROS_LOGS directory exists, return. Otherwise, attempt to
        create it and set both its mode and ownership.
        """
        ros_logs_dir = Path(DOCKER_VOLUMES_DIR) / ROS_LOGS / '_data'
        if ros_logs_dir.exists():
            return

        logging.warning(f'{ros_logs_dir} does not exist')

        try:
            #
            # If any of the parents don't exist, there are likely bigger
            # filesystem problems.
            #
            ros_logs_dir.mkdir(
                mode=0o755,
                parents=True,
                exist_ok=True
            )

            logging.info(f'{ros_logs_dir} created')

        except Exception as e:
            logging.warning(f'Failed to create ros_logs: {e}')

    def _check_configs(self, restore: bool = False) -> None:
        """
        Check the configurations.

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
                    try:
                        configuration = json.loads(
                            config_file_path.read_text()
                        )

                    except json.decoder.JSONDecodeError:
                        configuration = None

                    if configuration and 'version' in configuration:
                        continue
                    else:
                        logging.warning(f'{config_file}'
                                        ' missing version attribute')
                else:
                    logging.warning(f'{config_file}'
                                    ' is empty')
            else:
                logging.warning(f'{config_file}'
                                ' does not exist')

            if restore:
                self._restore_config(config_file)

        return

    def run(self) -> None:
        """
        Run the main loop.

        The loop for monitoring the containers and watching for
        a command to update the containers.
        """
        self._update_other_files()
        self._start_log_server()
        self._publish_versions()
        self._check_ros_logs()
        self._check_configs(restore=True)

        if self._update_in_progress():
            logging.info('resuming update in progress')
            self._update_images()

        logging.info('starting RoboQuest')
        self._status_msg('starting RoboQuest')
        while True:
            self._check_configs(restore=False)
            self._check_running_containers()

            message = self.read_message()
            if message:
                self._process_message(message)
            else:
                sleep(LOOP_PERIOD_S)

    def _shutdown_cb(self, arg):
        """
        Shutdown the robot.

        Called when the shutdown control signal has been detected and
        when the SHUTDOWN command is received.
        """
        logging.warning('Shutdown triggered')
        self.stop_containers()
        self._status_msg('Shutdown triggered')
        Path(LOG_SERVER_PID_FILE).unlink(missing_ok=True)
        os.system('systemctl halt')

        #
        # Pause here for a bit, so updater.py doesn't try
        # to restart the containers after the OS is trying
        # to stop the docker daemon.
        #
        sleep(LONG_TIME)
        logging.warning('Woke unexpectedly from sleep')

    def _reboot_cb(self, arg):
        """Reboot the robot."""
        logging.info(f'Reboot triggered by {arg}')
        self.stop_containers()
        self._status_msg('Reboot triggered')
        Path(LOG_SERVER_PID_FILE).unlink(missing_ok=True)
        os.system('systemctl reboot')

        #
        # Pause here for a bit, so updater.py doesn't try
        # to restart the containers after the OS is trying
        # to stop the docker daemon.
        #
        sleep(LONG_TIME)
        logging.warning('Woke unexpectedly from sleep')

    def _remove_old_images(self):
        """
        Remove old images.

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
                logging.warning(f'Failed to remove old {image.id}: {e}')


if __name__ == '__main__':

    rq_update = RQUpdate(UPDATE_FIFO)
    try:
        rq_update.run()

    except KeyboardInterrupt:
        rq_update.close_fifo()

    rq_update.stop_containers()
    Path(LOG_SERVER_PID_FILE).unlink(missing_ok=True)
    logging.warning('Shutdown')
