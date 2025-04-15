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
from signal import SIGHUP, SIGINT, SIGTERM, SIG_IGN, signal
from socket import AF_INET, SOCK_DGRAM, socket
from sys import exit
from time import sleep

import RPi.GPIO as GPIO

import docker

from requests import get
from requests.exceptions import ConnectionError as GetConnectionError

from rq_hat import RQHAT

VERSION = 20
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
UPDATER_VERSION = 'http://registry.q4excellence.com:8079/updater_version.txt'
FIRMWARE_VERSION = 'http://registry.q4excellence.com:8079/firmware_version.txt'
IMAGE_VERSIONS = 'http://registry.q4excellence.com:8079/image_versions.json'
ALL_VERSIONS = OS_PERSIST_DIR + '/versions.json'
UPDATE_SCRIPT = 'updater.py'
URL_BASE = 'http://registry.q4excellence.com:8079/'
MANAGE_FILES_MODULE = 'manage_files.py'
UPDATE_URL = URL_BASE + UPDATE_SCRIPT
CERT_FILE = 'cert.pem'
KEY_FILE = 'key.pem'
LOOP_PERIOD_S = 3.0
GET_TIMEOUT_S = 9.0
LONG_TIME = 60
EOL = '\n'
NULL_CHAR = '\0'

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

        signal(SIGHUP, self.shutdown)
        signal(SIGINT, self.shutdown)
        signal(SIGTERM, self.shutdown)

        #
        # A safety flag, to ensure the HAT serial port isn't touched
        # when any containers are running.
        #
        self._containers_running = False
        self._setup_HAT()

        self._status_messages = []
        self._status_msg(f'updater.py {VERSION}')

        self._ros_domain_id = self._get_ros_domain_id()
        logging.info(f'{self._ros_domain_id}')

        self._messages = []
        self._client = None

        self._latest_image_versions = None
        # TODO: Retrieve installed firmware version from HAT
        self._all_versions = {
            'installed': {
                'updater': VERSION,
                'firmware': 'Unknown'
            },
            'latest': {}
        }

        self._setup_docker()
        self._remove_old_images()

        self._fifo = None
        self._fifo_path = fifo_path
        self._setup_fifo()

    def _get_local_ip(self) -> str:
        """Get the IP address.

        Return the IP address of an UP interface which has a default
        route.
        """
        s = socket(AF_INET, SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(('10.254.254.254', 1))
            IP = s.getsockname()[0]

        except Exception:
            IP = '127.0.0.1'

        finally:
            s.close()

        return IP

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
            GPIO.RISING
        )

    def _setup_docker(self):
        """Create the docker client."""
        self._client = docker.from_env()

    def _setup_fifo(self):
        """
        setup_fifo.

        Ensure the FIFO rendezvous point exists and configure it for
        non-blocking reads.
        """
        try:
            os.mkfifo(self._fifo_path)

        except FileExistsError:
            pass

        fifo_fd = os.open(self._fifo_path, os.O_RDONLY | os.O_NONBLOCK)

        self._fifo = os.fdopen(fifo_fd)

    def _close_fifo(self) -> None:
        """Close and remove the FIFO."""
        try:
            if self._fifo:
                self._fifo.close()

        except Exception:
            pass

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

    def _install_manage_files(self) -> None:
        """Install the ManageFiles module."""
        logging.info('Installing the ManageFiles module')
        try:
            response = get(
                URL_BASE + MANAGE_FILES_MODULE,
                timeout=GET_TIMEOUT_S
            )

        except GetConnectionError:
            logging.warning('No Internet connectivity')
            return

        if response.status_code == 200:
            with open(MANAGE_FILES_MODULE, 'w') as f:
                f.write(response.text)
            Path(MANAGE_FILES_MODULE).chmod(0o444)
        else:
            logging.warning(
                f'Failed to retrieve {MANAGE_FILES_MODULE}'
            )

    def _update_files(self):
        """
        Update the local files.

        Use the ManageFiles class to maintain the files
        installed locally. If the module isn't  already
        installed, cause it to be installed and then force
        a restart of updater.py.
        """
        try:
            from manage_files import ManageFiles

        except ModuleNotFoundError:
            self._install_manage_files()
            logging.warning('Restarting updater.py')
            exit(0)

        MF = ManageFiles(logging)
        MF.get_config_file()
        MF.install_files()
        MF.remove_files()

    def _Internet_connected(self) -> bool:
        """
        Confirm requirements are met.

        Confirm Internet connectivity is available and appears stable.
        """
        logging.info('Checking Internet connectivity')
        self._status_msg('Checking Internet')
        try:
            response = get(
                UPDATER_VERSION,
                params=self._query_string(),
                timeout=GET_TIMEOUT_S
            )
        except GetConnectionError:
            logging.warning('No Internet connectivity')
            return False

        if response.status_code == 200:
            self._all_versions['latest']['updater'] = int(response.text)
        else:
            logging.warning('No Internet connectivity')
            return False

        try:
            response = get(
                FIRMWARE_VERSION,
                params=self._query_string(),
                timeout=GET_TIMEOUT_S
            )
        except GetConnectionError:
            logging.warning('No Internet connectivity')
            return False

        if response.status_code == 200:
            self._all_versions['latest']['firmware'] = response.text[:-1]
        else:
            logging.warning(f'Failed to get {FIRMWARE_VERSION}')
            return False

        return True

    def _update_updater(self):
        """
        Update this script.

        If a different version of this updater script is available, retrieve
        and install it. This provides both an upgrade and a rollback
        mechanism.
        """
        if VERSION == self._all_versions['latest']['updater']:
            return

        self._close_fifo()
        logging.info(
            f'Replacing updater {VERSION} with'
            f" {self._all_versions['latest']['updater']}"
        )
        try:
            Path(UPDATE_SCRIPT).replace(UPDATE_SCRIPT+'.old')

        except Exception:
            pass

        response = get(
            UPDATE_URL,
            timeout=GET_TIMEOUT_S
        )
        if response.status_code == 200:
            with open(UPDATE_SCRIPT, 'w') as f:
                f.write(response.text)
            Path(UPDATE_SCRIPT).chmod(0o554)
            logging.info(
                'updater.py upgraded to'
                f" {self._all_versions['latest']['updater']}"
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

    def _setup_HAT(self):
        """Prepare the HAT.

        The HAT is used to display status messages. The connection
        to the HAT must be destroyed before the containers are started,
        to prevent conflict with the containers.
        """
        self._hat = RQHAT(
            HAT_SERIAL,
            38400,
            7,
            'N',
            1,
            1.0)
        self._hat.control_comms(enable=False)

    def _close_hat(self):
        """Close the HAT connection.

        Completely reset and shutdown the serial port and the GPIO
        sub-system.
        """
        self._hat.close()

    def _status_msg(self, msg: str = None) -> None:
        """
        Add a status message to screen 4.

        Use the HAT screen 4 status message capability to display
        msg. The HAT is opened and closed every time this method is
        called, in order to minimize any conflict with rq_core.
        Incidentally, the HAT does not persist status messages, so
        this method persists them and overrides an internal RQHAT
        variable.

        In order to refresh the display of status messages, without
        adding another status line, set the msg argument to None.
        Setting msg to the empty string will add a blank line to
        the status display.
        """
        if self._containers_running:
            if msg:
                logging.warning(f'HAT UI not available for: {msg}')
            return

        #
        # This method deliberately stomps on a private class variable,
        # _hat._status_lines. Since the private variable is initialized
        # to the empty list by the constructor, the stomping provides a
        # means to persist previous updater.py status lines and to
        # refresh the display without adding another line.
        #
        self._hat._status_lines = self._status_messages
        if msg is not None:
            self._hat.status_msg(msg)
            self._status_messages = self._hat._status_lines
        else:
            self._hat.show_status_msgs()

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
        self._setup_HAT()
        logging.info('Containers stopped')
        self._status_msg('Containers stopped')
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
        self._status_msg('Updating images')

        if self._Internet_connected():
            self._close_fifo()
            self._update_updater()
            self._reset_update_in_progress()
            self._get_latest_images()
            self._setup_fifo()

            logging.info('update process complete')
            self._status_msg('images updated')

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
                cpuserial_raw = serial_file.read()
            self._cpuserial = ''.join(filter(
                lambda character: character != NULL_CHAR,
                cpuserial_raw
            ))

            logging.info(f'CPU serial {self._cpuserial}')
            self._unique_id = (
                int(self._cpuserial, base=16)
                % 100) + 1
            return f'ROS_DOMAIN_ID={self._unique_id}'

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
                self._status_msg(f'Missing {container_name}')
                images_exist = False

        if images_exist:
            self._status_msg('Starting containers')
            #
            # They're not running yet, but will soon be.
            #
            self._containers_running = True
            self._close_hat()

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
            try:
                image = self._client.images.get(
                    CONTAINERS[container_name]['image_name']
                )
                installed_images.append(
                    (container_name, image.labels['version'])
                )

            except docker.errors.ImageNotFound:
                logging.warning(f'Image {container_name} not present locally')
                installed_images.append(
                    (container_name, 'NA')
                )

        return installed_images

    def _write_version_object(self) -> None:
        """
        Write the version object to a file.

        Collect the versions of the installed images and other components.
        Create and write ALL_VERSIONS.
        """
        installed_image_versions = self._get_installed_image_versions()
        for image in installed_image_versions:
            self._all_versions['installed'][image[0]] = image[1]
        try:
            for image in self._latest_image_versions:
                image_details = self._latest_image_versions[image]
                self._all_versions['latest'][image_details['name']] = (
                    image_details['version']
                )
        except TypeError:
            pass

        with open(ALL_VERSIONS, 'w') as f:
            f.write(json.dumps(self._all_versions))
        return

    def _publish_versions(self):
        """
        Publish the software versions.

        Called once when this script starts, before any containers are
        started. It's  expected this method will usually fail to publish
        a full dictionary of versions, because the robot usually does NOT
        have Internet connectivity.
        """
        self._write_version_object()

        if self._Internet_connected():
            #
            # Retrieve the version numbers for the newest images.
            #
            response = get(
                IMAGE_VERSIONS,
                timeout=GET_TIMEOUT_S
            )
            if response.status_code == 200:
                self._latest_image_versions = json.loads(response.text)
                self._write_version_object()
            else:
                logging.warning(f'Failed to retrieve {IMAGE_VERSIONS}')

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

    def _get_uptime(self) -> str:
        """Get the system uptime."""
        with open('/proc/uptime', 'r') as f:
            uptime = f.readline().split()[0]

        return uptime

    def _query_string(self) -> dict:
        """
        Create the query string parameters for requests.

        The string contains information about the current
        environment.

        serial is the Raspberry Pi serial number
        id is the value assigned to ROS_DOMAIN_ID
        uptime is the elapsed seconds since the OS started
        up is the version of the updater script
        core is the installed version of the rq_core image
        ui is the installed version of the rq_ui image
        """
        try:
            rq_core_version = self._all_versions['installed']['rq_core']

        except KeyError:
            rq_core_version = None

        try:
            rq_ui_version = self._all_versions['installed']['rq_ui']

        except KeyError:
            rq_ui_version = None

        return {
            'serial': self._cpuserial,
            'id': self._unique_id,
            'uptime': self._get_uptime(),
            'updater': VERSION,
            'core': rq_core_version,
            'ui': rq_ui_version
        }

    def run(self) -> None:
        """
        Run the main loop.

        The loop for monitoring the containers and watching for
        a command to update the containers.
        """
        self._update_files()
        self._start_log_server()
        self._publish_versions()
        self._status_msg(
            f"c:{self._all_versions['installed']['rq_core']}"
            f" u:{self._all_versions['installed']['rq_ui']}"
        )
        self._status_msg(
            f'IP:{self._get_local_ip()}'
        )
        self._check_ros_logs()
        self._check_configs(restore=True)
        self._status_msg()

        if self._update_in_progress():
            logging.info('resuming update in progress')
            self._status_msg()
            self._update_images()

        logging.info('starting RoboQuest')
        self._status_msg('starting RoboQuest')
        self._setup_shutdown()
        while True:
            try:
                if GPIO.event_detected(SHUTDOWN_PIN):
                    self._shutdown_cb('BUTTON')

            except RuntimeError:
                self._setup_shutdown()

            self._check_configs(restore=False)
            self._check_running_containers()

            message = self.read_message()
            if message:
                self._process_message(message)
            else:
                sleep(LOOP_PERIOD_S)

    def _shutdown_cb(self, arg='UNKNOWN'):
        """
        Shutdown the robot.

        Called when the shutdown control signal has been detected and
        when the SHUTDOWN command is received.
        """
        if arg == 'SHUTDOWN':
            logging.warning('Shutdown triggered by UI')
        elif type(arg) is int:
            logging.warning('Shutdown button pressed')
        else:
            logging.warning(f'Shutdown by {arg}')

        os.system('systemctl halt')

        #
        # Pause here, so updater.py doesn't try
        # to restart the containers while systemd is
        # stopping the docker daemon.
        #
        sleep(LONG_TIME)

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

    def shutdown(
        self,
        signal_number=None,
        stack_frame=None
    ):
        """Ready the updater to be shutdown.

        This method must complete quickly, because it could have been
        called when a signal was received from systemd.
        """
        signal(SIGHUP, SIG_IGN)
        signal(SIGINT, SIG_IGN)
        signal(SIGTERM, SIG_IGN)

        if signal_number:
            logging.info(f'Shutdown signal: {signal_number}')
        else:
            logging.info('Shutdown by user')
        logging.shutdown()
        self._close_fifo()
        Path(LOG_SERVER_PID_FILE).unlink(missing_ok=True)
        self.stop_containers()
        self._status_msg('Shutdown')

        exit(0)


if __name__ == '__main__':

    rq_update = RQUpdate(UPDATE_FIFO)

    rq_update.run()
    rq_update.shutdown()
