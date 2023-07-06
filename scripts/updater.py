#!/usr/bin/env python3

import os
from sys import exit
from pathlib import Path
from requests import get
import logging
import json
import docker
from time import sleep

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
    - /var/log/updater.log
    - systemd service with auto restart
"""

CONTAINERS = {
    'rq_core': {
        'image_name': 'registry.q4excellence.com:5678/rq_core',
        'devices': ['/dev/gpiomem:/dev/gpiomem:rwm',
                    '/dev/i2c-1:/dev/i2c-1:rwm',
                    '/dev/i2c-6:/dev/i2c-6:rwm',
                    '/dev/ttyS0:/dev/ttyS0:rwm',
                    '/dev/video0:/dev/video0:rwm'],
        'volumes': ['/dev/shm:/dev/shm',
                    '/var/run/dbus:/var/run/dbus',
                    'ros_logs:/root/.ros/log']},
    'rq_ui': {
        'image_name': 'registry.q4excellence.com:5678/rq_ui',
        'devices': [],
        'volumes': ['/dev/shm:/dev/shm',
                    'ros_logs:/root/.ros/log']}
}
UPDATE_LOG = '/var/log/updater.log'
UPDATE_FIFO = '/tmp/update_fifo'
UPDATE_VERSION = 'http://registry.q4excellence.com:8079/updater_version.txt'
UPDATE_SCRIPT = 'http://registry.q4excellence.com:8079/updater.py'
LOOP_PERIOD_S = 10.0
EOL = '\n'
VERSION = 2


class RQUpdate(object):
    """
    All of the methods to manage the update process.
    """

    def __init__(self, fifo_path):
        """
        Setup all of the stuff.
        """

        logging.basicConfig(
            filename=UPDATE_LOG,
            format='%(asctime)s %(levelname)s %(message)s',
            level=logging.INFO)

        self._messages = list()
        self._client = None
        self._fifo = None
        self._updater_version = None

        self._fifo_path = fifo_path

        self._setup_docker()
        self._setup_fifo()

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
        Confirm that electrical power and network connectivity
        are available and appear stable.
        """

        result = True

        response = get(UPDATE_VERSION, timeout=10.0)
        if response.status_code == 200:
            self._updater_version = int(response.text)
        else:
            logging.warning("Network connectivity requirements not met")
            result = False

        # TODO: Implement check for electrical

        return result

    def _update_updater(self):
        """
        If a different version of this updater script is available, retrieve
        and install it. This provides both an upgrade and a rollback mechanism.
        Exit and allow systemd to restart it.
        """

        if VERSION == self._updater_version:
            return

        self.close_fifo()
        logging.info(f'Replacing {VERSION} with {self._updater_version}')

        response = get(UPDATE_SCRIPT, timeout=10.0)
        if response.status_code == 200:
            with open('updater.py', 'w') as f:
                f.write(response.text)
            logging.info('updater.py replaced')
            logging.warning('updater.py exiting')
            exit(0)
        else:
            logging.warning('Failed to retrieve updater.py,'
                            f' status code {response.status_code}')

    def _get_latest_images(self):
        """
        Allow the docker registry to determine if an image must be
        pulled.
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
            image_name = CONTAINERS[container_name]['image_name']
            _ = self._client.images.pull(image_name)
            logging.info(f'Pulled image {image_name}')

    def _update_images(self) -> None:
        """
        Update the images if a newer version exists or
        there is no local copy.
        """

        if self._requirements_met():
            self.close_fifo()
            self._update_updater()
            self._get_latest_images()
            self._setup_fifo()

    def _process_message(self, message: str) -> None:
        """
        Process the message received from the UI.

        The defined messages:

            {
                timestamp: 1234567890,
                version: 1
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
            self._update_images()
        else:
            logging.warning('Unrecognized command message: %s', message)

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

    def run(self) -> None:

        """
        The loop for monitoring the containers and watching for
        a command to update the containers.
        """

        while True:
            self._check_running_containers()

            message = self.read_message()
            if message:
                self._process_message(message)
            else:
                sleep(LOOP_PERIOD_S)


if __name__ == "__main__":

    rq_update = RQUpdate(UPDATE_FIFO)
    try:
        rq_update.run()

    except KeyboardInterrupt:
        rq_update.close_fifo()

    logging.warning('Shutdown')
