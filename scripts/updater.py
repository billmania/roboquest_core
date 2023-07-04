#!/usr/bin/env python3

from os import open as osopen, fdopen, unlink, O_RDONLY, O_NONBLOCK
from time import sleep

"""
A utility script to manage the updating of RoboQuest docker images.
"""


class RQUpdate(object):
    """
    All of the methods to manage the update process.
    """

    def __init__(self, fifo_path):
        """
        TODO:
        """

        self._messages = list()
        self._fifo_path = fifo_path
        self._fifo = None
        self._setup_fifo(self._fifo_path)

    def _setup_fifo(self, fifo_path):
        """
        Ensure the FIFO rendezvous point exists and configure it for
        non-blocking reads.
        """

        # test if it exists and create if not

        # open it as non-blocking and for reading
        fifo_fd = osopen(fifo_path, O_RDONLY | O_NONBLOCK)

        # create file object from the file descriptor
        self._fifo = fdopen(fifo_fd)

    def close(self) -> None:
        """
        Close and remove the FIFO.
        """

        self._fifo.close()
        unlink(self._fifo_path)

    def read_message(self) -> str:
        """
        If a message is available to read from the FIFO, read and return it.
        """

        if not self._messages:
            for message in self._fifo.readline().split('\n'):
                if message:
                    self._messages.append(message)

        if self._messages:
            message = self._messages.pop(0)
            return message

        return None


if __name__ == "__main__":
    rq_update = RQUpdate('/tmp/fifo')

    for counter in range(1000):
        message = rq_update.read_message()
        if message:
            print(message)
        else:
            sleep(1.0)

    rq_update.close()
