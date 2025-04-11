"""Manage files.

A utility class to handle the installation and removal of files
on the robot's base OS.
"""

from json import loads as jloads
from typing import Callable

from requests import get


CONFIG_FILE_URL = 'https://registry.q4excellence.com:5678/manage_files.json'
GET_TIMEOUT_S = 5.0


class ManageFiles(object):
    """Manage file installation and removal."""

    def __init__(self, logger: Callable):
        """Prepare to manage files."""
        if logger:
            self._log_debug = logger.debug
            self._log_info = logger.info
            self._log_warning = logger.warning
            self._log_error = logger.error
        else:
            self._log_debug = print
            self._log_info = print
            self._log_warning = print
            self._log_error = print

    def _parse_config_file(self, raw_config_file) -> None:
        """Parse the contents of the config file."""
        try:
            self._file_config = jloads(raw_config_file)
        except Exception as e:
            self._log_error(
                f'Failed to parse {raw_config_file}'
                f' : {e}'
            )

        self._log_debug(
            f"config file parsed installs: {self._file_config['install']}"
        )
        self._log_debug(
            f"config file parsed removes: {self._file_config['remove']}"
        )

    def get_config_file(self) -> None:
        """Get the configuration file.

        Retrieve the file management configuration file from
        the registry and parse it into a Python object.
        """
        try:
            config_file_content = get(
                CONFIG_FILE_URL,
                timeout=GET_TIMEOUT_S
            )
            if config_file_content.status_code == 200:
                self._log_debug(
                    'config file raw content:'
                    f' {config_file_content.text}'
                )
            else:
                self._log_error(
                    'config file content status:'
                    f' {config_file_content.status_code}'
                )

        except Exception as e:
            self._log_error(
                'Exception getting file management config file'
                f' {CONFIG_FILE_URL}'
                f' : {e}'
            )
            self._config_file = None

        self._parse_config_file(config_file_content.text)

    def remove_files(self) -> None:
        """Remove files.

        Use the file management object to get the list of files
        to remove. Remove those files.
        """
        pass

    def install_files(self) -> None:
        """Install files.

        Use the file management object to get the list of files
        to be installed. Install those files.
        """
        pass


if __name__ == '__main__':
    MF = ManageFiles(None)
    MF.get_config_file()
