"""Manage files.

A utility class to handle the installation and removal of files
on the robot's base OS.
"""

from grp import getgrnam
from hashlib import md5
from json import loads as jloads
from os import chown
from pathlib import Path
from pwd import getpwnam
from typing import Callable

from requests import get


BASE_URL = 'https://registry.q4excellence.com:5678/'
CONFIG_FILE_URL = BASE_URL + 'manage_files.json'
BASE_FILE_URL = BASE_URL + 'files/'
GET_TIMEOUT_S = 5.0


class ManageFiles(object):
    """Manage file installation and removal."""

    def __init__(self, logging: Callable):
        """Prepare to manage files."""
        if logging:
            self._log_info = logging.info
            self._log_warning = logging.warning
            self._log_error = logging.error
        else:
            self._log_info = print
            self._log_warning = print
            self._log_error = print

    def _parse_config_file(self, raw_config_file) -> dict:
        """Parse the contents of the config file."""
        try:
            file_config = jloads(raw_config_file)

        except Exception as e:
            self._log_error(
                f'Failed to parse {raw_config_file}'
                f' : {e}'
            )
            file_config = None

        return file_config

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
            if config_file_content.status_code != 200:
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

        else:
            self._config_file = self._parse_config_file(
                config_file_content.text
            )

    def _remove_file(self, full_path) -> None:
        """Remove the file."""
        try:
            remove_path = Path(full_path)
            remove_path.unlink(missing_ok=True)

        except Exception as e:
            self._log_warning(
                f'Failed to remove {full_path}'
                f' Exception: {e}'
            )

    def remove_files(self) -> None:
        """Remove files.

        Use the file management object to get the list of files
        to remove. Remove those files. If the file to be removed
        has changed on the robot, it won't be removed, unless the
        "force" attribute is set.
        """
        for file in self._config_file['remove']:
            full_path = file['full_path']

            if not Path(full_path).exists():
                continue

            try:
                if not self._file_changed(
                    full_path,
                    file['md5sum']
                ) or file['force']:
                    self._log_info(
                        f'Removing: {full_path}'
                    )
                    self._remove_file(full_path)
                else:
                    self._log_warning(
                        f'{full_path} note removed. set "force"'
                    )

            except Exception as e:
                self._log_warning(
                    f'Exception removing: {full_path}'
                    f' : {e}'
                )

    def _file_changed(self, full_path, md5sum) -> bool:
        """Check the state of the file.

        If it doesn't exist, return True.
        If it exists but doesn't match the md5sum of the source
        file, return True.
        Otherwise return False.
        """
        if Path(full_path).exists():
            try:
                hasher = md5()
                with open(full_path, 'rb') as f:
                    hasher.update(f.read())
                local_digest = hasher.hexdigest()
            except Exception as e:
                self._log_warning(
                    f'Failed on hash: {e}'
                )
                return True

            if local_digest == md5sum:
                return False

        return True

    def _retrieve_file(self, source_file) -> str:
        """Retrieve the source file contents."""
        try:
            config_file_content = get(
                BASE_FILE_URL+source_file,
                timeout=GET_TIMEOUT_S
            )
            if config_file_content.status_code == 200:
                return config_file_content.text
                self._log_error(
                    'config file content status:'
                    f' {config_file_content.status_code}'
                )

        except Exception as e:
            self._log_warning(
                f'Failed to retrieve source file {source_file}'
                f' Exception: {e}'
            )
            return None

    def _install_file(self, file_content, file_details) -> None:
        """Install an individual file.

        Install file_content into the full path and set
        the owner, group, and mode.
        """
        full_path = Path(file_details['full_path'])
        full_path.unlink(missing_ok=True)
        try:
            full_path.touch(mode=file_details['mode'])
            with open(full_path, 'w') as f:
                f.write(file_content)

            chown(
                full_path,
                getpwnam(file_details['owner']).pw_uid,
                getgrnam(file_details['group']).gr_gid
            )
        except Exception as e:
            self._log_warning(
                f"Failed to install {file_details['full_path']}"
                f' Exception: {e}'
            )

    def install_files(self) -> None:
        """Install files.

        Use the file management object to get the list of files
        to be installed. Install those files.
        """
        for file in self._config_file['install']:
            try:
                if self._file_changed(
                    file['details']['full_path'],
                    file['md5sum']
                ):
                    file_content = self._retrieve_file(file['source'])
                    if file_content:
                        self._log_info(
                            f"Installing: {file['details']['full_path']}"
                        )
                        self._install_file(
                            file_content,
                            file['details']
                        )

            except Exception as e:
                self._log_warning(
                    f"Exception installing: {file['details']['full_path']}"
                    f' : {e}'
                )


if __name__ == '__main__':
    MF = ManageFiles(None)
    MF.get_config_file()
    MF.install_files()
    MF.remove_files()
