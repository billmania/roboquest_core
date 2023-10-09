"""
Manage persistent configuration files.
"""
from typing import Callable
import pathlib
from json import dumps, loads


class ConfigFileError(Exception):
    pass


class ConfigFile(object):
    """
    Methods to initialize configuration files, retrieve them from persistent
    storage, and update what's in persistent storage.
    """

    def __init__(self, persist_dir: str):
        """
        Ensure the persistent directory exists and is writable.
        """

        if not persist_dir:
            raise ConfigFileError(
                'Must provide a persistent directory path as a string')

        self._persist_dir_path = pathlib.Path(persist_dir)
        if (not self._persist_dir_path.exists()
                or not self._persist_dir_path.is_dir()):
            raise ConfigFileError(
                f"{persist_dir} does not exist or is not a directory")

    def init_config(
         self,
         config_file_name: str,
         get_default_data: Callable) -> None:
        """
        If config_file_name already exists in the persist_dir, return
        without changing anything. If it doesn't exist, interpret
        config_data as the object to save in the configuration file.
        Create a file named config_file_name in the persist_dir, convert
        config_data to a JSON string, and write it to the file.
        """

        config_file_path = self._persist_dir_path / config_file_name
        if config_file_path.exists() and config_file_path.is_file():
            return

        config_file_path.unlink(missing_ok=True)
        config_file_path.write_text(
            dumps(
                get_default_data(),
                sort_keys=True,
                indent=2)
        )

    def get_config(self, config_file_name: str) -> object:
        """
        Get the contents of config_file_name and return it as an
        object. The config file is expected to exist in the persistent
        directory.
        """

        config_file_path = self._persist_dir_path / config_file_name
        if config_file_path.exists() and config_file_path.is_file():
            try:
                return loads(config_file_path.read_text())
            except Exception as e:
                raise ConfigFileError(e)
        else:
            raise ConfigFileError(f"{config_file_name} does not exist")

    def save_config(self, config_file_name: str, config_data: object) -> None:
        """
        Archive the current config_file_name and save the config_data
        in its place.
        """

        self._archive_config(config_file_name)

        config_file_path = self._persist_dir_path / config_file_name
        try:
            config_file_path.write_text(
                dumps(
                    config_data,
                    sort_keys=True,
                    indent=2
                )
            )
        except Exception as e:
            raise ConfigFileError(e)

    def _archive_config(self, config_file_name: str):
        """
        Remove any old config_file_name and rename the current config_file_name
        by appending ".old".
        """

        old_config_file_path = (
            self._persist_dir_path / f"{config_file_name}.old"
        )
        old_config_file_path.unlink(missing_ok=True)
        config_file_path = self._persist_dir_path / config_file_name
        config_file_path.rename(old_config_file_path)


if __name__ == '__main__':
    cf = ConfigFile('/tmp')
    cf.init_config(
        'servos.config',
        [{'name': 'camera_tilt', 'channel': 0},
         {'name': 'camera_pan', 'channel': 1}]
    )
    print(cf.get_config('servos.config'))
    cf.save_config(
        'servos.config',
        [{'name': 'camera_tilt', 'channel': 10},
         {'name': 'camera_pan', 'channel': 11}]
    )
    print(cf.get_config('servos.config'))
