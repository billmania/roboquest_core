"""Dynamically import and instantiate any available I2C user modules."""
from pathlib import Path
from sys import path


class I2CSupport(object):
    """Manage the run-time I2C user modules."""

    def __init__(self):
        """Initialize the object."""
        pass

    def import_modules(self, module_dir: str) -> dict:
        """Import I2C modules.

        If the persistent directory exists and contains one or more
        modules, import each one at a time and instantiate an object.

        A dictionary is returned, containing a member for each
        object instantiated. The key is the name of the class.
        """
        if not Path(module_dir).exists():
            return {}

        # TODO: May require an __init__.py in MODULE_DIR
        path.append(module_dir)

        i2c_objects = {}
        for directory_entry in Path(module_dir).glob('*.py'):
            if directory_entry.is_file():
                module_name = directory_entry.name[:-3]
                class_name = module_name

                imported_module = __import__(module_name)
                i2c_objects[module_name] = getattr(
                    imported_module,
                    class_name
                )(class_name)

        return i2c_objects
