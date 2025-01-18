"""Utility functions for motion control.

Convert a linear or angular velocity to drive sprocket RPMs.
See https://github.com/billmania/roboquest_core/wiki/Robot-motion-calculations
"""

from math import pi as π

SECS_PER_MIN = 60


class DriveUtils(object):
    """Hold utility functions for RPM."""

    def __init__(
        self,
        sr: float,
        tl: float,
        ts: float
    ):
        """Initialize the constants.

        sr is the radius of the drive sprocket in meters
        tl is the length of the drive track in meters
        ts is the distance between the centers of the two tracks,
           in meters
        """
        self._SR = sr
        self._TL = tl
        self._TS = ts

        self._scm = 2 * π * self._SR
        self._AR = self._TS / 2

    def angular_to_rpm(
        self,
        angular_velocity: float = 0.0
    ) -> int:
        """Convert angular velocity to RPM.

        The angular_velocity input is interpreted as
        radians per second AND a positive value is interpreted
        as turning the bow to the left.

        The return value is RPMs for the left sprocket. Apply the
        negative of that value to the right sprocket.
        """
        return round(
            angular_velocity *
            SECS_PER_MIN *
            self._AR /
            self._TL /
            self._scm
        )

    def linear_to_rpm(
        self,
        linear_velocity: float = 0.0
    ) -> int:
        """Convert linear velocity to RPM.

        The linear velocity is interpreted as meters per second. The
        return value is RPMs for both sprockets.
        """
        return round(
            linear_velocity *
            SECS_PER_MIN /
            self._TL /
            self._scm
        )
