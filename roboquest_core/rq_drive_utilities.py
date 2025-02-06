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
        sprocket_radius: float,
        track_separation: float
    ):
        """Initialize the constants.

        sprocket_radius is the radius of the drive sprocket in meters,
                        including the thickness of the track
        track_separation is the distance between the centers of the two tracks,
           in meters
        """

        # scm is the sprocket circumference
        scm = 2 * π * sprocket_radius
        # AR is the angular motion radius
        AR = track_separation / 2

        self._linear_conversion = (
            SECS_PER_MIN /
            scm
        )

        self._angular_conversion = (
            SECS_PER_MIN *
            AR /
            scm
        )

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
            angular_velocity * self._angular_conversion
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
            linear_velocity * self._linear_conversion
        )
