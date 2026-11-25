"""Movement commands.

``MotionMixin`` wraps the firmware's single trajectory primitive. All the
public helpers (``move``, ``turn``, ``arc``, ``set_wheels``) reduce to a
``(speed, distance, radius)`` triple sent by :meth:`_send_trajectory`.
"""


class MotionMixin:
    """See module docstring."""

    def movement_settings(self, accel: float, decel: float):
        """Sets the default acceleration and deceleration for all later movements.

        These defaults are stored on the Python side and used by ``drive``,
        ``move``, ``turn``, ``arc`` and ``set_wheels`` whenever their ``accel`` /
        ``decel`` argument is ``None``. They persist for the lifetime of the
        ``Robot`` object; pass explicit values to a movement call to override them
        for that movement only.

        :param accel: Default linear acceleration, in m/s^2.
        :param decel: Default linear deceleration, in m/s^2.

        Example::

            robot.movement_settings(0.3, 0.5)
            robot.move(0.5, 0.2)             # uses 0.3 / 0.5
            robot.move(0.5, 0.2, accel=1.0)  # overrides accel only
        """
        self.default_accel = float(accel)
        self.default_decel = float(decel)

    def _send_trajectory(self, speed, distance, radius, accel, decel, synchronous, notify):
        """Internal helper to construct and send trajectory intents."""
        self._queue_or_send({
            "type": "MoveTrajectory",
            "speed": float(speed),
            "distance": float(distance),
            "radius": float(radius),
            "accel": float(accel if accel is not None else self.default_accel),
            "decel": float(decel if decel is not None else self.default_decel),
            "synchronous": bool(synchronous),
            "notify": bool(notify)
        })

    def drive(self, speed: float, distance: float = float('inf'), radius: float = float('inf'), 
              accel: float = None, decel: float = None, synchronous: bool = False, notify: bool = False):
        """Starts moving at a given speed, with no defined end by default.

        This is the most general movement command: it is what ``move``, ``turn``,
        ``arc`` and ``set_wheels`` ultimately call. By default it drives straight
        ahead forever, until another command replaces it; give ``distance`` and/or
        ``radius`` to constrain it.

        :param speed: Signed cruising speed in m/s (negative drives backwards).
        :param distance: Distance to travel before stopping, in meters. Defaults to
            ``inf``, i.e. keep going until told otherwise.
        :param radius: Turn radius in meters. ``inf`` (default) drives straight,
            a positive value curves left, a negative value curves right, and ``0``
            spins in place.
        :param accel: Acceleration in m/s^2. ``None`` uses ``default_accel``.
        :param decel: Deceleration in m/s^2. ``None`` uses ``default_decel``.
        :param synchronous: If ``True``, the robot waits for any current motion to
            finish before starting this one, instead of replacing it immediately.
        :param notify: If ``True``, the robot reports when this movement completes.
            Note that notification handling on the Python side is still a stub, so
            this is currently only useful in firmware scripts.

        Example::

            robot.drive(0.2)                 # forward forever at 20 cm/s
            robot.drive(-0.2)                # backwards forever
            robot.drive(0.2, distance=1.0)   # forward one metre, then stop
        """
        self._send_trajectory(speed, distance, radius, accel, decel, synchronous, notify)

    def move(self, distance: float, speed: float, accel: float = None, decel: float = None, 
             synchronous: bool = False, notify: bool = False):
        """Moves in a straight line for a given distance, then stops.

        :param distance: Distance to travel, in meters. Negative values move
            backwards.
        :param speed: Signed cruising speed in m/s. Keep it small enough for the
            robot to brake over ``distance``.
        :param accel: Acceleration in m/s^2. ``None`` uses ``default_accel``.
        :param decel: Deceleration in m/s^2. ``None`` uses ``default_decel``.
        :param synchronous: If ``True``, wait for the current motion to finish
            before starting this one instead of replacing it.
        :param notify: If ``True``, the robot reports when the movement completes.

        Example::

            robot.move(0.5, 0.2)    # half a metre forward at 20 cm/s
            robot.move(-0.2, 0.1)   # 20 cm backwards
        """
        self._send_trajectory(speed, distance, float('inf'), accel, decel, synchronous, notify)

    def turn(self, radians: float, speed: float, accel: float = None, decel: float = None, 
             synchronous: bool = False, notify: bool = False):
        """Turns in place by a given angle, then stops.

        The angle is converted to a wheel travel distance using the wheel base
        (``WHEEL_DISTANCE_M``) and sent as an in-place rotation. Positive angles
        turn left (counter-clockwise), negative angles turn right.

        :param radians: Angle to turn, in radians. Positive = left, negative = right.
        :param speed: Linear speed of the wheels in m/s. Note this is not an
            angular speed; smaller values mean a slower, more precise turn.
        :param accel: Acceleration in m/s^2. ``None`` uses ``default_accel``.
        :param decel: Deceleration in m/s^2. ``None`` uses ``default_decel``.
        :param synchronous: If ``True``, wait for current motion to finish first.
        :param notify: If ``True``, the robot reports when the turn completes.

        Example::

            robot.turn(math.pi / 2, 0.1)    # quarter turn left
            robot.turn(-math.pi, 0.1)       # half turn right (about face)
        """
        dist = abs(radians) * (self.WHEEL_DISTANCE_M / 2.0)
        rad = 0.0 if radians > 0.0 else -0.00001
        self._send_trajectory(speed, dist, rad, accel, decel, synchronous, notify)

    def arc(self, distance: float, radius: float, speed: float, accel: float = None, decel: float = None, 
            synchronous: bool = False, notify: bool = False):
        """Moves along a circular arc and stops after a given path length.

        ``distance`` is the length travelled *along the arc*, not the straight-line
        distance to the endpoint. The robot's heading changes by
        ``distance / radius`` radians over the course of the movement.

        :param distance: Arc length to travel, in meters. Negative moves backwards
            along the arc.
        :param radius: Turn radius in meters. Positive curves left, negative
            curves right. ``0`` is not meaningful here; use :meth:`turn` to spin
            in place.
        :param speed: Signed cruising speed in m/s.
        :param accel: Acceleration in m/s^2. ``None`` uses ``default_accel``.
        :param decel: Deceleration in m/s^2. ``None`` uses ``default_decel``.
        :param synchronous: If ``True``, wait for current motion to finish first.
        :param notify: If ``True``, the robot reports when the movement completes.

        Example::

            # Quarter circle of radius 0.3 m: travels ~0.47 m, turns 90 deg left.
            robot.arc(0.3 * math.pi / 2, 0.3, 0.15)
        """
        self._send_trajectory(speed, distance, radius, accel, decel, synchronous, notify)

    def set_wheels(self, left: float, right: float, distance: float = float('inf'), 
                   accel: float = None, decel: float = None, synchronous: bool = False, notify: bool = False):
        """Drives each wheel at its own speed, the closest thing to raw motor control.

        The two wheel speeds are converted into the (speed, radius) pair that the
        firmware's trajectory generator understands:

        - equal speeds drive straight,
        - opposite speeds spin in place,
        - anything else drives along an arc.

        Because of that conversion the ``distance`` applies to the path actually
        travelled, and the commanded speed is approximate when turning.

        :param left: Target linear velocity of the left wheel in m/s. Negative
            drives that wheel backwards.
        :param right: Target linear velocity of the right wheel in m/s. Negative
            drives that wheel backwards.
        :param distance: Distance to travel before stopping, in meters. Defaults to
            ``inf``, i.e. keep going until told otherwise.
        :param accel: Acceleration in m/s^2. ``None`` uses ``default_accel``.
        :param decel: Deceleration in m/s^2. ``None`` uses ``default_decel``.
        :param synchronous: If ``True``, wait for current motion to finish first.
        :param notify: If ``True``, the robot reports when the movement completes.

        Example::

            robot.set_wheels(0.1, 0.1)    # straight at 10 cm/s
            robot.set_wheels(0.1, -0.1)   # spin in place, left wheel forwards
            robot.set_wheels(0.15, 0.05)  # gentle left curve
        """
        speed = (left + right) / 2.0
        omega = (right - left) / self.WHEEL_DISTANCE_M

        if abs(omega) < 0.0001:
            rad = float('inf')  # Moving straight
        elif abs(speed) < 0.0001:
            rad = 0.0 if omega > 0.0 else -0.00001  # In-place spin
            speed = abs(left)
        else:
            rad = speed / omega  # Curved arc

        self._send_trajectory(speed, distance, rad, accel, decel, synchronous, notify)
