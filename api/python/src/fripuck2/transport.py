"""Inbound side of the robot link.

``TransportMixin`` owns the connection handshake and turns the raw bytes that
arrive from the robot into sensor samples. Everything here runs on the
``ConnectionManager`` background threads, never on the caller's thread.
"""

from ._generated import commands_generated as commands
from ._generated import sensors_generated as sensors
from .connection_manager import ConnectionManager


class TransportMixin:
    """See module docstring."""

    def connect(self, manager: ConnectionManager):
        """Starts the connection to the robot via a :class:`ConnectionManager`.

        This call returns immediately. The TCP link is established on a background
        thread, and the manager keeps retrying until it succeeds (and again if the
        robot is switched off and comes back later). Poll ``robot.connected`` to
        know when the robot is actually reachable:

        - ``False`` until the first successful connection,
        - ``True`` while the link is up,
        - ``False`` again if the link drops, back to ``True`` once it reconnects.

        One manager can serve several robots; robot packets are routed back to
        the right ``Robot`` instance by IP address.

        Once connected, sensor data is pushed into the history buffers in the
        background. Read the latest value with ``robot.tof.get()``, or the whole
        window with ``values, timestamps = robot.tof.get_all()``.

        :param manager: The :class:`ConnectionManager` to register with. Keep a
            reference to it for the lifetime of the program and call
            ``manager.shutdown()`` when you are done.

        Example::

            cm = ConnectionManager()
            robot = Robot("192.168.2.214")
            robot.connect(cm)
            ...
            cm.shutdown()
        """
        # Register and capture the send function natively
        self._send_tcp = manager.register_ip(
            self.ip, self._receive_packet, on_connect=self._set_connected
        )

    def _set_connected(self, connected: bool):
        """Called by the connection manager when the TCP link goes up or down."""
        self.connected = connected

    def _receive_packet(self, data: bytes):
        """Default callback function to recieve UDP/TCP packets."""
        if sensors.SensorBatch.SensorBatchBufferHasIdentifier(data, 8): # starts with crc and size, ignore for now
            # TODO: actually analyze the crc and size
            self._parse_sensor_batch(data)
        elif commands.CommandBatch.CommandBatchBufferHasIdentifier(data, 0):
            self._parse_command_batch(data)
        else:
            print("Couldn't parse packet.")

    def _parse_sensor_batch(self, data: bytes):
        batch = sensors.SensorBatch.GetRootAsSensorBatch(data, 8)
        base_timestamp = batch.BaseTimestamp()
        self.tof._parse(base_timestamp, batch)
        self.prox._parse(base_timestamp, batch)
        self.imu._parse(base_timestamp, batch)
        self.ground._parse(base_timestamp, batch)
        self.audio._parse(base_timestamp, batch)
        pass 

    def _parse_command_batch(self, data: bytes):
        pass
