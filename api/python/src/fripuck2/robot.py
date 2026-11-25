import json
import os

from ._generated import sensors_generated as sensors
from ._generated import commands_generated as commands
from .sensors import TofHisory, ProxHisory, ImuHistory, GroundHistory, AudioHistory
from .commands import ModuleMask, PlayMode
from .connection_manager import ConnectionManager
from contextlib import contextmanager
import flatbuffers
import time
import struct
from pprint import pprint

class Robot:
    """Control handle for one e-puck2 robot running the fripuck2 firmware.

    The class has two independent halves:

    **Commands (you -> robot).** ``drive``, ``move``, ``turn``, ``arc`` and
    ``set_wheels`` plan a motion; ``set_modules`` powers hardware on or off;
    ``play`` makes sound; ``upload_file`` pushes files to the robot's
    filesystem. All of these are fire-and-forget unless you wrap them in
    :meth:`batch`, and they do nothing until :meth:`connect` has been called and
    ``connected`` is ``True``.

    **Sensors (robot -> you).** Each sensor fills a rolling buffer that you read
    with ``get()`` (latest value) or ``get_all()`` (the whole window, as
    ``values, timestamps``). Data arrives in the background, so reads are
    instantaneous and never block on the network::

        distance = robot.tof.get()
        values, timestamps = robot.tof.get_all()

    Units follow SI conventions unless stated otherwise: metres, metres per
    second, metres per second squared and radians for motion; millimetres for
    ToF distance; seconds for timestamps.

    Example::

        from fripuck2 import ConnectionManager, Robot

        cm = ConnectionManager()
        robot = Robot("192.168.2.214")
        robot.connect(cm)

        robot.set_modules(tof=True, motors=True)
        robot.move(0.5, 0.2)

        while robot.connected:
            print(robot.tof.get())

        cm.shutdown()
    """
    WHEEL_DIAMETER_M = 0.041
    WHEEL_DISTANCE_M = 0.053
    
    def __init__(self, ip: str, history_size=100): 
        """Creates a Robot handle for one e-puck2 running the fripuck2 firmware.

        Nothing is sent to the robot here: call :meth:`connect` to start the
        connection. The sensor buffers are created immediately, so you can read
        them before a connection exists (they simply stay empty).

        :param ip: IPv4 address of the robot, e.g. ``"192.168.2.214"``. It is
            also the address the robot sends its UDP sensor packets from, so it
            is used to tell robots apart when several are registered.
        :param history_size: Number of samples kept per sensor buffer. This is
            the maximum depth of the rolling window served by
            ``robot.<sensor>.get_all()``. Older samples are dropped, so pick a
            size that covers the longest window you want to inspect (e.g. `1000`
            at ~50 Hz keeps roughly the last 20 seconds).

        The following attributes are available after construction:

        ``ip``
            The robot's address, as passed in.
        ``connected``
            ``True`` while the TCP link to the robot is up, ``False`` before the
            connection is established and whenever it is lost. Poll this to know
            whether the robot is currently reachable: it is updated in the
            background and by the automatic reconnect logic.
        ``tof``, ``prox``, ``imu``, ``ground``, ``audio``
            Per-sensor history buffers. Read the latest sample with ``.get()`` or
            the whole window with ``values, timestamps = .get_all()``.
        ``default_accel``, ``default_decel``
            Default acceleration / deceleration in m/s^2, used by the movement
            methods when their ``accel`` / ``decel`` argument is left as ``None``.
            Change them in one go with :meth:`movement_settings`.

        Example::

            robot = Robot("192.168.2.214", history_size=1000)
            robot.connect(ConnectionManager())
            while not robot.connected:
                time.sleep(0.1)
        """        
        self.default_accel = 0.5
        self.default_decel = 0.5
        self._command_queue = None

        # True while the TCP connection to the robot is up. Set to False until
        # the connection is established and whenever it is lost.
        self.connected = False
        
        self.ip = ip 
        self.tof = TofHisory(size=history_size)
        self.prox = ProxHisory(size=history_size)
        self.imu = ImuHistory(size=history_size)
        self.ground = GroundHistory(size=history_size)
        self.audio = AudioHistory(size=history_size)

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

    @contextmanager
    def batch(self):
        """Context manager grouping every command sent inside the block into one packet.

        Commands issued inside the ``with`` block (``drive``, ``move``,
        ``set_modules``, ``play``, ...) are not sent immediately: they are queued
        and flushed together as a single TCP packet when the block exits. This
        avoids network round-trips and, more importantly, makes several commands
        take effect on the robot at the same instant instead of a few milliseconds
        apart.

        The batch is sent even if the block raises (the flush happens in a
        ``finally``), but the exception still propagates.

        Example::

            with robot.batch():
                robot.set_modules(tof=True)
                robot.set_wheels(0.1, 0.1)
                robot.play("I2 V7 la16")
        """
        self._command_queue = [] # Start queuing
        try:
            yield # Let the student's code inside the 'with' block run
        finally:
            # The block exited. Send the batch and disable queuing.
            commands_to_send = self._command_queue
            self._command_queue = None
            if commands_to_send:
                self._build_and_send(commands_to_send)
                
    def _intent2command(self, intent):
        """Converts an intent dict directly into a wrapped CommandT object."""
        cmd = commands.CommandT()
        
        if intent["type"] == "Custom":
            cmd.commandType = commands.Instruction.Custom
            cmd.command = commands.CustomT(
                topic=intent["topic"],
                payload=intent.get("payload", "")
            )
            
        elif intent["type"] == "EnableCommand":
            cmd.commandType = commands.Instruction.EnableCommand
            cmd.command = commands.EnableCommandT(
                enableMask=intent["enable_mask"],
                disableMask=intent["disable_mask"]
            )

        elif intent["type"] == "MoveTrajectory":
            cmd.commandType = commands.Instruction.MoveTrajectory
            cmd.command = commands.MoveTrajectoryT(
                speed=intent["speed"],
                distance=intent["distance"],
                radius=intent["radius"],
                accel=intent["accel"],
                decel=intent["decel"],
                notify=intent["notify"],
                synchronous=intent["synchronous"]
            )
        elif intent["type"] == "FileTransfer":
            cmd.commandType = commands.Instruction.FileTransfer
            cmd.command = commands.FileTransferT(
                filename=intent["filename"],
                content=intent["content"],
                selectorSlot=intent["selector_slot"]
            )
        elif intent["type"] == "PlaySound": 
            cmd.commandType = commands.Instruction.PlaySound
            cmd.command = commands.PlaySoundT(
                mml=intent["mml"],
                mode=intent["mode"] 
            )
            
        return cmd

    def _build_and_send(self, intents):
        """Compiles the FlatBuffer and pushes it over the network."""
        builder = flatbuffers.Builder(1024)
        
        batch_obj = commands.CommandBatchT(
            timestamp=int(time.time() * 1000),
            commands=[self._intent2command(intent) for intent in intents]
        )
                
        batch_offset = batch_obj.Pack(builder)
        builder.FinishSizePrefixed(batch_offset)
        
        data = bytes(builder.Output())
        # size_prefix, = struct.unpack_from('<I', data, 0)
        # root_off,    = struct.unpack_from('<I', data, 4)
        # print(f"total={len(data)} prefix={size_prefix} (expect {len(data)-4}) root_off={root_off}")
        # print(data[:16].hex(' '))   # bytes 8..11 should be 43 4d 4e 44 ("CMND")
        print(f"Sending {len(data)} ove TCP as FB.")
        self._send_tcp(data)
                
    def _queue_or_send(self, intent):
        """Helper to either queue the intent or send immediately."""
        if self._command_queue is not None:
            self._command_queue.append(intent)
        else:
            self._build_and_send([intent])
    
    def custom(self, topic: str, payload: str = ""):
        """Sends an arbitrary topic/payload pair to the robot.

        This is the escape hatch for firmware features that the Python API does
        not wrap yet. The pair is forwarded to the robot's Lua VM, where it can be
        picked up by a script listening on a matching ``robot.on(topic, ...)``
        hook.

        :param topic: MQTT-style topic string, e.g. ``"my:module:do_thing"``.
        :param payload: Optional string payload. The firmware interprets it; the
            Python side does no parsing or escaping.

        Example::

            robot.custom("cmd:beep", "3")
        """
        self._queue_or_send({
            "type": "Custom",
            "topic": topic,
            "payload": payload
        })
        
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
        
    #TODO: make it so notify is by default true when in with batch() block
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

    def set_modules(self, **kwargs):
        """Enables or disables hardware modules on the robot.

        Pass module names as keyword arguments with a boolean value: ``True``
        turns the module on, ``False`` turns it off. Anything not mentioned is
        left in its current state. Names are matched case- and
        underscore-insensitively, so ``tof``, ``TOF``, ``body_led`` and
        ``bodyled`` all work. Unknown names print a warning and are ignored.

        This is the main power-saving knob: sensor modules that are not enabled do
        not produce data, so no samples arrive in the corresponding buffer.

        Available modules: ``mode_selector``, ``ir_receiver``, ``battery``,
        ``proximity``, ``led_1``, ``led_3``, ``led_5``, ``led_7``, ``body_led``,
        ``front_led``, ``tof``, ``imu``, ``camera``, ``motors``, ``ground``.

        .. note::
            The firmware's Lua API calls the ring LEDs ``ring_led_1`` ...
            ``ring_led_7``, but the generated ``ModuleMask`` names them
            ``Led1`` ... ``Led7``, so this method currently accepts ``led_1``
            (and ``led1``), **not** ``ring_led_1``.

        Example::

            robot.set_modules(body_led=True, tof=True, motors=False)
            robot.set_modules(proximity=False)   # leaves tof as it was
        """
        enable_mask = 0  # Assuming 0 is None
        disable_mask = 0
        
        # Build a lowercase -> bit lookup once, e.g. "tof" -> ModuleMask.Tof.
        # This keeps the public API case/naming-agnostic (tof, TOF, body_led, bodyled...).
        modules = {
            name.lower().replace("_", ""): value
            for name, value in vars(ModuleMask).items()
            if not name.startswith("_") and name != "None_"
        }

        for key, value in kwargs.items():
            module_bit = modules.get(key.lower().replace("_", ""))
            if module_bit is None:
                print(f"Warning: Unknown module '{key}'")
                continue
            if value:
                enable_mask |= module_bit
            else:
                disable_mask |= module_bit

        command_intent = {
            "type": "EnableCommand",
            "enable_mask": enable_mask,
            "disable_mask": disable_mask
        }

        self._queue_or_send(command_intent)
        
    def upload_file(self, local_path: str, remote_filename: str = None, selector_slot: int = -1):
        """
        Reads a local file and queues it to be sent to the robot's filesystem.

        The file is read entirely into memory on your computer and shipped in a
        single command, so keep uploads small (a Lua init script, a model, ...).

        :param local_path: The path to the file on your computer.
        :param remote_filename: Name to use on the robot. Defaults to the local
            file's name.
        :param selector_slot: Where to store the file, as a selector slot number
            from ``0`` to ``15``. The robot keeps one ``/lfs/<slot>/`` directory
            per selector position and boots the ``init.lua`` found in the
            directory matching its current selector. Use ``-1`` (the default) to
            place the file in the shared root directory instead.

        If the file cannot be read, an error is printed and nothing is sent.

        Example::

            robot.upload_file("init.lua", "init.lua", 0)  # slot 0 boot script
            robot.upload_file("lib.lua")                 # shared, root directory
        """
        if remote_filename is None:
            remote_filename = os.path.basename(local_path)
            
        try:
            with open(local_path, "rb") as f:
                file_content = f.read()
        except FileNotFoundError:
            print(f"Error: Could not find file '{local_path}'")
            return

        self._queue_or_send({
            "type": "FileTransfer",
            "filename": remote_filename,
            # FlatBuffers Object API accepts a byte string / list of ints for [uint8]
            "content": file_content, 
            "selector_slot": selector_slot
        })
        
    def play(self, mml: str, mode: int = PlayMode.Replace):
        """Plays a melody described in Music Macro Language (MML).

        MML is a compact text notation for music. A few useful commands:
        ``T`` tempo, ``O`` octave, ``L`` default note length, ``V`` volume,
        ``Q`` note gate time and ``I`` instrument (``0`` is a sine wave; the
        firmware also has instruments that sound like a beep). Note names are
        ``a``-``g`` (lowercase, with ``+``/``#`` for sharps and ``-`` for flats),
        optionally followed by a duration such as ``4`` or ``4.``. ``>`` and ``<``
        shift the octave up and down.

        :param mml: The melody, e.g. ``"T120 O4 L16 Q5 I2 c d e"``.
        :param mode: How to interact with a melody that is already playing.
            ``PlayMode.Replace`` (default) stops it and starts this one,
            ``PlayMode.Queue`` appends this one after it, ``PlayMode.IfFree``
            only plays if nothing is playing.

        Example::

            robot.play("I2 V7 la16")                      # short beep
            robot.play("T150 O5 L8 e e r e", PlayMode.IfFree)
        """
        self._queue_or_send({
            "type": "PlaySound",
            "mml": mml,
            "mode": mode
        })