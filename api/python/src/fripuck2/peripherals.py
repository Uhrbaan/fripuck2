"""Everything that is not motion: hardware modules, files, sound, custom topics.

``PeripheralMixin`` groups the smaller command families so that each stays in
one obvious place, without each getting a two-function module of its own.
"""

import os

from .commands import ModuleMask, PlayMode


class PeripheralMixin:
    """See module docstring."""

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
