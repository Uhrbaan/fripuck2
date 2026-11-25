"""Outbound side of the robot link.

``ProtocolMixin`` turns the small ``intent`` dicts produced by the command
methods into FlatBuffers packets, and decides whether a command goes out
immediately or waits for the enclosing :meth:`~fripuck2.robot.Robot.batch`.
"""

from contextlib import contextmanager
import time

import flatbuffers

from ._generated import commands_generated as commands


class ProtocolMixin:
    """See module docstring."""

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

        elif intent["type"] == "SetLeds":
            cmd.commandType = commands.Instruction.SetLeds
            entries = []
            for item in intent["entries"]:
                r, g, b = item["color"]
                entries.append(
                    commands.LedEntryT(
                        target=item["target"],
                        on=item["on"],
                        color=commands.RgbColorT(r=r, g=g, b=b),
                        intensity=item["intensity"],
                    )
                )
            cmd.command = commands.SetLedsT(
                entries=entries,
                fadeMs=intent["fade_ms"],
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
