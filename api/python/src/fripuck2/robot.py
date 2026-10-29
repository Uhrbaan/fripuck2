import json

from ._generated import sensors_generated as sensors
from ._generated import commands_generated as commands
from .sensors import TofHisory, ProxHisory, ImuHistory, GroundHistory, AudioHistory
from .commands import ModuleMask
from .connection_manager import ConnectionManager
from contextlib import contextmanager
import flatbuffers
import time
import struct
from pprint import pprint

class Robot:
    WHEEL_DIAMETER_M = 0.041
    WHEEL_DISTANCE_M = 0.053
    
    def __init__(self, ip: str, history_size=100): 
        """Initializes the Robot class. 
        
        :param ip_addr: IP address of the e-puck2 robot.
        :param tcp_port: TCP port the robot will try to connect to. Don't touch this option if you are using the standard firmware. 
        :param udp_port: UDP port you will recieve data from. Don't touch this option if you are using the standard firmware.
        :param history_size: Number of elements stored in history. You can get them with .get_all()
        """
        
        self.default_accel = 0.5
        self.default_decel = 0.5
        self._command_queue = None
        
        self.ip = ip 
        self.tof = TofHisory(size=history_size)
        self.prox = ProxHisory(size=history_size)
        self.imu = ImuHistory(size=history_size)
        self.ground = GroundHistory(size=history_size)
        self.audio = AudioHistory(size=history_size)

    def connect(self, manager: ConnectionManager):
        # Register and capture the send function natively
        self._send_tcp = manager.register_ip(self.ip, self._receive_packet)

    def _receive_packet(self, data: bytes):
        """Default callback function to recieve UDP/TCP packets."""
        if sensors.SensorBatch.SensorBatchBufferHasIdentifier(data, 0):
            self._parse_sensor_batch(data)
        elif commands.CommandBatch.CommandBatchBufferHasIdentifier(data, 0):
            self._parse_command_batch(data)
        else:
            print("Couldn't parse packet.")

    def _parse_sensor_batch(self, data: bytes):
        batch = sensors.SensorBatch.GetRootAsSensorBatch(data, 0)
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
        """Context manager to group multiple commands into a single network packet."""
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
        self._send_tcp(data)
                
    def _queue_or_send(self, intent):
        """Helper to either queue the intent or send immediately."""
        if self._command_queue is not None:
            self._command_queue.append(intent)
        else:
            self._build_and_send([intent])
    
    def custom(self, topic: str, payload: str = ""):
        """Sends a custom MQTT-style topic and payload."""
        self._queue_or_send({
            "type": "Custom",
            "topic": topic,
            "payload": payload
        })
        
    def movement_settings(self, accel: float, decel: float):
        """Configures the global default acceleration and deceleration."""
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
        """Starts continuous movement at a specific speed."""
        self._send_trajectory(speed, distance, radius, accel, decel, synchronous, notify)
        
    def move(self, distance: float, speed: float, accel: float = None, decel: float = None, 
             synchronous: bool = False, notify: bool = False):
        """Moves the robot in a straight line for a specified distance."""
        self._send_trajectory(speed, distance, float('inf'), accel, decel, synchronous, notify)
        
    def turn(self, radians: float, speed: float, accel: float = None, decel: float = None, 
             synchronous: bool = False, notify: bool = False):
        """Turns the robot in-place by a specific angle."""
        dist = abs(radians) * (self.WHEEL_DISTANCE_M / 2.0)
        rad = 0.0 if radians > 0.0 else -0.00001
        self._send_trajectory(speed, dist, rad, accel, decel, synchronous, notify)
        
    def arc(self, distance: float, radius: float, speed: float, accel: float = None, decel: float = None, 
            synchronous: bool = False, notify: bool = False):
        """Moves the robot along an arc."""
        self._send_trajectory(speed, distance, radius, accel, decel, synchronous, notify)
        
    def set_wheels(self, left: float, right: float, distance: float = float('inf'), 
                   accel: float = None, decel: float = None, synchronous: bool = False, notify: bool = False):
        """Drives the robot by setting individual wheel velocities directly."""
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
        """
        Example: robot.set_modules(body_led=True, tof=True, motors=False)
        """
        enable_mask = 0  # Assuming 0 is None
        disable_mask = 0
        
        # Iterate through kwargs, convert to uppercase to match Enum, and apply bitwise OR
        for key, value in kwargs.items():
            try:
                module_bit = ModuleMask[key.upper()]
                if value:
                    enable_mask |= module_bit
                else:
                    disable_mask |= module_bit
            except KeyError:
                print(f"Warning: Unknown module '{key}'")

        command_intent = {
            "type": "EnableCommand",
            "enable_mask": enable_mask,
            "disable_mask": disable_mask
        }

        self._queue_or_send(command_intent)