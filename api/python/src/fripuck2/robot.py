from ._generated import sensors_generated as sensors
from ._generated import commands_generated as commands
from .sensors.sensors import TofHisory, ProxHisory, ImuHistory, GroundHistory
from .connection_manager import ConnectionManager
import flatbuffers
import time

class Robot:
    def __init__(self, ip: str, history_size=100): 
        """Initializes the Robot class. 
        
        :param ip_addr: IP address of the e-puck2 robot.
        :param tcp_port: TCP port the robot will try to connect to. Don't touch this option if you are using the standard firmware. 
        :param udp_port: UDP port you will recieve data from. Don't touch this option if you are using the standard firmware.
        :param history_size: Number of elements stored in history. You can get them with .get_all()
        """
        
        self.ip = ip 
        self.tof = TofHisory(size=history_size)
        self.prox = ProxHisory(size=history_size)
        self.imu = ImuHistory(size=history_size)
        self.ground = GroundHistory(size=history_size)
        self.builder = flatbuffers.Builder(1024)

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
        pass 

    def _parse_command_batch(self, data: bytes):
        pass

    def send_dummy(self, topic: str, payload: str | None):
        """Sends a custom Custom FB packet
        """
        self.builder.Clear()

        batch = commands.CommandBatchT(
            timestamp=0,
            immediateCommands=[
                commands.CommandT(
                    commandType=commands.Instruction.Custom,
                    command=commands.CustomT(topic, payload)
                )
            ],
            sequences=None,
            abortAll=False,
        ).Pack(self.builder)

        self.builder.FinishSizePrefixed(batch, file_identifier=b"CMND")

        self._send_tcp(self.builder.Output())
        