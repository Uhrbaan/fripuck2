from ._generated import sensors_generated as sensors
from ._generated import commands_generated as commands
from .sensors.sensors import TofHisory, ProxHisory, ImuHistory, GroundHistory

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