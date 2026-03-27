from ._generated import sensors_generated as sensors
from ._generated import commands_generated as commands
import numpy

class TofHisory:
    def __init__(self, size=100):
        self.size = size 
        # columns: [timestamp, distance]
        self.data = numpy.zeros((size, 2))
        self.ptr = 0

    def _add(self, timestamp, distance):
        self.data[self.ptr] = [timestamp, distance]
        self.ptr = (self.ptr+1) % self.size

    def _parse(self, base_timestamp, batch: sensors.SensorBatch):
        if not batch.TofIsNone(): 
            for i in range(batch.TofLength()):
                item = batch.Tof(i)
                self._add(base_timestamp + item.TimestampOffset(), item.Distance())

    def get(self):
        """Get the latest Time of flight data point.

        :return: Latest time of flight reading.
        """
        return self.data[self.ptr-1][1]
    
    def get_all(self): 
        """Get """
        return numpy.roll(self.data, -self.ptr, axis=0)


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
        base_timestamp
        self.tof._parse(base_timestamp, batch)
        # (base_timestamp, batch)
        # (base_timestamp, batch)
        # (base_timestamp, batch)
        # (base_timestamp, batch)
        # (base_timestamp, batch)
        # (base_timestamp, batch)
        # (base_timestamp, batch)
        # (base_timestamp, batch)
        pass 

    def _parse_command_batch(self, data: bytes):
        pass