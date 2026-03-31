from ._generated import sensors_generated as sensors
from ._generated import commands_generated as commands
import numpy
import time

class TofHisory:
    def __init__(self, size=100):
        self._size = size 
        # columns: [timestamp, distance]
        self._data = numpy.zeros((size, 2))
        self._ptr = 0
        self._count = 0

    def _add(self, timestamp, distance):
        self._data[self._ptr] = [timestamp, distance]
        self._ptr = (self._ptr+1) % self._size
        if self._count < self._size:
            self._count += 1

    def _parse(self, base_timestamp, batch: sensors.SensorBatch):
        if not batch.TofIsNone(): 
            for i in range(batch.TofLength()):
                item = batch.Tof(i)
                self._add(base_timestamp + item.TimestampOffset(), item.Distance())

    def get(self):
        """Get the latest Time of flight data point.

        :return: Latest time of flight reading.
        """
        return self._data[self._ptr-1][1]
    
    def get_all(self): 
        """Get all the last elements stored and clear the queue.
        
        :return: a numpy array of size (0, 2) containing the latest sensor readings. Note that it is not guranteed that the elements are in order !
        """
        if self._count < self._size:
            result = self._data[:self._ptr] # Copy so 'clear' doesn't affect it
        else:
            result = numpy.roll(self._data, -self._ptr, axis=0)

        self._ptr = 0
        self._count = 0

        return result
    
    def __len__(self):
        return self._count

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