from .._generated import sensors_generated as sensors
from .._generated import commands_generated as commands
from typing import Tuple, List

from .base import SensorHistory

import numpy
import time

class TofHisory(SensorHistory[Tuple[float, float]]):
    def __init__(self, size=100):
        super().__init__(size=size)

    def _parse(self, base_timestamp, batch: sensors.SensorBatch):
        if not batch.TofIsNone(): 
            for i in range(batch.TofLength()):
                item = sensors.TofDataT.InitFromObj(batch.Tof(i))
                seconds = (base_timestamp + item.timestampOffset)/1000
                self._add((seconds, item.distance))

class ProxHisory(SensorHistory[Tuple[float, List[float]]]):
    def __init__(self, size=100):
        super().__init__(size=size)

    def _parse(self, base_timestamp, batch: sensors.SensorBatch):
        if not batch.ProximityIsNone(): 
            for i in range(batch.ProximityLength()):
                item = batch.Proximity(i)
                proximities = sensors.Uint16Array8()
                ambients = sensors.Uint16Array8()
                item.Proximity(proximities)
                item.AmbientLight(ambients)
                self._add(((base_timestamp + item.TimestampOffset())/1000, [ 
                    proximities.A0() - ambients.A0(), # Calculate the difference
                    proximities.A1() - ambients.A1(), 
                    proximities.A2() - ambients.A2(), 
                    proximities.A3() - ambients.A3(), 
                    proximities.A4() - ambients.A4(), 
                    proximities.A5() - ambients.A5(), 
                    proximities.A6() - ambients.A6(), 
                    proximities.A7() - ambients.A7()
                ]))

class ImuHistory(SensorHistory[Tuple[float, sensors.ImuDataT]]):
    def __init__(self, size=100):
        super().__init__(size=size)

    def _parse(self, base_timestamp, batch: sensors.SensorBatch):
        if not batch.ImuIsNone(): 
            for i in range(batch.ImuLength()):
                item = sensors.ImuDataT.InitFromObj(batch.Imu(i))
                self._add(((base_timestamp + item.timestampOffset)/1000, item))