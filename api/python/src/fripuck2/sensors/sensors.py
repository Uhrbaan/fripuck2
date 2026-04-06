from .._generated import sensors_generated as sensors
from .._generated import commands_generated as commands

from .base import SensorHistory

import numpy
import time

class TofHisory(SensorHistory):
    def __init__(self, size=100):
        super().__init__(size=size, dims=2)

    def _parse(self, base_timestamp, batch: sensors.SensorBatch):
        if not batch.TofIsNone(): 
            for i in range(batch.TofLength()):
                item = batch.Tof(i)
                self._add([base_timestamp + item.TimestampOffset(), item.Distance()])

class ProxHisory(SensorHistory):
    def __init__(self, size=100):
        super().__init__(size=size, dims=9)

    def _parse(self, base_timestamp, batch: sensors.SensorBatch):
        if not batch.ProximityIsNone(): 
            for i in range(batch.ProximityLength()):
                item = batch.Proximity(i)
                proximities = sensors.Uint16Array8()
                item.Proximity(proximities)
                self._add([
                    base_timestamp + item.TimestampOffset(), 
                    proximities.A0(), 
                    proximities.A1(), 
                    proximities.A2(), 
                    proximities.A3(), 
                    proximities.A4(), 
                    proximities.A5(), 
                    proximities.A6(), 
                    proximities.A7()
                ])
