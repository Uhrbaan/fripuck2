from collections import deque
from abc import ABC, abstractmethod
from typing import TypeVar, Generic, Optional, Tuple

import numpy as np

from .._generated.sensors_generated import SensorBatch

T = TypeVar('T')


class SensorHistory(ABC, Generic[T]):
    """Rolling buffer of sensor samples, keeping values and timestamps apart.

    Timestamps are stored in seconds (float), values are whatever the sensor
    produces. Both are kept in parallel deques bounded by ``size``.
    """

    def __init__(self, size=100):
        self._size = size
        self._values: deque = deque(maxlen=size)
        self._timestamps: deque = deque(maxlen=size)

    def _add(self, value: T, timestamp: float):
        """Internal method to handle circular insertion."""
        self._values.append(value)
        self._timestamps.append(timestamp)

    @abstractmethod
    def _parse(self, base_timestamp: int, batch: SensorBatch):
        pass

    def get(self) -> Optional[T]:
        """Returns the most recent value (no timestamp)."""
        return self._values[-1] if self._values else None

    def get_all(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns every buffered sample as ``(values, timestamps)``.

        The buffer is left untouched: calling this twice returns the same data.
        Use :meth:`clear` to empty it.
        """
        if not self._values:
            return np.empty(0), np.empty(0, dtype=np.float64)
        return np.asarray(self._values), np.asarray(self._timestamps, dtype=np.float64)

    def clear(self):
        """Empties the buffer."""
        self._values.clear()
        self._timestamps.clear()

    def __len__(self):
        return len(self._values)
