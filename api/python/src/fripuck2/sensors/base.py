from collections import deque
from abc import ABC, abstractmethod
from typing import TypeVar, Generic, List, Optional
from .._generated.sensors_generated import SensorBatch

T = TypeVar('T')

class SensorHistory(ABC, Generic[T]):
    def __init__(self, size=100):
        self._size = size
        self._data: deque[T] = deque(maxlen=size)

    def _add(self, value: T):
        """Internal method to handle circular insertion."""
        self._data.append(value)

    @abstractmethod
    def _parse(self, base_timestamp: int, batch: SensorBatch):
        pass

    def get(self) -> Optional[T]:
        """Returns the most recent entry."""
        return self._data[-1] if self._data else None

    def get_all(self) -> List[T]:
        """Returns data in order and clears the buffer."""
        result = list(self._data)
        self._data.clear()
        return result

    def __len__(self):
        return len(self._data)