import numpy
from abc import ABC, abstractmethod

class SensorHistory(ABC):
    def __init__(self, size=100, dims=2):
        self._size = size 
        self._dims = dims
        # Use dims to handle different array widths (e.g., [t, x, y, z])
        self._data = numpy.zeros((size, dims))
        self._ptr = 0
        self._count = 0

    def _add(self, values: list):
        """Internal method to handle circular insertion."""
        self._data[self._ptr] = values
        self._ptr = (self._ptr + 1) % self._size
        if self._count < self._size:
            self._count += 1

    @abstractmethod
    def _parse(self, base_timestamp, batch):
        """Subclasses must implement specific parsing logic here."""
        pass

    def get(self):
        """Returns the most recent entry."""
        if self._count == 0:
            return None
        return self._data[(self._ptr - 1) % self._size]

    def get_all(self): 
        """Returns sorted data and clears the buffer."""
        if self._count < self._size:
            result = self._data[:self._ptr].copy()
        else:
            result = numpy.roll(self._data, -self._ptr, axis=0)

        self._ptr = 0
        self._count = 0
        return result

    def __len__(self):
        return self._count