import wave
import numpy as np
from .._generated import sensors_generated as sensors
from typing import Optional
from .base import SensorHistory

class AudioHistory(SensorHistory[float]):
    """Keeps one int16 array of PCM samples per 10 ms frame, with timestamps alongside."""

    def __init__(self, size=100, sample_rate=16000):
        super().__init__(size=size)
        self.sample_rate = sample_rate

    def _parse(self, base_timestamp, batch: sensors.SensorBatch):
        count = batch.AudioMetadataLength()
        if count == 0:
            return

        # We are now using the dedicated flatbuffer array for the right channel
        # rather than the generic blob.
        blob = batch.AudioChannelRightAsNumpy()

        for i in range(count):
            meta = batch.AudioMetadata(i)
            
            # Extract the raw PCM bytes for this frame from the flatbuffer
            frame = np.asarray(blob[meta.Offset():meta.Offset() + meta.Length()], dtype=np.uint8)
            
            if len(frame) != meta.Length():
                print("Audio frame outside blob bounds, skipping.")
                continue

            # Ensure the length is even (16-bit samples = 2 bytes per sample)
            if len(frame) % 2 != 0:
                print("Audio frame has a bad length (not divisible by 2), skipping.")
                continue

            # Convert raw little-endian bytes directly to 16-bit signed integers
            # No decompression needed anymore!
            pcm = frame.view('<i2')

            if meta.SampleCount() and meta.SampleCount() != len(pcm):
                print(f"Audio frame: read {len(pcm)} samples, metadata says {meta.SampleCount()}")
                
            self._add(pcm, (base_timestamp + meta.TimestampOffset()) / 1000)

    def get_pcm(self) -> np.ndarray:
        """All buffered audio as one flat int16 array. Leaves the buffer untouched."""
        frames = list(self._values)
        return np.concatenate(frames) if frames else np.empty(0, dtype=np.int16)

    def clear(self):
        """Empties the buffer. Call this after get_pcm()/save_wav() if you don't want to keep the audio in memory."""
        super().clear()

    def save_wav(self, path: str):
        """Writes buffered audio as a standard mono 16kHz WAV file."""
        pcm = self.get_pcm()
        with wave.open(path, "wb") as w:
            w.setnchannels(1)  # Mono, since we only send the right mic
            w.setsampwidth(2)  # 16-bit
            w.setframerate(self.sample_rate)
            w.writeframes(pcm.tobytes())