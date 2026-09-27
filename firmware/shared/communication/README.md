# STM ↔ ESP Communication 
The STM (controller) and ESP (radio) chips communicate over SPI, the controller being _master_ in this case. 
The controller sends data to the ESP in form of FlatBuffers. 
This way, the data is neatly serialized and ready to be send to the remote when it reaches the ESP.
This also ensures efficient packing of vectors of data.

The communication from the ESP to the STM is much lower in data volume (except if in the future we allow sending soud data to be played on speakers). 
Because of this, we use the UART interface since it is simpler to work with, and send data with [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) (there is little to no gain to use Flatbuffers again, since we do not need to package multiple data into a single packet, and helps reduce the computation needed to encode the data).
I decided to use the `cmacqueen/cobs-c` library so I didn't have to do everything from scratch.