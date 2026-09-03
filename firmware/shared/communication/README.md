# STM ↔ ESP Communication 
The STM (controller) and ESP (radio) chips communicate over SPI, the controller being _master_ in this case. 
The controller sends data to the ESP in form of FlatBuffers. 
This way, the data is neatly serialized and ready to be send to the remote when it reaches the ESP.
This also ensures efficient packing of vectors of data.

The communication from the ESP to the STM is much lower in data volume (except if in the future we allow sending soud data to be played on speakers). 
If we used FlatBuffers, unions would be quite handy but they too create a lot of overhead and are a pain to work with. 
For the volume of data we will be sending, FlatBuffers serialization is overkill, and we can use a simpler protocol. 
Since the SPI bridge is running at a high clock speed, which creates a little bit of corruption, we must also ensure that we have CRC to check if the commands sent from the ESP are correct. 

This is the packet structure: 
```
┌──────┬────────────┬───────────┬──────────────────────────────────────────┬────────┐
│ SYNC │ PACKET_LEN │ CMD_COUNT │ COMMAND PAYLOAD (Sequence of Opcodes)    │ CRC-16 │
│ 0xA5 │  (1 Byte)  │ (1 Byte)  │ [OPCODE_1 + ARGS] [OPCODE_2 + ARGS] ...  │ (2B)   │
└──────┴────────────┴───────────┴──────────────────────────────────────────┴────────┘
```

It allows to chain multiple commands into a single packet, each command comming with its own opcode, followed by a struct of known length.
There are only a limited number of actuators connected to the controller chip: 
- 6 standard LEDS (the other 4 RGB leds are connected to the radio chip)
- 2 motors  (iplemented as a single module)
- speaker (support not planned for now)
- micro-sd (support not planned)

Beyond controlling the actuators, we must also be able en/disable the different sensors, which include: 
- Mode selector (planned, not implemented)
- IR reveiver (planned, not implemented)
- Battery
- 8 proximty sensors
- ToF distance sensor
- Inertial measurement unit
- Camera (planned, not implemented)

## Opcodes
- `0x0`: Using a mask to enable/disable modules