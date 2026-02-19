# Firmware
This folder contains the necessary firmware for the processors on the epuck-2 robot. 
The `controller` folder contains the code for the main processor, the `STM3F407`.
The `radio` folder contains the code for the secondary `esp32` processor, which is supposed to be a bridge between the client (over wifi) and the main chip.

The contents of the two folders are technically git subtrees, since both projects were developed separately.
They have been merged together into this monorepo for easier management.

In the future, they might even be merged into a single PlatformIO project, instead of a separate `pio` project for the esp, and custom `Cmake` project for the stm chip.
This is however, purely theoretical since the amount of work is absolutely massive.
