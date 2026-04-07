# Fripuck2
Technology stack for the robotics course of the university of Fribourg. 

Fripuck2 is a re-implementation of original e-puck2 firmware and api (which can be found [here](https://github.com/e-puck2/e-puck2_main-processor), [here](https://github.com/gctronic/epuck2-esp32) and [here](https://github.com/davidfrisch/UNIFR_API_EPUCK)), originally developped by the [EPFL](https://www.epfl.ch/labs/mobots/robots-technologies/e-puck2/), built and maintained by [GCtronic](https://www.gctronic.com/doc/index.php/e-puck2).

## Goals 
The project's goal is to optimize the firmware and api of the robot to better fit the needs of the university's requirements, as well as enabeling me to delve in the world of embedded development.

This project is developped as my bachelor thesis in collaboration with the [Human-IST institute](https://human-ist.unifr.ch/en/).

## Technology stack 
While the previous project used a mix of [ChibiOS](https://www.chibios.org/dokuwiki/doku.php) and [FreeRTOS](https://www.freertos.org/), this project uses only FreeRTOS on the firmware for consistency.

Both chips use [PlatformIO](https://platformio.org/) as the build and dependency management system to simplify development and debugging.

To transmit data between the robot's chips and the API, [Flatbuffers](https://flatbuffers.dev/) is used rather than a custom protocol for efficiency and ease of development. 
Thus, the whole protocol is exposed in the schema (`.fbs`) files in the `protocol-schemas` directory.

## Feature parity
Reaching full feature parity is probably an impossible task since I am alone for this project. The following table shows my progress and improvements next to the _status quo_.

| Category | Feature | _Status quo_ | Fripuck2 |
|-|-|:-:|:-:|
| **Movement** | Speed control | ✅ 
| | Distance target | ❌ 
| | Encoder feedback | ✅ 
| **Sensing** | Proximity sensors | ✅
|| Accelerometers | ✅ 
|| Gyroscope | ✅
|| Magnetometer | Available in firmware
|| Time of flight | ✅ | ✅
|| Proximity | ✅ | ✅
| **Vision** | Color video feedback | ✅
|| Basic image processing | ✅ 
|| Yolo (AI) image processing | ✅
| **Audio** | Microphones volume | ✅
|| Microphones audio stream | ❌
|| Play predefined sound | ✅
|| Play streamed audio | ❌
| **Feedback** | Red LEDs | ✅
|| Other LEDs | ✅
| **Communication** | Bluetooth LE | Available in firmware | ❌\ Not planned
|| Wi-Fi (station) | ✅ | ✅ |
|| Wi-Fi (access point) | Available in firmware | 
|| Manage connection loss | 
| **I/O** | TV Remote | ✅
|| SD card | Available in firmware
| **Control logic** | Real-time control | ✅
|| Command sequencing | ❌
| **Data handling** | Real-time Streaming | ✅
|| High-frequency data aggregation | ❌ 
| **Miscelaneous** | Conversion to SI units | ❌
|| Graphical intercace | ✅
|| ROS support | ✅

## Building
> Note: For now, building has only been tested on Linux. 

First, make sure you have access to the robot over serial. You may need to add your user to the `dialout` or `uucp` depending on your distribution (should be `dialout` on most, `uucp` on arch)
```sh
usermod -aG dialout $USER
```

Then, to build the firmware you need [PlatformIO](https://platformio.org/). You can install the [VScode extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide), or you can install the [PlatformIO Core CLI](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html).

If you chose the latter (or both), you can also use the makefile at the root of the project for convenience.

```sh 
make generate # generates flatbuffers code if schemas were changed
make flash # flashes the chips with pio
```
