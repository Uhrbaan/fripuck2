#!/bin/bash

# small script that prints a backtrace from backtrace codes from the clipboard (must have wl-paste installed)
pio pkg exec --package "espressif/toolchain-xtensa-esp32s3" -- xtensa-esp32s3-elf-addr2line -pfiaC -e .pio/build/esp32dev/firmware.elf $(wl-paste)