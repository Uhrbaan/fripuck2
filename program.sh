#!/bin/bash

GDB = gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gdb

port=$(python3 list_com_ports_F407.py);
if [ "$port" = "" ]; then
    echo "Robot not detected, be sure the robot is connected to the computer"
    exit
fi
echo "Robot detected on port $port";

filename=$(find . -name *.elf)
if [ "$filename" = "" ]; then
    echo "Firmware not found, be sure you put the firmware (.elf) inside this directory"
    exit
fi
echo "Robot will be updated with firmare $filename";

$GDB \
--interpreter=mi -nx \
-ex "target extended-remote $port" \
-ex "monitor swdp_scan" \
-ex "attach 1" \
-ex "set mem inaccessible-by-default off" \
-ex "load" \
$filename \
-ex quit
# -ex "b main" \
# -ex "c"