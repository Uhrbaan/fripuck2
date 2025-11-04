#/bin/bash

port=/dev/ttyACM0;

filename=$(find . -name *.elf)
if [ "$filename" = "" ]; then
    echo "Firmware not found, be sure you put the firmware (.elf) inside this directory"
    exit
fi
echo "Robot will be updated with firmare $filename";

gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gdb --interpreter=mi -nx \
-ex "target extended-remote $port" \
-ex "monitor swdp_scan" \
-ex "attach 1" \
-ex "set mem inaccessible-by-default off" \
-ex "load" \
$filename \
-ex "set confirm off" \
-ex quit




