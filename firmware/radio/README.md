# Fripuck2 radio firmware 
This project contains the firmware necessary to flash the esp32 on the epuck2 made by GCtronic.
In the fripuck2 project, this (secondary) processor is responsible for all the radio communication, especially Wi-Fi which the university of Fribourg uses.
The other connectivity methods like bluetooth are unsupported.
This code is *technically* a fork of the GCtronic's repository at <https://github.com/e-puck2/esp-idf/tree/wifi>, but doesn't include the entire esp-idf system with it for the sake of simplicity. 
Most of the code has been originally written by GCtronic. 

## Building 
We need to program the espressif chip that lives on `/dev/ttyACM1` (usually, if it doesn't it's sad I guess).

```sh
# 1. Install the external esp-idf project 
git clone -b release/v6.0 https://github.com/espressif/esp-idf.git --recursive # clone the esp-idf project
cd esp-idf 
./install.sh # install necessary dependencies
. ./export.sh # setup the necessary environment variables. You MUST run this in every terminal session where you use the idf.py tool.

# 2. Install this project 
cd ..
git clone https://github.com/Uhrbaan/fripuck2-radio.git # clone this repository
cd fripuck2-radio
idf.py component install # install necessary components
idf.py build # build the project. Make sure you ran export.sh !
idf.py -p /dev/ttyACM1 flash # The port may change. Please double check if you get an error.
```
