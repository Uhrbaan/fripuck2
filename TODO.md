# What there is left to do 
## Hardware
- Actuators 
    - [X] LEDS
    - [X] Motors 
    - [ ] Motors extended (allow advancing a specific amount of steps)
- Sensors
    - Analog
        - [ ] Proximity 
    - I2C
        - [ ] Distance sensor (TOF) 
        - [ ] IMU 
        - [ ] Ground sensors 
        - [ ] Camera (setup)
    - DCMI 
        - [ ] Camera (data)
    - SPI (I2S) 
        - [ ] Microphones (1-4)
        - [ ] Encoders (left and right)
    - Binary
        - [ ] Selector
 
## Communication
- [ ] UART
    - [X] RX
        - [ ] Not perfect. If sender sends two packets exactly one after the other the idle line detection might not work. Might implement some logic to fire the user callback multiple times if multiple packets are recieved at once, or add mandatory delay in the sender.
        - This problem could also possibly be handled by the user, who has more context on the data.
    - [ ] TX
- [X] SPI (dummy) STM -> ESP
- [ ] SPI (dummy) ESP -> STM (hard)
- [X] data over UDP  (dummy)
- [ ] data over TCP (commands, dummy) -> UART: ESP -> STM

## VM 
- [ ] Get Lua running 
- [ ] Upload Lua bytecode
- [ ] Create the library
    - [ ] functions to controll all hardware 
    - [ ] functions to intercept and act on TCP chunks
    - [ ] functions to intercept and act on Data
        - [ ] Accelerated functions for data manipulation


## Testing
I want to add tests using platformIO's testing framework.
It should test for breakage and throughput mostly.