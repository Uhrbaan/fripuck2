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
 
## Communication
- [ ] UART
    - [X] RX
        - [ ] Not perfect. If sender sends two packets exactly one after the other the idle line detection might not work. Might implement some logic to fire the user callback multiple times if multiple packets are recieved at once, or add mandatory delay in the sender.
        - This problem could also possibly be handled by the user, who has more context on the data.
    - [ ] TX
- [ ] SPI