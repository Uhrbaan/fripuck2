# What there is left to do

## Hardware

- Actuators
  - [x] LEDS
  - [x] Motors
  - [ ] Motors extended (allow advancing a specific amount of steps)
- Sensors
  - Analog
    - [x] Proximity
  - I2C
    - [x] Distance sensor (TOF)
    - [x] IMU
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
  - [x] RX
    - Not perfect. If sender sends two packets exactly one after the other the
      idle line detection might not work. Might implement some logic to fire
      the user callback multiple times if multiple packets are recieved at
      once, or add mandatory delay in the sender.
    - This problem could also possibly be handled by the user, who has more
      context on the data.
  - [ ] TX
- [x] SPI (dummy) STM -> ESP
- [ ] SPI (dummy) ESP -> STM (hard)
- [x] data over UDP  (dummy)
- [ ] data over TCP (commands, dummy) -> UART: ESP -> STM

- - -
- [x] Sending sensor data to the user over SPI & UDP
- [ ] Sending notifications to the user over UART & TCP
- [ ] Receiving instructions over TCP & UART
- [ ] Receiving data over UDP/TCP & SPI

## VM

- [ ] Get Lua running
- [ ] Upload Lua bytecode
- [ ] Create the library
  - [ ] functions to controll all hardware
  - [ ] functions to intercept and act on TCP chunks
  - [ ] functions to intercept and act on Data
    - [ ] Accelerated functions for data manipulation

## Testing

I want to add tests using platformIO's testing framework. It should test for
breakage and throughput mostly.

## Documentation

- [ ] Add rate per second difference in readme
- [ ] Document all public function
  - [ ] Hardware (controller)
  - [ ] Hardware (radio)
  - [ ] API (python)
