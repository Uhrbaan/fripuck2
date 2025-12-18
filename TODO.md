# v0.0
Version `v0` focused on testing to make sure the setup was working. 
I got the debugger working inside of VScode (although really janky) and got a basic lua script working on the robot. 
Later the standard lua code will be changed to simply running bytecode, for the sake of memory efficiency (parsing lua code is very memory hungry).
Also, compiling the lua code on the host machine has the benefit that you are aware earlier of a parsing error.

`v0` was achieved on the 3.11.2025.

## v0.1
- [X] Get lua bytecode working on the robot
  - [X] Custom allocator 
  - [X] Create VM
  - [X] Upload bytecode
  - [X] Run bytecode 

Note: in this version, to run some lua code on the machine, you first need to write valid lua code into a file, say `test.lua`.
Then, compile that code to lua bytecode with the `luac` compiler, and finally, include that bytecode into your C project. 

```sh 
vim test.lua # write your code 
luac -o test.lua.bytecode test.lua # compile to bytecode
xxd -i test.lua.bytecode # prints the bytes as a C array, copy and paste it into your code.
```

## v0.2
- [X] Run lua code in parallell of default asercom protocol 
  - [X] Restore old protocol
  - [X] Create 4 modes: 
    - `0`: legacy: run asercom2 protocol just like it used to 
    - `1`: lua + asercom: run the two protocols in paralell (without communication between them for now) 
    - `2-9`: Reseved for future use 
    - `A-F`: Places where students can save lua scripts on ROM to load later (can be used for demonstrations).
      - This may change later. 
      - NOT YET IMPLEMENTED.
  - [X] Create a separate thread for lua and asercom
  - [X] Make them run in paralell without running out of memory

> Note: currently, since the python api constantly sends datapackets that set *everything* on the robot, the example lights will flicker, since lua will enable the leds, and a milisecond after the asercom protocol gets a new packet where the state is "off". 
> This means one will have to rework the interpretation of packets so the data doesn't get updated if it is not intentionally set. 
> This can be tricky since we have to decide on the priority.

## v0.3
- [ ] Upload bytecode to the robot
  - [X] Design new protocol for data upload 
  - [X] Create new entry in the asercom protocol (0x1b)
  - [ ] Verify checksums 
  - [X] Load bytecode to Lua 
  - [ ] Restart upload if checksum fails
- [ ] Solve the `0xf8`, `0xf7`, `0x00` problem ***secondary firmware***
- [ ] Improve the tcp side of things on the esp firmware ***secondary firmware***

> Note, as of writing, I cannot get upload over tcp (`SD3`) working because of some noise that comes from somewhere I couldn't understand where.

## v0.4
- [ ] Limit conflicts between lua and asercom instructions (race conditions)
  - [ ] Lock functions used by lua thread (priority)
- [ ] Save Lua code to memory

## v0.5
- [ ] Create a basic library of functions students can use
  - [ ] Decide on which default lua modules to keep 
  - [ ] Implement all the hardware initialization functions 
  - [ ] Implement all the getters for the different sensors
- [ ] Minor adaptations of the python library
  - [ ] Update the python library to check and compile lua code to bytecode
  - [ ] Update the python library to upload the bytecode with a checksum


# v1.0
`v1` will implement the basics - getting the Lua VM to work _with_ the already existing network protocol.
`v1` should be reached in early january.
The network interface will be implementedd to remove some errors and make it as efficient and simple as possible.

# v2.0
`v2`, which has not be thought through yet, will be about networking. 
This update will rework how data is transmitted and shared between the two processes. 
Some sensor readings will be moved to the ESP32 where it makes sense. 
The asercom protocol will probably be removed from the main version, but kept for legacy reasons. 
It will be the responsability of the Lua protocol to recieve instructions and handle them -- and thus the student's role to do them. However, a basic implementationo will be provided as a standard library for usability. 
Lua will be able to configure a UDP stream that sends (very) frequent data readings for the computer. 
Ideally, the system should be robust enough to keep an audio stream both ways, but this is optional since it will require *a lot* of work. 

# v3.0
This version does mostly clean-up. 
It will remove unnecessary and legacy code (except the asercom2 protocol), and will move both of the processor firmware to a single programming environment, probably PlatformIO, if possible, to make development easier and streamlined. 
Ideally, it should be accessible to students in their first year of Bachelor.

> Note: Porting to platformIO seems like a real challenge, maybe it will take too long.

Another, really important part will be to write documentation, for the C api, the Lua API and the Python API. 