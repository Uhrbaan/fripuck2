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
- [ ] Run lua code in parallell of default asercom protocol 
  - [ ] Restore old protocol
  - [ ] Create 4 modes: 
    - `0`: nothing (run `main.c` in `Project_template`, which can be implemented by a student in C)
    - `1`: lua only (runs a single provided lua script, later will load latest from memory)
    - `2`: legacy: run asercom2 protocol just like it used to 
    - `3`: lua + asercom: run the two protocols in paralell (without communication between them for now) 
  - [ ] Create a separate thread for lua and asercom
  - [ ] Make them run in paralell without running out of memory

## v0.3
- [ ] Upload bytecode to the robot
  - [ ] Modify asecom 2 for a new "bytecode upload" mode 
  - [ ] Update the python library to check and compile lua code to bytecode
  - [ ] Update the python library to upload the bytecode with a checksum
  - [ ] Restart upload if checksum fails
- [ ] Save the bytecode on device
- [ ] Run specific bytecode based on the selector, maybe ?
  
## v0.4 
- [ ] Create a basic library of functions students can use
  - [ ] Decide on which default lua modules to keep 
  - [ ] Implement all the hardware initialization functions 
  - [ ] Implement all the getters for the different sensors
- [ ] Clean up the code 
  - [ ] Organize my own code better
  - [ ] Remove unnecessary dependencies 
- [ ] **Write documentation**
  - [ ] For the lua functions
  - [ ] For the added C code
  - [ ] For the python interface.

# v1.0
`v1` will implement the basics - getting the Lua VM to work _with_ the already existing network protocol.
`v1` should be reached in early january.

# v2.0
`v2`, which has not be thought through yet, will be about networking. 
The main goal will be to 
1. Make the current network implementation faster and/or more robust
2. Make it configurable with such that the student can decide what to send (possibly sending their own packets from lua).4.
3. Maybe, if the work isn't enough for a bachelor thesis, think of (re)implementing other means of communication, for example through serial, bluetooth, hotspot etc.