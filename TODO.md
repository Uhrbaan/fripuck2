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
  - [ ] Create 3 modes: lua only, asercom only and asercom *and* lua mode 
  - [ ] Create a separate thread for lua and asercom
  - [ ] Make them run in paralell without running out of memory

## v0.3
- [ ] Upload bytecode to the robot
  - [ ] Modify asecom 2 for a new "bytecode upload" mode 
  - [ ] Update the python library to check and compile lua code to bytecode
  - [ ] Update the python library to upload the bytecode with a checksum
  - [ ] Restart upload if checksum fails
- [ ] Save the bytecode on device
- [ ] Run specific bytecode based on available wheel
  
# v1.0
`v1` will implement the basics - getting the Lua VM to work _with_ the already existing network protocol.
`v1` should be reached in early january.