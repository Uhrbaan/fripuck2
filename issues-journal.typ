= Issues
== 2025.11.06
Having to create a lua vm with a custom allocator (```c lua_newstate(lua_Alloc, NULL)```).
A simple allocator was created.

== 2025.11.08
=== Bytecode
To solve an issue, switched to the lua VM reading bytecode directly.
This was achived by compiling a test script with
```sh
luac -o test.lua.bytecode test.lua
xxd -i test.lua.bytecode
```

xxd is used to generate the byte array I can paste in C.

Some issues arose: got error `3` when calling `lua_loadbuffer` since the version of my `luac` compiler did not match the version of the lua submodule.
To solve this, I needed to checkout the submodule on the `3.4.8` branch, which is the version of my compiler.

Downgrading the lua implementation created a problem: now can't change the lua seed to be fixed, which is annoying since the lua implementation uses the default time(NULL) as a seed.
This meant I had to define a `time` function to make it work (in luaport.c)

=== `lua_pcall` fails because of `setjmp`
After discussion with an AI, the issue seems to be that the setjmp function as implemented probably does not correctly save the FPU state, thus creating an error when jumping back from it (this is some really low level shit).

One attempted fix (as suggested by an IA, will have to examing why it would work), is to set the `USE_FPU=soft` flag.
This however creates an error (so it did something
!!):
```
.//src/cmd.c: In function 'cmd_sqrt':
.//src/cmd.c:446:13: error: inconsistent operand constraints in an 'asm'
  446 |             __asm__ volatile (
      |             ^~~~~~~
```
This issue is due to the fact that since we are not using the `hard` flag anymore, we can't use the inline assemble anymore (why? IDK.)

So a fix is to simply replace the inline assembly:
```c
x = (float) input;
__asm__ volatile (
    "vsqrt.f32 %[var], %[var]"
    : [var] "+t" (x)
    );
```
with some standard c code (from the standard library): ```c x = sqrtf(x);```.
This however will probably impact performance, but who cares at this point.

== 2025.11.09
So it turned out this wasn't the issue.
The flag was removed and somehow the program now magically works without explanations.
The change was thus reverted.
Will have to look into it for the report.
Will also have to look at the diff to see what could have solved the issue.

If I would have to take a wild guess, I'd say that the code stopped working because I removed some necessary initialisation code withou paying attention to it.
But this has to be confirmed.

Anyway, happy that it works.

== 2025.11.14
Working on reimplementing the old asercom protocol and making it run alongside the lua VM.

Had to connect the robot to my network.
Thankfully, Gtronic has a pretty good guide here: https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development how to configure the wifi, which went quite well.

Then, the asercom protocol worked without many issues.

When creating a thread for the asercom protocol I simply reused the selector thread that I wasn't using, and created a separate thread for lua.

At first it worked, but then I compiled the project again and it stopped working for some reason.
One of the issues could be traced back to a stack overflow thrown by the TOF thread, so I increased the size to 1024, which seemed to fix the panic.
Also increased the thread size of the lua VM to 8kb insdead of 2, which is more realistic.

However, now the robot doesn't panic but it doesn't seem to launch the lua nor the asercom thread, and instead gets stuck in a `_idle_thread` which does nothing.

This was a mistake.
It is normal that there is an idle thread that does nothing.
The issue was that the update to `Asercom.c` made the `use_bt = 0` which meant that the protocol was not set to look for bytes arriving on wifi (logically, since we didn't set the bluetooth flag 🤷).

So I replace the block that chose the flag based on the selector simply to be fixed to 1, since we don't even launch the thread if we don't want wifi support.

#line(length: 100%)

Now, we have to do a file upload.
Looking at the way the Asercom protocol is implemented, the first byte of the packet determines the case that will decrypt it.
To send a file, we either have to send a really large stream of data, which is usually advised against, or blocks of data -- let's go with this, more common approach.

We will allocate an empty ID -> `0x13` as the packet command for the file transfer.
It will then be followed by a packet id (in case it is out-of-order), the length of the data, the data and a checksum (CRC since it looks quite easy to copy the implementation from other places, and is the standard for other protocols).

Also, the fact that I couldn't print stuff for information or logging was annoying.
Turned out the solution was to use the `chprintf` fonction on the `SDU1` serial over usb.
Then, I can `screen` `/dev/ttyACM2` to see the prints.

However the method seems unreliable, and doesn't seem to work outside of the main function.
But technically, if brave, this:
```c
chprintf((BaseSequentialStream*)&SDU1, "Got command %#02x\n", c);
```
technically works. Then just:
```sh
screen /dev/ttyACM2
```
if it exists.


Another problem is endianness.
Since we are working with a network protocol, we use big-endianness, which mixes up some numbers.
We have to take care of that.

In the end of this session, I had some issues with the robot not wanting to behave and connect to my home network.

== 2025.11.15
Having issues with the debugging and understanding the flow of the code of the asercom 2 protocol.
Have started to restructure the code in a more modern way, using function pointers in a dispatch table.
This should also make debugging easier, since each packet type is isolated into its own function.

Half of the asercom 1 protocol (the ascii part) has already been ported over.
Will do the binary part and additions another day.
This is all in preparation to do v0.3.

Will also have to think about using a microsd to store the lua bytecode on the robot, instead of placing it on the flash, which to my small research is advised agains ?
If using the flash, I will have to look into how to store files into it, which isn't trivial.
Maybe I will postpone this part to a later update, maybe even 1.x.

== 2025.11.16
While porting to asercom3, I also noticed there are a lot of these statements:
```c
while (e_getchar_uart1(&c1) == 0);
```

Which translate to:
```c
while (chSequentialStreamRead(uart_target, &c1, 1) == 0)
    ;
```

in Asercom3.
I just don't understand why we are waiting to read something that is not of length 0.

$=>$ wait, we are simply waiting for the bytes to arrive I gues...

All of the necessary functions were ported over.
Testing is needed. Also the report should mention all of them that didn't get included and for which reastons: $=>$ extensions that are not in use, etc.
