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
