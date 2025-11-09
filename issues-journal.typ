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
