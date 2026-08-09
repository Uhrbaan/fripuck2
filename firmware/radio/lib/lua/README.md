The project uses the 5.1.5 release of Lua (the latest v5.1 release). It has brough some slight modifications so it compiles on the esp32.

Notable changes: 
- removed files that are not useful or don't make sense in an embedded environment: 
    - `lua.c`: don't need standalone Lua CLI
    - `luac.c`: don't need standalone Lua compiler
    - `liolib.c`: standard file IO doesn't exist in an embedded environment. However, might add it back if adding a small FS.
    - `loadlib.c`: cannot load .so files anyway (dynamic module loading).
    