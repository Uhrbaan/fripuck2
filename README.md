# E-Puck2 firmware fork for the university of Fribourg
This project aims to add lua support to program the robot on-board instead of only using the python library provided at https://github.com/davidfrisch/UNIFR_API_EPUCK.

This project is based on the original work made by [GCtronic](https://www.gctronic.com/) at https://github.com/e-puck2/e-puck2_main-processor.

## Installing 
The project uses `make` as its build system. 
Make sure that you have it installed: 
```sh 
sudo apt install make build-essential
```

Once you have installed the dependencies, you can clone the repository. 
Make sure you also download the submodules like [ChibiOS](https://github.com/e-puck2/ChibiOS_3.1.0/tree/f57f665733e8d1078533a235e43fbd3d3e17230f), [Aseba](https://github.com/aseba-community/aseba/tree/b18fb8f21a866495ef379c551ef104f88d37bd72) and [Lua](https://github.com/lua/lua).

```sh 
git clone --recurse-submodules https://github.com/Uhrbaan/fripuck2.git
```

Once that is done, you need to download the `gcc-arm-none-eabi` toolchain, the version 10.3 to be precise (other versions will not work).
You can download it from arm's website or [here](https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2?rev=78196d3461ba4c9089a67b5f33edf82a&revision=78196d34-61ba-4c90-89a6-7b5f33edf82a&hash=B94A380A17942218223CD08320496FB1) (last checked the 2025.11.05).

Once you have finished the download, extract it into the root of your project. 
The file structure should look something like this: 

```
fripuck2
├── gcc-arm-none-eabi-10.3-2021.10
│   ├── arm-none-eabi
│   ├── bin
│   ├── lib
│   └── share
...
```

Once you have installed everything, you can run 
```sh 
make -j16
```
to see if everything has been installed correctly. 
You may see many warnings, but there shouldn't be any errors. 
If there are, maybe you skipped a step. 
Make sure you have downloaded all submodules. 
If you haven't, run 
```
git submodule update --init --recursive
```

If it still doesn't work, try to run 
```sh 
make clean
``` 
before building the project again. 

If it still doesn't work, please open an issue. 

## Uploading code to the epuck
First, compile the code:
```sh 
make -j16
```

Before uploading the code to the robot, you will need to add your user to the `dialout` group:
```sh 
sudo usermod -aG dialout $USER
```
You might need to reboot your computer for the change to take effect.

> Note: Always be carful when running random code from the internet with root priviledges.

Then, you can run the provided `program.sh` script to upload the code to the robot.
Please plug in the robot to your computer with a USB cable, turn it on and then run:
```sh 
./program.sh
```

If you get an error saying there is no `/dev/ttyACM0` file, remove the cable, reboot the robot and plug it back in.

If the issue persist, please run:
```sh 
ls /dev/ttyACM*
```
If you get:
```
/dev/ttyACM0  /dev/ttyACM1
```
It should normally work. If it doesn't, please open an issue. 

If it doesn't, you can change `port=` in `program.sh` to the first file listed by `ls`.

If it works, you should see some text appearing on the terminal saying the file is downloading, and the led on the usb port on the robot should blink green. 
When the upload is finished, the new code is on the robot !

## Debugging 
This project uses VScode. 
It will thus describe how to setup the debugger inside of VScode. 

If you do not use VScode, you should know that the debugging works by connecting to a gdb server running on the robot, provided by [Black Magic Probe](https://black-magic.org/).
Good luck with your research !

On VScode, you will first need to install the `cortex-debug` extension from the [Visual Studio Code marketplace](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug).
The necessary `task.json` and `launch.json` configurations are already provided by this repository in the `.vscode` folder. 

If previously you had to change the `port=` in the `program.sh` file to make uploading work, you might also need to change the `"BMPGDBSerialPort":` field in the `launch.json` to match what you used in `port=`.

If everything is configured correctly, you should now be able to run the debugger with `Run → Start Debugging` or `F5` on the `Debug (Black Magic Probe)` launch target.
It will first start by building the firmware, and then will automatically load it, run it and stop on a breakpoint in `main`. 
After that, you can set breakpoints and use the debugger like you usually do. 

> Note: I have not yet had the time to attempt the installation and debugging process on a new machine. If you do and you encounter errors, please open an issue so I can fix the instructions.