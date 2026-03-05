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

== 2025.11.17
So, asercom3 was implemented, but some testing is required.
Right now I am going to move forward and try to get file upload working.
Thinking of a code to use, I think the easiest would be to use the magic number lua uses for its bytecode.
Each lua bytecode stars with `\x1B Lua`, so we can use the `1b` magic number as the command, since it is not in use.
This should also make it possible to pipe lua bytecode directly into `/dev/ttyACM2` without additional work, which could be handy.

```
❯ xxd test.lua.bytecode
00000000: 1b4c 7561 5400 1993 0d0a 1a0a 0408 0878  .LuaT..........x
00000010: 5600 0000 0000 0000 0000 0000 2877 4001  V...........(w@.
00000020: 8a40 7465 7374 2e6c 7561 8080 0001 038f  .@test.lua......
00000030: 5100 0000 0180 ff7f 1500 0080 2f00 8006  Q.........../...
00000040: 4000 8e00 3800 0080 0180 ff7f 8b00 0000  @...8...........
00000050: 0001 0000 c400 0201 8b00 0001 0101 0080  ................
00000060: c400 0201 b8f9 ff7f c600 0101 8204 8b65  ...............e
00000070: 6e61 626c 654c 4544 7304 8673 6c65 6570  nableLEDs..sleep
00000080: 8101 0000 808f 0100 0200 0100 0102 0000  ................
00000090: 0100 0000 0180 8184 6c65 6482 8f81 855f  ........led...._
000000a0: 454e 56                                  ENV
```

UPDATE: I scrapped this idea. It would be really cool but by reading online, since we get the data from the connectivity processor over uart, and uart is really unreliable and slow, we should send the file in small chunks and continuously check the data integrity with a checksum, reconstruct the data etc.

=== Designing the protocol
I'll still keep the `1b` protocol since it doesn't matter anyway it's a magic number.

We will segment everything in packets of 512 bytes.

Here is the format I am thinking off:
#figure(table(
  columns: 512,
  align: horizon + center,
  table.cell(colspan: 512)[Total size: 512 bytes],
  table.cell(colspan: 1)[`0x1B`\ 1 byte],
  table.cell(colspan: 2)[total bytecode length\ 2 bytes],
  table.cell(colspan: 2)[offset\ 2 bytes],
  table.cell(colspan: 2)[packet length \ 2 bytes],
  table.cell(colspan: 512 - 9)[data\ at most 503 bytes],
  table.cell(colspan: 2)[CRC-16\ 2 bytes],
))

We first send the command, then the offset (where this data should be written in the buffer), the data and finally a CRC.

== 2025.11.21
Currently trying to test the asercom3 protocol.
I am facing a weird issue where if I use the wifi connection (`SD3`), I always get the message `f8, f7, 0` looping, and I can't figure out why.
For now I am going to communicate with the robot through `SDU1`, or the usb cable, to test out the different functions.

Also, the lua VM must not run until the lua script is fully uploaded.
To do that, I will just create a lock, and unlock once everything is uploaded.
In the future, it is also important for the VM to react if a new file is uploaded while it is running. The simplest would simply be to ignore it. It's what I will do for now.
It will work like that:

At first, the buffer is `unlocked`.
If we start recieving files, we `lock` it, undil we have recieved all the bytes.
Once we did, we will `unlock` the buffer, letting the lua VM take car of it.

Once the buffer is `unlocked` again, and the lua VM sees that there is a valid pointer to read memory from, it will `lock` the buffer, read the instructions and free the memory since it doesn't need it anymore.

As long as the buffer is locked, the file upload will fail.
If the lua VM terminates, it will unlock the file again and the process continues.

$=>$ Scrap that idea. I will just reaplace that with a "upload complete" flag, which is a lot easier.

$=>$ Srap: using a chibios binary semaphore to send a signal when the upload is complete.

It now works. I had an issue where the lua thread wasn't blocking, but it was simply because I didn't set it to `true` initially.
However, when changing to tcp, the problem with the repeating `f8`, `f7` and `0` still remains.

== 2025.11.26
Met Julien today.
The most important things right now are: *solving the f8 f7 0 errors*, *improving the current tcp communication implementation* and finally *finalizing the lua upload code*.

Have done the first steps to try and modify the radio module firmware.
After some issues, I thought the simplest solution would be to use platformIO (which I am more familiar with, and is a much nicer platform to work with), and port the current implementation over (partially, only the necessary part).
Managed to get compilation and flashing working, but for some reason, logging does not work.

Even at the end of the day, logging is not working.

== 2025.11.27
Fixed the logging issue. the robot had to be set to a baudrate of 230400, which was updated in the sdkconfig file.
now, logging works well, and I can move to the networking.

Also identified the `0xf8`, `0xf7` issue in the asercom function (on the radio firmware, where the bits are set and send through the UART channel)

== 2025.11.28
Currently working on the wifi/tcp implementation.
Current version runs pretty slow, but it probably can't be much better.
Tested on the commit `79e36b1` with the following script:
```py
import socket
import time

# --- CONFIGURATION ---
HOST = '192.168.1.179'  # Your ESP32's IP address
PORT = 1000             # Your TCP port
PAYLOAD = b'Test Message' # Use bytes for sending/receiving
NUM_TRIALS = 100        # Number of times to repeat the test
# ---------------------

def measure_rtt():
    rtt_measurements = []

    for i in range(NUM_TRIALS):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                # 1. Connect
                s.connect((HOST, PORT))

                # 2. Start Timer
                start_time = time.time()

                # 3. Send Data
                s.sendall(PAYLOAD)

                # 4. Receive Data (Echo)
                received_data = s.recv(len(PAYLOAD))

                # 5. Stop Timer
                end_time = time.time()

                # 6. Validate and Record
                if received_data == PAYLOAD:
                    rtt = (end_time - start_time) * 1000 # Convert to milliseconds
                    rtt_measurements.append(rtt)
                else:
                    print(f"Trial {i+1}: Received corrupted data.")

            except Exception as e:
                print(f"Trial {i+1}: An error occurred: {e}")
                time.sleep(0.1) # Wait before next trial
                continue

    if not rtt_measurements:
        print("\nFailed to complete any successful RTT measurements.")
        return

    # Calculate and Display Results
    min_rtt = min(rtt_measurements)
    max_rtt = max(rtt_measurements)
    avg_rtt = sum(rtt_measurements) / len(rtt_measurements)

    print("\n--- RTT Test Results ---")
    print(f"Total successful trials: {len(rtt_measurements)}")
    print(f"Payload Size: {len(PAYLOAD)} bytes")
    print(f"Minimum RTT: {min_rtt:.3f} ms")
    print(f"Maximum RTT: {max_rtt:.3f} ms")
    print(f"Average RTT: **{avg_rtt:.3f} ms**")

if __name__ == "__main__":
    measure_rtt()
```

I got the following result:
```
❯ python3 round-trip-time.py

--- RTT Test Results ---
Total successful trials: 100
Payload Size: 12 bytes
Minimum RTT: 6.529 ms
Maximum RTT: 29.971 ms
Average RTT: **12.514 ms**
```


So now that TCP is working, I need to design a simple system so the network and serial interface can work simoultaneously.
What I was thinking was:

- Have 2 processes,
  - "network" handles the tcp connection, revieves/sends data from/to a client
  - "serial" handles the spi connection, recieves/sends data from/to the network process and the STMF4.

Network has two tasks: Recieving data from the client, and giving it to the "serial" taks, and taking data from the "serial" task and sending it to the client.
The first task will be called 'client2robot' and the second 'robot2client'.

Serial also has two tasks: receiving data from Network, and sending it over uart to the STMF4 chip. The second task is to take a response from the stmf4 chip and sending it back to Network.
The first taks will be called 'esp2stm', and the second 'stm2esp', since it's the names of the chips.

`client2robot` continuously gets data from the client.
It stores it in a buffer with three slots: `client2robot` will select the oldest unlocked slot lock it, write to it and unlock it.
`esp2stm`, if it's not busy, will ready the latest unlocked slot.
It will lock it, and while it is sending if over uart, the `client2robot` will continue to update the other buffers, one after the other, to the newely recieved instruction.
The rule for `client2robot` is simple: if you recieve a payload over TCP, write it to a buffer that *is unlocked* and is the *oldest*.
The rule for `esp2stm` is also simple: if you are not busy, upload the *youngest* and *unlocked* buffer.

`stm2esp` continuously reads data from SPI. Since I believe the spi connection will be the bottleneck, I will only have two buffer slots, and send a "ready" signal as soon as the upload of one is complete, then `robot2client` will lock the one that was ready and send it, while the `stm2esp` fills in the second slot.


Then, thinking a little more, since TCP should never lose data, I shouldn't go with the 3 buffer approach where it overwrites one of the instructions, but a queue.
If the queue fills up, then I should apply "backpressure"#footnote[Never heard of that before...] to slow the client down.

Overall, this is a *"producer/consumer"* architecture, which I have to remind myself of for the report.

#quote(block: true, attribution: [Gemini AI agent])[
  Your project uses a multi-threaded (or multi-task) architecture to efficiently bridge a high-speed network connection with a slower, local serial link.

  Network Interface: Two tasks (Net_Cmd_Receiver and Net_Resp_Sender) manage the TCP/IP communication with the external client.

  Serial Interface: Two tasks (Serial_Cmd_Sender and Serial_Resp_Receiver) manage the slower, local SPI/UART link to the STM32 chip.

  Command Flow (Client → STM32): Commands from the client are placed into a FIFO Queue by the network task, creating back-pressure to prevent the faster TCP connection from overwhelming the slower serial link.

  Response Flow (STM32 → Client): Sensor/response data from the STM32 is handled using a double-buffering mechanism protected by a Mutex, allowing the serial task to continuously fill one buffer while the network task transmits the other, maximizing throughput.
]

== 2025.11.29
Working on implementing communication over spi.
Implented the first part of the control flow, so client → robot → queue → spi.
Currently, have some weird regression where cannot connect to wifi.

== 2024.12.01
So, I implemented the radio firmware to work over spi, since it's a much faster connection.
However, the asercom protocol is designed to run over uart, for whatever reason.
Now, I have to remove the old spi communication (which was only implemented for sending image data, which makes sense, but could also be extended to the rest) and implement spi communication in the asercom protocol, meaning I need to initialize the spi communication (I am not keeping the previous implementation) and changin the uart (BaseSequentialStream) to spi, which also has the problem that logging will become annoying (can't just `screen /dev/ttyACM2`).

Here are the key differences of SPI vs UART according to an AI, so I can know when to use which if I event want to use UART too. I could imagine piping all the stram data through SPI and UDP, and using UART for simple 1-time communication (signals) over TCP and UART.

#table(
  columns: (1.5fr, 3fr, 3fr),
  align: (left, left, left),

  [Feature], [SPI (Serial Peripheral Interface)], [UART (Universal Asynchronous Receiver/Transmitter)],

  // Data Rows
  [Synchronization], [Synchronous], [Asynchronous],

  [Clock Signal],
  [Has a dedicated clock line (SCK), generated by the Master.],
  [No dedicated clock line. Relies on pre-agreed Baud Rate.],

  [Speed],
  [Very High (typically $10+ "Mbps"$)],
  [Lower (typically up to $approx 1 "Mbps"$, often maxing around $115200 "bps"}$)],

  [Number of Wires],
  [Minimum 4 wires for full-duplex (SCK, MOSI, MISO, SS).],
  [Minimum 2 wires for full-duplex (TX, RX).],

  [Multi-Device Support],
  [Excellent (Uses a dedicated Slave Select/Chip Select (SS/CS) line for each slave).],
  [Limited (Primarily Point-to-Point communication between two devices).],

  [Data Framing],
  [No start/stop bits or parity bits, data is a continuous stream.],
  [Uses Start bits, Stop bits, and optional Parity bits to frame the data byte.],

  [Architecture],
  [Master-Slave (One master controls the clock and selects the slave).],
  [Transmitter-Receiver (Peer-to-peer).],
)

So, we will do it that way. Since we have variable data sizes, for each spi communication (tcp$->$spi$->$stm32), the slave will first send the first byte packet (code for the asercom protocol), and the according function will read the rest of the data.

Problem: with SPI, communication is always initialized by the master processor, so to get instructions from the slave, the master has to poll it firsts.

Note: Benjamin told me the robots worked a lot better on ROS, so the real performance problem is likely due to a problem in the python library, according to him.
It might be, I'll have to test it.

Also, I should make it so the stm continuously send data, instead of the esp sending the request.

I'll do the following: remove the slave → master communication for now, since it is not really needed.
I'll pass messages from the robot through UART since it is simpler and doesn't require a lot of bandwith (the messages are small).
However, all the data readings from the robot are going to go over SPI and maybe even UDP.

== 2025.12.01
So, found out the problem why the wifi would not want to connect to the wifi.
according to https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html#esp-wifi-reason-code
and the logs, the problem is
```
NO_AP_FOUND_AUTHMODE (211):
Espressif-specific Wi-Fi reason code: NO_AP_FOUND_IN_AUTHMODE_THRESHOLD will be reported if an AP that fit identifying criteria (e.g. ssid) is found but the authmode threhsold set in the wifi_config_t is not met.
```

This suggest that the mode I put in is not supported by the `robotics_lab` router.
So I changed the `.threshold.authmode` setting to use WP2 instead of mixed wp2/wp3, which seemed to have fixed the issue.

Now, I am facing another issue with the queue that throws an error.
Here is the backtrace:
```
❯ pio pkg exec --package "espressif/toolchain-xtensa-esp32s3" -- xtensa-esp32s3-elf-addr2line -pfiaC -e .pio/build/esp32dev/firmware.elf 0x40082e7a:0x3ffcac30 0x4008b731:0x3ffcac60 0x40093f6a:0x3ffcac90 0x4008c1e1:0x3ffcadc0 0x400d23b7:0x3ffcae00
Using toolchain-xtensa-esp32s3@12.2.0+20230208 package
0x40082e7a: panic_abort at /home/luclement/.platformio/packages/framework-espidf/components/esp_system/panic.c:469
0x4008b731: esp_system_abort at /home/luclement/.platformio/packages/framework-espidf/components/esp_system/port/esp_system_chip.c:87
0x40093f6a: __assert_func at /home/luclement/.platformio/packages/framework-espidf/components/newlib/src/assert.c:80
0x4008c1e1: xQueueReceive at /home/luclement/.platformio/packages/framework-espidf/components/freertos/FreeRTOS-Kernel/queue.c:1535 (discriminator 2)
0x400d23b7: spi_request_sender at /home/luclement/Projets/fripuck2-radio/lib/serial/spi.c:86
```

== 2025.12.02
Decided I will abandon backwards-compatibility since it won't work well, and it would create a lot more work.
I decided to simply create a 64-bit bitmask that holds which sensory data is being sent.
This can be sent at the start of each packet to tell the receiver what data is sent.

== 2025.12.03
So, I changed so that the tcp packets go over UART. Now, sending packets work.

You can try an asercom command. The following commands:
```sh
printf "%b" '\xb4\x09\x01' | nc 192.168.2.147 1000
printf "%b" '\xb4\x09\x00' | nc 192.168.2.147 1000
```

Respectively enable and disable the front LED.
Note That you need to replace the IP address with the correct one.

== 2025.12.05
To watch the response of the second processor, just
```sh
screen /dev/serial/by-id/usb-STMicroelectronics_e-puck2_STM32F407_301-if00
```
This is a fixed path, it wont change (it symlinks to the correct `/dev/tty` port).

== 2025.12.08
Added the UDP task. Had a few issues where the main TCP task sent a corrupted ip address.
// TODO: diminish the stack space alocated for each task
Now it is working and I have to make it read and send the spi buffer.

To do that, I'll have a separate task that will get data from the spi connection, and place the data into a #link("https://en.wikipedia.org/wiki/Circular_buffer")[circular buffer] using the FreeRTOS `RingbufHandle_t` object.
We also have to come up with another protocol to ensure at least the order of the bytes, and to recognise what kind of data we are sending.
Each data type (to group the sensors) should have a specific preamble and tell how many elements of itself it has, and maybe headers.

Right now I am a bit overwhelmed, since there is a *lot* of possible configurations. For example, what if I wanted to read _raw_ acceleraton values from the sensor, and so on.
Technically, we can configure anything with lua, so for now I'll probably do the bare minimum, meaning I will transmit raw spi data as bytes without modification, only keeping track with a small header at the start of the packet to identify its order.

I have currently to decide what I place at the start of each packet, if anything.
I also came accross a thing called #link("https://protobuf.dev/")[protobuf], a norm from google to serialize data simply.
I might look into that to encode data onto spi, and decode it after it, but this decision shouldn't affect the esp firmware.

== 2025.12.24
Added skeleton for the report.

Also did a small attempt at using platformIO with freeRTOS for fun.
Gemini suggested generated the necessary configuration code with the stm32Cube IDE beforehand, which I did, following its instructions.

Here are the instructions:
```md
Since you are using PlatformIO, the most reliable way to get "official" files without manually scouring GitHub is to use **STM32CubeMX**. It is a visual configuration tool that generates the exact `FreeRTOSConfig.h` and clock initialization code your chip needs.

### Step 1: Install and Launch STM32CubeMX

1.  Download and install **STM32CubeMX** from the ST website (it's free).
2.  Open it and click **"Access to MCU Selector"**.
3.  Search for **STM32F407VG** and start a new project.

### Step 2: Configure Hardware and Middleware

In the left-hand column, you need to enable two things to match your current setup:

1.  **System Core -> RCC**: Set "High Speed Clock (HSE)" to **Crystal/Ceramic Resonator**. (The e-puck2/Black board has an external crystal).
2.  **System Core -> SYS**: Set "Timebase Source" to **Timer 1** (or any timer _except_ SysTick).

    > _Why?_ FreeRTOS takes over SysTick. The HAL needs its own timer to handle `HAL_Delay()` and timeouts without crashing into the OS.

3.  **Middleware -> FREERTOS**: Set "Interface" to **CMSIS\_V2** (this is the modern standard for STM32).

### Step 3: Set the Clock (The 168MHz Secret)

Go to the **Clock Configuration** tab.

1.  Input **8** MHz in the Input Frequency (HSE) box (standard for these boards).
2.  In the "System Clock Mux," select **PLLCLK**.
3.  In the "HCLK" box, type **168** and hit Enter. The tool will automatically calculate all the multipliers (PLL) for you.

### Step 4: Generate the Code

1.  Go to the **Project Manager** tab.
2.  **Project Name:** Call it "TempProject".
3.  **Toolchain/IDE:** Select **Makefile**. (This ensures the code is generated in a clean C structure that is easy to copy into PlatformIO).
4.  Click **"Generate Code"** in the top right.
* * *

### Step 5: Porting to PlatformIO

Once generated, open the folder. You don't need the whole project; you just need to "steal" a few specific pieces:

1\. The Config File:

Find Core/Inc/FreeRTOSConfig.h. Copy this into your PlatformIO /include folder.

2\. The Clock Setup:

Look in Core/Src/main.c. Find the function void SystemClock\_Config(void). It will look like a long list of structures. Copy that entire function and replace the empty one in your PlatformIO main.c.

3\. The HAL Timer:

Because you told CubeMX to use Timer 1 for the HAL, it generated a file called stm32f4xx\_hal\_timebase\_tim.c.

-   Copy this file into your PlatformIO `/src` folder.
-   This ensures `HAL_Delay()` works correctly alongside FreeRTOS.

### Summary Checklist

-   \[ \] `FreeRTOSConfig.h` is in `/include`.
-   \[ \] `SystemClock_Config` is updated in `main.c`.
-   \[ \] `stm32f4xx_hal_timebase_tim.c` is in `/src`.
-   \[ \] `lib_deps = FreeRTOS` is in `platformio.ini`.
**Would you like me to show you how to check the e-puck2's old `mcuconf.h` to make sure the 168MHz clock settings match exactly what the robot expects?**
```

Here is a picture of the configuration:
#image("assets/image.png")
It looks horible.

The hard part is to configure the connectivity of the chip, since it has a lot of peripherals.
GCtronic provides a schematic of the connections:
#image("assets/epuck-schematic.png")

I've decided that I'm not going to configure the connectivity right now, and am going to do that later, once I'm sure freeRTOS and lua can be included easily with PIO's dependency management.

Also, the instructions of the AI to make FreeRTOS work are a bit iffy.
I am basing myself of the structure of the example project at https://github.com/maxgerhardt/pio-stm32h7-stm32cube-freertos/tree/main.
This post also seems to be quite important: https://community.platformio.org/t/platformio-stm32-and-freertos-library/44418/5.

Finally, I got it working (as a test of course), so it is technically possible to implement it also inside of PlatformIO and with FreeRTOS (meaning both `radio` and `controller` processes would use the same OS, which could simplify future development).

Now, this was done for fun. I believe that this is a lot more approachable than the legacy build system, but I have to evaluate the amount of work needed to see if worth doing the port (since I would still have to add back all the sensors and their management).

== 2026.02.18
I have started (and hopefully finished) transitionning the pin definitions from ChibiOS to FreeRTOS.
This was done by checkin the board definition (at `ChibiOS_ext/os/hal/boards/epuck2/cfg/board.chcfg`) and putting the values inside STM32CubeMX to generate the basic code.
It produced conflicts, but since the hardware can't be changed, I won't attempt to fix them right now.

I now have an issue that the `/dev/serial` virtual file system isn't getting shown.
This is probably a permission issue on my system, since I recently changed it.
To solve it, I had to add myself to the `uucp`.
Now it works.

Code was refactored into `harware-init.c/h` and `gpio-pins.h` to make the `main.c` file more approachable.
A small test was made to see if the body LED would blink, which worked, giving me the confidence to move forward.

Now, we need to implement all the sensors individually.
I am going to place vendor code in their respective `lib/` folders, and then adapt them in their own section in a `src/sensors` folder.

The e-puck consists of the following sensors:
- Camera: PixelPlus PO8030D CMOS image sensor, datasheet, no IR cut filter
  - From about July 2019, the camera mounted on the e-puck2 robot is the Omnivision OV7670 CMOS image sensor, datasheet
- Microphones: STM-MP45DT02, datasheet
- Optical sensors: Vishay Semiconductors Reflective Optical Sensor, datasheet
- ToF distance sensor: STM-VL53L0X, datasheet, user-manual
- IMU: InvenSense MPU-9250, product-specification, register-map
- Motors: details
- Speaker: Diameter 13mm, power 500mW, 8 Ohm, DS-1389 or PSR12N08AK or similar
- IR receiver: TSOP36230

== 2026.02.19
Moved the Core files (hardware initialisation) into its own `lib/` folder.
Now I am trying to fix some timing issues.
First I've changed the clock configuration in STM32CubeMX to match what is inside the `board.chcfg` file.
Then, AI told me to change the Hardware (HAL) timer to TIM6 (or anything other than TIM1, which apparently is special).
Then it told me to lower the TIM6 priority to 0, so it has a higher priority than the OS tick, which apparently causes some issues.

Everything was done to try and fix an issue where the `HAL_GetTick()` function did not work correctly and always returns `0` (which is not correct).
After trying, the FreeRTOS delay works fine, so it's definitely a problem with the HAL configuration.

In the end, the issue seems to be that not the correct functions were invoked since PIO wasn't compiling the correct c files in the Core lib.
The solution was to simply dump everything in a `core` folder in src for the autogenerated files, then renaming the `core/main.c` to `hardware_init.c`, creating a header file that exposes the initialisation functions, removing the static declarations to fix inclusion errors, and then stripping the default taks and the `main()` function into a separate `src/main.c` file as the entry point to keep the file concise.

This way, all the essential functions are overwritten and the robot can initialize correctly.

#line(length: 100%)

Now, the difficult part is to do the porting.
I started with the LEDS since they are the easiest.
They are just set through pins.
I only did the normal (red) LEDS since the RGB LEDS are managed by the esp chip.
Since all user commands come from that chip, I don't see why I should spend time implementing the RGB LEDS inside the stm chip.

Now, let's stay on the same line in the graph:
#image("assets/epuck-schematic.png")
You can see that the other component connected directly through pins are the motors.
These are quite complicated.

So here is my understanding:
- So, from my understanding it works like this:
- The motor is a step motor.
- To tell the motor what to do, we have to set multiple pins in a certain fashion (that's what the step_table is for?).
- For the motor to move continously (and not do a single step, or micro-step), we have to send these commands continuously.
- For this, in ChibiOS, we have a pulse width modulation which send calls a callback each [defined interval].
- This callback then executes the next step for the motor (or halt if none).
- In freertos, we would have to use a precise interrupt ad a specific interval that would do that.
- I suppose we can't just use freertos tasks with dimple Delay(interval) in them, because they would be *imprecise*

So, to tell the motor to move, we have to continously send instructions through the pins:
```c
static const uint8_t step_table[8][4] = {
  {1, 0, 1, 0},
	{0, 0, 1, 0},
  {0, 1, 1, 0},
	{0, 1, 0, 0},
  {0, 1, 0, 1},
	{0, 0, 0, 1},
  {1, 0, 0, 1},
	{1, 0, 0, 0},
};

static const uint8_t step_halt[4] = {0, 0, 0, 0};
```
This `step_table` shows all the steps the motor has to receive to move.
At each index, we have to set the pins accordingly (set to 1 if there is a 1, 0 else).
To go one direction, we _increase_ the index (determines the order in which we send the instruction), and to go in the other, we _decrease_ the index.
The `step_halt` table is simply the steps to make the robot stop.
To make a step, we simply aply the table to the correct pins (on/off state).

The speed of the motors is then managed by the number of times per second the updates are performed.
This means we need two additional functions: one that updates every `interval` µs, and one that can update that speed.
The first of the two functions is going to be an interrupt, that runs each time a hardware timer `TIMX` runs out. We will need 2 timers since we have two motors that can have different speeds.
The other function will simply have to change the interval the two interrupts wait for.
For simplicity's sake, we'll use `TIM3` for the right motor and `TIM4` for the left one.

=== Configuring the timers
First, not all timers are equal, each of them is linked to a timer clock, either `APB1` or `APB2`.
#image("/assets/image.png")
- TIM2, TIM3, TIM4, TIM5 are usually on APB1.
- TIM1, TIM8, TIM9, TIM10, TIM11 are usually on APB2.

Since we use `TIM3` and `TIME4`, we are working with clocks running on 84 Mhz.

I will configure them inside CubeMX.
I can select `TIMX` and change its Clock Source from 'Disable' to 'Internal Clock'.
Then, more configuration options appear.
- Prescaler: "A prescaler is an electronic circuit that reduces a high frequency signal to a lower frequency by dividing it, allowing timers to operate at desired rates."
  For simplicity, we'll want to work with a 1µs delay, or 1Mhz clock.
  - This means we need to set the prescaler to 83 (since $(84 "MHz")/(83 + 1) = 1 "MHz"$).
  - Then, we can set the ARR (couter period) to 999, to get the 1 KHz that is listed inside the old code: ```c #define MOTOR_TIMER_FREQ 100000 // [Hz]```
  - Finally, we need to enable the "auto-reload preload" option. This ensures that when we change the ARR on the fly (which we need to modify the speed of the wheels), the counter first finishes counting to 1000, which can prevent glitches.
- Motors: 2 stepper motors with a 50:1 reduction gear; 20 steps per revolution; about 0.13 mm resolution , with at most 1200 steps/s, Wheels diamater = 41 mm
  - The 50:1 gear reduction makes the wheels smoother (else the wheel would move 18° by step).
  - One full wheel rotation is 1'000 pulses $20 "steps" times 50 "reduction"$.
  - This means that since we have a 41 mm wheel, we have a circumference of $C = pi dot d = 128,805298797 "mm"$ and thus need $1000/C$ steps to advance 1 mm,

The logic to change the motor registers is quite straight forward.
The complicated part (since we are not familiar with HAL) is the TIM callback.
Here is how we register a callback:
```c
HAL_StatusTypeDef HAL_TIM_RegisterCallback(TIM_HandleTypeDef *htim, HAL_TIM_CallbackIDTypeDef CallbackID, pTIM_CallbackTypeDef pCallback)
```

`htim` is simply the handle to the timer.
The second parameter is a CallBack ID, which points to which the interrupt source calls the given CallBack

== 2026.02.21
Starting to work on the IR sensors (analog).
Looking at the old code and what is said online, we use adc (Analog Digital Converter).
Since we have only 8 ir sensors, I will put all of them on ADC1 instead of annoying myself by using two different ADCs.
The additional time needed when using only one channel is probably negligeable considered we have to wait for the response time of the ir sensors themselves. #link("https://projects.gctronic.com/epuck2/doc/tcrt1000.pdf", [link to the datasheet])

Here is what the all mighty google gemini has to say about the adc1 configuration:
#quote(block: true, attribution: [Google Gemini])[
  - Mode: Independent Mode.
  - Scan Conversion Mode: Enabled (This tells the ADC to move from one pin to the next).
  - Continuous Conversion Mode: Enabled (This tells it to start over at Pin 0 after finishing Pin 7).
  - DMA Continuous Requests: Enabled (Crucial: this keeps the DMA engine fed).
  - Number of Conversions: 8.
  - External Trigger Conversion Source: Regular Conversion launched by software.
  - Rank Table: Assign your 8 channels to Ranks 1 through 8.
  - Warning: Ensure the Channel numbers in the Ranks match your PROXIMITY_0 to PROXIMITY_7 physical wiring order.
]

I also need to reorder the rank of the ir pins.
According to the channel (in the gpio name, after the IN) in #image("assets/Copie d'écran_20260222_114204.png")
we need to reorder the rank so the numbering on the labels get read in order (so we know which ones are updated when.)


== 2026.03.01
Finished IR recievers last week. Will need to do some testing, especially since the old code does some smart trickery to do readings with the IR recievers off to remove the ambien light factor.

Right now working on UART (USART) transmission. From my understanding there are three transmission modes: blocking, interrupts or DMA.
blocking is the simples, where we call `HAL_USART_Receive` and we block the program until we receive some amount of bytes.
Then, using interrupts the CPU is interrupted for each byte (or the size we get) to process the bytes and store them.
Finally, DMA mode is where in the background the data is directly copied into memory and we interrupt when we have moved the entire data packet size.

Since we will be sending variable length packets we can also use idle detection from what I can see online.
DMA copies data automatically into the buffer, and we can have callbacks when we either get half or full data, and we can use IDLE detection when no data is being sent (finished sending).
We can then process data either when a full sized packet has finished sending, or when a smaller packet has finished and we hit idle.
