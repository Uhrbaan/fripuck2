#show link: it => text(fill: blue, underline(it))

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
#image("assets/tim-conf.png")
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


== 2026.03.10
Been working for quite a wile on UART. A simple test has been set up in the radio module and seems to be working fine, but I am always met with an overrun error `ORE` on the controller side.
To my understanding, UART works by placing one word (byte, which is the size of a word in our case) into a register, and the second processor should "remove" it from the register and consume it.
An ORE error happens when that doesn't happen quickly enough and byte hasn't been removed when the first process wants to place the second byte.

In the hopes of fixing the issue, I am going to modify the UART reception to follow the #link("https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx")[following guide].
It suggests that for bauds of 115200, which we are using, we should use Interrupt mode (no DMA).

After a lot of trial and error, I finally have something that works with idle line detection. Here is the `main.c` that works:

```c
#include "main.h"

#include "core/driver.h"
#include "core/can.h"
#include "core/gpio.h"
#include "core/tim.h"
#include "core/usart.h"
#include "core/dma.h"

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#include "leds.h"
#include "motors.h"
#include "uart.h"

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument)
{
    while (1)
    {
        osDelay(pdMS_TO_TICKS(5000));
    }
}

int init_hardware(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    MX_CAN1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    MX_DMA_Init();

    return 0;
}

#define uart_data_size 128
static uint16_t uart_data_pointer = 0;
static uint8_t uart_data[uart_data_size] = {0};

void buffer_is_full(uint16_t size)
{
    (void)uart_data;
    (void)size;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        toggle_led(LED_1);
        buffer_is_full(Size);

        // 'Size' is the number of bytes received until the IDLE event occurred.
        // You can now process 'uart_data' directly as a full string!

        // IMPORTANT: You must RE-START the listener immediately
        HAL_UARTEx_ReceiveToIdle_IT(huart, uart_data, uart_data_size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        uint32_t err = HAL_UART_GetError(huart);

        if (err & HAL_UART_ERROR_FE)
        {
            // Frame Error detected
            toggle_led(LED_7); // Error LED
        }

        if (err & HAL_UART_ERROR_ORE)
        {
            // Overrun Error detected
            __HAL_UART_CLEAR_OREFLAG(huart);
        }

        // The peripheral is now in an error state and stopped.
        // We MUST clear the error and restart the listener.
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UARTEx_ReceiveToIdle_IT(huart, uart_data, uart_data_size);
    }
}
int main(void)
{
    init_hardware();
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart_data, uart_data_size); // motors_init(htim3, htim4);
    // uart_init(&huart3);
    // uart_register_receive_callback(uart_action);

    // // 1. Start the UART DMA reception
    // // This should be called in your main() or init function
    // HAL_StatusTypeDef status = HAL_OK;
    // status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_BUF_SIZE);
    // __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); // Optional: Disable Half-Transfer interrupt if not needed

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    osKernelStart();
    while (1)
    {
        osDelay(pdMS_TO_TICKS(5000));
    }
}
```

I don't really understand what was not working before that. One issue could be that by mistake I ```c osKernelInitialize()``` in the ```c init_hardware()``` even if it should be called just before creating and starting the initial task. This could have introduced som undefined behavior.
Secondly, I seemed to have forgotten to set the baud rate of the second chip (esp) to 115200, which produced Frame errors (`4`).

Now I will move to SPI.
Here is what SPI is:
#quote(block: true, attribution: link(
  "https://www.totalphase.com/blog/2016/06/spi-vs-uart-similarities-differences/",
)[Total Phase])[
  SPI (Serial Peripheral Interface) is a serial communication protocol originally developed by Motorola that enables communication between nearly any electronic device that supports clocked serial streams. SPI uses a master-slave method for communication that enables high-speed data streaming.

  As opposed to just using two wires, SPI must use at least 4 wires. As there can be multiple slave devices in an SPI implementation, the actual amount of wires or traces used will be n+3 where n = the number of slave devices in use.

  A few additional key details on SPI before we move on:
  - SPI is synchronous
  - SPI is a simple protocol with little overhead
  - SPI supports full-duplex communication
  - SPI communication does not have a means for acknowledgment or flow control
  - SPI does not use much board space
]

#figure(image("assets/image.png"), caption: [
  Overall communication schema of the epuck2.
  https://www.gctronic.com/doc/index.php?title=e-puck2
])<fig-communication-schema>

Chatting with an AI and looking at the diagram in @fig-communication-schema we see that the microphones 1-4 are connected over a different protocol sister to SPI named I2S -- specifically the connections I2S2 and I2S3.

#image("assets/image-1.png")
Currently, we have too many errors. I will focus on the SPI1 connection which is connected to the radio module and the encoders.
The encoders are simply a sensor which detect how much the wheels have turned.
This can be useful for example to see if the robot is stuck: the step couter will go up, while the encoders will stay the same.
If there is a difference, then the robot is stuck.
The data from the encoders can also be used for odometry, since it is precise.

The difficulty of the SPI1 connection is that we have two very different use cases for it.
On one hand, we have the main use which is high-speed transfer from the STM to the ESP, and on the other hand we have the very low speed encoders which we need to read.
The issue is that first, we have to chip select the correct line (which we have three of: `SPI1_CS_ENC_L_Pin, SPI1_CS_ENC_L_Pin, SPI1_CS_ESP32_Pin`), and we still have to optimise for sending packets, while still being able to receive the the encoder data.

One option would be to not use the encoders. Right now in the old code does not use it so we would not loose any information if we don't either.
Still I think it should be supported.
Another difficulty is that I want to allow audio streaming. This means that we also have to be able to recieve high-speed data from the ESP chip.
This however is for later. It is currently not supported by the epuck and thus isn't our priority.

== 2026.03.11
I finished the first attempt at implementing SPI1.
Right now I am also growing tired of having no logging mechanism. I will look into that.

// What seems the most straight forward option would be to use the uart3 connection to log things to the esp chip, which will then log it amongs his own logs on behalf of the stm chip.

// However, this has the quite important issue that logging will inevitably introduce timing issues.

== 2026.03.12
Slightly changed the spi and uart apis on the radio chip to match the programming style of the controller chip a little more.

Testing the throughput of the SPI connection from the STM to the ESP chip with the following AI-generated code:

```c
// Keep track of total bytes received
volatile uint32_t total_bytes_received = 0;

void spi_recieve_cb(uint8_t *data, uint16_t length) {
    // Process your data here...

    // Accumulate the length
    total_bytes_received += length;
}

void throughput_monitor_task(void *pvParameters) {
    uint32_t last_bytes = 0;
    const uint32_t interval_ms = 2000; // 2 seconds

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(interval_ms));

        // Get current snapshot and calculate delta
        uint32_t current_bytes = total_bytes_received;
        uint32_t delta = current_bytes - last_bytes;
        last_bytes = current_bytes;

        // Calculate Throughput (Bytes per second)
        float throughput_bps = (float)delta / (interval_ms / 1000.0);

        // Print in a human-readable format
        if (throughput_bps < 1024) {
            printf("Throughput: %.2f B/s\n", throughput_bps);
        } else if (throughput_bps < (1024 * 1024)) {
            printf("Throughput: %.2f KB/s\n", throughput_bps / 1024.0);
        } else {
            printf("Throughput: %.2f MB/s\n", throughput_bps / (1024.0 * 1024.0));
        }
    }
}
```

The results of this are:
```
Throughput: 32.23 KB/s
```
pretty consistently.

This is prett low.
Looking into it, I think it could be a few issues:
- Overhead because I'm only sending 33 bytes at a time for testing
- To low of a clock speed (which I don't think because it is set to a baud rate of \~2Mb/s)
- There is some delay generated while sending
- There is some delay generated when recieving.

Trying to tacle the first issue, I simply increased the message to be a 1Kb block. This caused a segmentation fault.
After debugging, I realized that I went over the allocated stack limit of the task I was runing it in.
To solve it, I simply needed to declare the payload as ```c static```.

Now, I was getting along the lines of
```
Throughput: 315.43 KB/s
Throughput: 311.49 KB/s
Throughput: 314.93 KB/s
Throughput: 315.42 KB/s
```
Which makes me happy. I still think we could go a little higher but it is already a 10x improvement.

Doubling the packet size at that point only makes the throughput go up to 317 Kb/s, so the 1Kb size seems to be the sweetspot.

Currently the code is using a blocking ```c HAL_SPI_Transmit()``` call. This is pretty wasteful, and looking online it is recommended to use DMA when transfering large packets.

The obvious next step is thus do do transmission over DMA.

Well, it turns out the obvious solution isn't an easy one.
I haven't managed to enable DMA because of a wierd issue where ```c HAL_SPI_TxCpltCallback()``` never gets called, which would never release the semaphore waiting for DMA to complete, and soft-lock the robot.

The other easiest change we can do to change the speed is to change the prescaler.
With a prescaler of 32, we have the 315 Kb/s we had before, and with 16 we have 620 Kb/s and finally with a prescaler of 8 we have 1.18 Mb/s.

However I am going to keep the prescaler at 32 for now since it is less likely to generate noise than a lower prescaler. I might increase it later if necessary.

== 2026.03.14
It is time to move to transmit the spi packets over UDP.
I've started by setting the maximum size of the SPI packet to 1400 bytes. Typically, we set the maximum data size of UDP/TCP packets to 1452 since that is the maximum size allowed over UDP IPv6.
Also, testing showed that going over the 1K limit of the SPI packet doesn't improve throughput that much, so setting a 1400 byte limit doesn't seem too much of a burden.
This will prevent fragmentation wich will hopefully increase the throughput over wifi.

We also have to decide wether to add something to the UDP packets, like order.
Since our application is purely real-time, this isn't really necessary. The data is independent, high frequency, real-time and will probably only be analyzed over time (trend analysis), so it is pretty safe to assume that we won't car if a packet is lost or not in order, as long as its timestamp is correct.

For the timestamps, we need to pay attention two things: we shouldn't use up too much space to save bandwith, and we should also make sure that the reciever won't break if the packets arrive out-of-order or a packet is lost.

To do that, I will probably use a 16-bit millisecond counder. It takes up 2 bytes, and will wrap around every minute or so (65'535 milliseconds \~ 65 seconds).

We also need to think about how we will package our packets so they can be serialized and un-serialized very easily across all components/languages.
This could be done in a few ways -- I could use JSON or similar, which would be inneficient, I could use simple C structs and use C-compatibility APIs in other languages, but that wouln't be very portable and hard to maintain.

An alternative would be to use common serialization libraries. Protobuf seems to be popular, but it doesn't really seem fit for embedded use since it can be quite heavy on computation, and the alternative, also developed by Google is #link("https://flatbuffers.dev/", "FlatBuffers").

With flatbuffer we can define all the structures in a `.fbs` schema (docs #link("https://flatbuffers.dev/schema/", "here")) and then autogenerate the code to read and encode the data with `flatc`, which produces output in all the languages we want: C, Lua (for the embedded side), Go and Python (for the remote side).

Doing a first test with a simple dummy schema:
```fbs
table Dummy {
  timestamp: uint16;
  data: ubyte;
}

root_type Packet;
```

Then we can #link("https://flatbuffers.dev/quick_start/")[install] and run the `flatc` compiler to generate the C definitions inside of the `flatbuffers` library inside the shared `common` libraries.
Note that while C is supported by flatbuffers, it comes with its own compiler `flatcc` for #link("https://flatbuffers.dev/languages/c/")[technical reasons]. #link("https://github.com/dvidelabs/flatcc")[Here] is its repository for installation.

```sh
flatcc -a packets.fbs -o firmware/common/
```

For it to compile, we will also need the flatcc header files. Let's copy them from the repository we downloaded to the common include folder.

```sh
cp /path/to/flatcc/include/flatcc firmware/common -r
```

We also need platformIO to use our flatbuffer library, so we need to add `-D FLATC_PORTABLE` to our `platformio.ini` configuration file.

While configuring, I came back to errors related to the project structure. I will try to clean up how the shared definitions are structured in the codebase.

Now, the shared definitions have been moved into a correct library with a `library.json` definition. The added `.ini` lines look like this:
```ini
lib_deps =
  flatbuffers=symlink://../shared/flatbuffers
  communication-config=symlink://../shared/communication
```

This is a lot more cleaner. To complete the flatbuffers library, I added the include directory of the `flatcc` project to the `include` folder of the library, alongside the generated files. To export the definition, I also exported the src files into the thing.

Commit `a056e6e` has a test dummy object to test the transmission of dummy data as a flatbuffer.

I still have to get familiar with the flatbuffers syntax and ways of thinking, but it seems to be working.
I also now have to test sending it over UDP and opening it in another language.

So here is an example of what our schema could look like:
```fbs
namespace MyProject;

// MPU data is always fixed size, so we use a 'struct'
// for zero-overhead inside the table.
struct MpuData {
  accel_x: float;
  accel_y: float;
  accel_z: float;
  gyro_x: float;
  gyro_y: float;
  gyro_z: float;
}

table MpuReading {
  sensor: MpuData;
}

table AudioReading {
  samples: [int16]; // Variable length buffer
}

table CameraFragment {
  data: [ubyte];    // The actual image bytes
}

// The Union that allows us to distinguish the packet types
union SensorData { MpuReading, AudioReading, CameraFragment }

table Entry {
  timestamp: uint32;
  packet: SensorData;
}

table SensorBatch {
  entries: [Entry];
}

root_type SensorBatch;
```

We would directly use flatbuffers to serialize all of the different stuff together.
We would sequentially add Entries to the SensorBatch until we can't fit an element that would make us go over the 1400 byte limit.

Here is the conversation I had with the AI: https://gemini.google.com/share/4f0d14114464

Then, recieving and "parsing" the data can be quite simple:
```c
#include "packets_reader.h"

void process_received_spi_packet(uint8_t *buffer, size_t size) {
    // 1. Get the root of the buffer
    MyProject_SensorBatch_table_t batch = MyProject_SensorBatch_as_root(buffer);
    if (!batch) return;

    // 2. Access the vector of entries
    MyProject_Entry_vec_t entries = MyProject_SensorBatch_entries(batch);
    size_t num_entries = MyProject_Entry_vec_len(entries);

    for (size_t i = 0; i < num_entries; i++) {
        // 3. Get the individual entry
        MyProject_Entry_table_t entry = MyProject_Entry_vec_at(entries, i);

        // 4. Read the shared timestamp
        uint32_t ts = MyProject_Entry_timestamp(entry);

        // 5. Check the UNION type and cast accordingly
        switch (MyProject_Entry_packet_type(entry)) {

            case MyProject_SensorData_MpuReading: {
                MyProject_MpuReading_table_t mpu = (MyProject_MpuReading_table_t)MyProject_Entry_packet(entry);
                MyProject_MpuData_struct_t data = MyProject_MpuReading_sensor(mpu);
                printf("TS: %lu | MPU: ax=%f, gx=%f\n", ts, data->accel_x, data->gyro_x);
                break;
            }

            case MyProject_SensorData_AudioReading: {
                MyProject_AudioReading_table_t audio = (MyProject_AudioReading_table_t)MyProject_Entry_packet(entry);
                flatbuffers_int16_vec_t samples = MyProject_AudioReading_samples(audio);
                size_t len = flatbuffers_int16_vec_len(samples);
                printf("TS: %lu | Audio: %zu samples received\n", ts, len);
                break;
            }

            case MyProject_SensorData_CameraFragment: {
                MyProject_CameraFragment_table_t cam = (MyProject_CameraFragment_table_t)MyProject_Entry_packet(entry);
                flatbuffers_uint8_vec_t img_data = MyProject_CameraFragment_data(cam);
                size_t img_len = flatbuffers_uint8_vec_len(img_data);
                printf("TS: %lu | Camera: %zu bytes fragment\n", ts, img_len);
                break;
            }

            default:
                printf("Unknown packet type!\n");
                break;
        }
    }
}
```

And it would work across multiple languages too !

Python:
```py
import MyProject.SensorBatch as SensorBatch
import MyProject.SensorData as SensorData

def process_spi_data(raw_bytes):
    # 1. Map the root
    batch = SensorBatch.SensorBatch.GetRootAsSensorBatch(raw_bytes, 0)

    # 2. Iterate through entries
    for i in range(batch.EntriesLength()):
        entry = batch.Entries(i)
        ts = entry.Timestamp()

        # 3. Check Union Type
        union_type = entry.PacketType()

        if union_type == SensorData.SensorData().MpuReading:
            mpu_table = entry.Packet()
            # Cast the generic packet to the specific MpuReading
            from MyProject.MpuReading import MpuReading
            mpu = MpuReading()
            mpu.Init(mpu_table.Bytes, mpu_table.Pos)

            sensor = mpu.Sensor()
            print(f"TS: {ts} | MPU: {sensor.AccelX()}")

        elif union_type == SensorData.SensorData().AudioReading:
            # Handle Audio...
            pass
```

Go:
```go
import (
	"fmt"
	"MyProject"
)

func processSpiData(buf []byte) {
	// 1. Get the root
	batch := MyProject.GetRootAsSensorBatch(buf, 0)

	// 2. Loop through entries
	for i := 0; i < batch.EntriesLength(); i++ {
		entry := new(MyProject.Entry)
		if batch.Entries(entry, i) {

			ts := entry.Timestamp()

			// 3. Handle Union via Switch
			unionTable := new(flatbuffers.Table)
			if entry.Packet(unionTable) {

				switch entry.PacketType() {
				case MyProject.SensorDataMpuReading:
					mpu := new(MyProject.MpuReading)
					mpu.Init(unionTable.Bytes, unionTable.Pos)
					s := mpu.Sensor(nil)
					fmt.Printf("TS: %d | MPU X: %f\n", ts, s.AccelX())

				case MyProject.SensorDataAudioReading:
					// Handle Audio...
				}
			}
		}
	}
}
```

Note that this code is probably not valid since the AI had a lot of trouble to generate valid FlatBuffers code, but it gives an idea of how simple it will be, and easy to implement in other languages.

== 2026.03.15
Now that the initial testing works, let's create two schemas. `telemetry.fbs` will contain data generated by the stm or esp, and `commands.fps` will serve to decode the instructions sent from the APIs.
To generate them, we can run
```sh
flatcc -a commands.fbs telemetry.fbs -o firmware/shared/flatcc-generated
```

To simulare real worload, I created \~ 1Kb of InfoMessages, batched them together and sent them over SPI.
Here is the necessary code to do this:

Here is the sender (STM side):
```c
void TelemetryTask(void *argument)
{
    static flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    // Array to store references until we are ready to finish the batch
    flatbuffers_ref_t entry_refs[MAX_ENTRIES_PER_BATCH];
    size_t entry_count = 0;

    while (1)
    {
        // --- 1. Create your Message ---
        // We create the table but don't add it to a batch yet
        char msg_text[32];
        snprintf(msg_text, sizeof(msg_text), "Tick: %lu", HAL_GetTick());

        // Create the string and the InfoMessage table
        flatbuffers_string_ref_t f_str = flatbuffers_string_create(&builder, msg_text, strlen(msg_text));
        Fripuck2_Telemetry_InfoMessage_start(&builder);
        Fripuck2_Telemetry_InfoMessage_text_add(&builder, f_str);
        flatbuffers_ref_t msg_ref = Fripuck2_Telemetry_InfoMessage_end(&builder);

        // Create the Entry wrapper
        Fripuck2_Telemetry_Entry_start(&builder);
        Fripuck2_Telemetry_Entry_timestamp_add(&builder, (uint16_t)HAL_GetTick());
        Fripuck2_Telemetry_Entry_content_add(&builder, Fripuck2_Telemetry_Data_as_InfoMessage(msg_ref));
        flatbuffers_ref_t current_entry = Fripuck2_Telemetry_Entry_end(&builder);

        // --- 2. Store the Entry ---
        entry_refs[entry_count++] = current_entry;

        // --- 3. Check Size and Send if needed ---
        size_t current_size = flatcc_builder_get_buffer_size(&builder);

        if (current_size >= BATCH_THRESHOLD || entry_count >= MAX_ENTRIES_PER_BATCH)
        {

            // Finalize the Batch Table
            Fripuck2_Telemetry_Batch_start_as_root(&builder);
            Fripuck2_Telemetry_Batch_entries_create(&builder, entry_refs, entry_count);
            Fripuck2_Telemetry_Batch_end_as_root(&builder);

            // Get the final buffer
            size_t final_size;
            void *buf = flatcc_builder_get_direct_buffer(&builder, &final_size);

            if (buf && final_size <= RADIO_MAX_PACKET_SIZE)
            {
                spi_radio_send((uint8_t *)buf, (uint16_t)final_size);
            }
            else if (final_size > RADIO_MAX_PACKET_SIZE)
            {
                // Error: Even with threshold, we went over 1KB (likely a very long string)
            }

            // --- 4. Reset for next batch ---
            flatcc_builder_reset(&builder);
            entry_count = 0;
        }

        // osDelay(50); // Small delay to allow messages to accumulate
    }

    flatcc_builder_clear(&builder);
}
```

And here is the reciever:
```c
void spi_recieve_cb(uint8_t *data, uint16_t length) {
    // Process your data here...
    if (Fripuck2_Telemetry_Batch_verify_as_root(data, length) != 0)
        return;

    Fripuck2_Telemetry_Batch_table_t batch = Fripuck2_Telemetry_Batch_as_root(data);
    Fripuck2_Telemetry_Entry_vec_t entries = Fripuck2_Telemetry_Batch_entries(batch);
    size_t n_entries = Fripuck2_Telemetry_Entry_vec_len(entries);

    for (int i = 0; i < n_entries; i++) {
        Fripuck2_Telemetry_Entry_table_t entry = Fripuck2_Telemetry_Entry_vec_at(entries, i);
        uint32_t timestamp = Fripuck2_Telemetry_Entry_timestamp(entry);
        Fripuck2_Telemetry_Data_union_type_t type = Fripuck2_Telemetry_Entry_content_type(entry);

        switch (type) {
        case Fripuck2_Telemetry_Data_InfoMessage:
            Fripuck2_Telemetry_InfoMessage_table_t message =
                (Fripuck2_Telemetry_InfoMessage_table_t)Fripuck2_Telemetry_Entry_content(entry);
            flatbuffers_string_t text = Fripuck2_Telemetry_InfoMessage_text(message);
            ESP_LOGI("SPI RECIEVE FB", "(%d) Batch element %d contains text: %s", timestamp, i, text);
            break;

        default:
            ESP_LOGI("SPI RECIEVE FB", "(%d) Cannot read type of Batch element %d", timestamp, i);
            break;
        }
    }
    // Accumulate the length
    total_bytes_received += length;
}
```

Note that the logging was disabled for testing since it would absolutely destroy the throughput.
We roughly get a throughput of \~250Kb/s for a prescaler of 32, which is reasonable at 80% of the original speed.
Chaning the prescaler to 16 we get \~410KB/s

While it still is a reasonable speed, we clearly see that we get further from the previous \~ 620Kb/s we had at the same prescaler.

Note however that the code is not optimal. First, we can elminiate slow functions like `snprintf`, we can make the individual packets a little bigger so we call flatbuffers less often, and we could optimize our schema to use structs when there is static data.

Simple changing the text being sent to a fixed string like the first sentence of the Lorem Ipsum already increases the throughput to \~290Kb.

The next step would be to send these same packets over the network and decode them there.

First I have to enable the wifi again on the ESP.
I already implemented it a while ago so it shouldn't take long. Now, I need to create an external program that will connect to the UDP stream.

== 2026.03.19
Working on (semi) reliable UDP connection. Had an issue for a long while where the packets would go through the ESP and get corrupted.
The issue was probably that since the esp was sending the packet in a blocking manner after recieving the data through SPI, the SPI would overwrite the buffer which would cause corruption.
It essentially was a timing error. It was fixed by creating a fast send function that pushed the data directly onto a queue and went back to listening.

Initially, the STM had to be slowed down to fix the timing issues, and we had \~80Kbps of useful data.
Now with the improvement, we get \~220Kbps of useful data ($times 2$ improvement).
The final goal for today is to identify the bottleneck.

While I thought last time that the SPI connection wasn't the bottleneck, changing the prescaler from $32 -> 16$ nearly doubled the throughput to 400Kbps, while nearly not affecting the amount of dropped (corrupted) packets which remain at roughly 2%.

Again, halving the prescaler from $16 -> 8$ increased the throughput, from 220Kbps to 660Kbps. However this time the dropped (corrupted) packets increased to 6%. The ESP chip also produces a lot of `E (164644) UDP TRANSMITTER: Error while sending data: Not enough space (12).` errors, indicating that the ESP has reached its limits.

This is quite nice to see !
