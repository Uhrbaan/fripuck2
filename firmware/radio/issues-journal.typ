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
