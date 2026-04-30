For later:

2.  What are these files?

Since you are migrating from the e-puck2 project, these files are highly
specific to its functionality. stm32f407xG.ld (Linker Script)

What it is: This file tells the compiler exactly where the RAM and Flash memory
start and end on your specific chip.

Should you care? No, not yet. PlatformIO automatically generates a standard
linker script for the black_f407vg. Unless the e-puck2 project used "custom
memory sections" (like special storage for a bootloader), the default one
provided by PlatformIO will work fine.

libarm_cortexM4lf_math.a (CMSIS-DSP Library)

What it is: This is the compiled ARM CMSIS-DSP library. It contains highly
optimized functions for digital signal processing (Fast Fourier Transforms,
complex math, filters).

Should you care? Yes. If you plan on doing sensor fusion or audio processing
(like the e-puck2 does), you will need this. However, instead of manually
dragging this .a file, you can simply add lib_deps = CMSIS-DSP or similar to
your platformio.ini.

libPDMFilter_CM4F_GCC.a (Pulse Density Modulation Filter)

What it is: This is a closed-source library from STMicroelectronics used to
convert signals from digital MEMS microphones (PDM) into standard audio signals
(PCM).

Should you care? Only if you use the microphones. The e-puck2 has several
onboard microphones. If your "scratch" project needs to "hear" sound, you will
need to link this specific library in PlatformIO.

3.  How to include these libraries in PlatformIO

If you decide you do need those .a files later, you have to tell PlatformIO
where they are. You would create a folder named lib in your project and update
your platformio.ini: Ini, TOML

```toml
build_flags =
    -Llib/             ; Tells the compiler to look in the 'lib' folder
    -larm_cortexM4lf_math
    -lPDMFilter_CM4F_GCC
```
Next Step

Try running the Permission Fix and then attempt to upload your "do-nothing"
main.c.

Would you like me to help you verify if the Black Magic Probe is correctly
detecting the chip once the permissions are fixed?

## Project strucuture (recommended)

```
.
├── include/
│   ├── pins.h        <-- Pin mappings (derived from board.h)
│   ├── main.h              <-- Global config and peripheral handles
│   └── FreeRTOSConfig.h    <-- FreeRTOS kernel settings
├── lib/                    <-- Put reusable sensor drivers here
│   ├── VL53L0X/            <-- TOF Sensor
│   │   ├── vl53l0x_api.c   <-- Vendor code (Copy-Paste)
│   │   ├── vl53l0x_api.h
│   │   └── vl53l0x_platform.c <-- YOUR PORT: Change I2C/Delays to HAL/FreeRTOS
│   ├── IMU/                <-- Accelerometer/Gyro
│   └── Motors/             <-- Motor control logic
├── src/
│   ├── main.c              <-- Initialization flow & main tasks
│   ├── stm32f4xx_hal_msp.c <-- Peripheral hardware init (clocks, GPIOs)
│   └── stm32f4xx_it.c      <-- Interrupt handlers
└── platformio.ini          <-- Project build settings
```
# Configuring pins

Done by looking at board.chcfg (`ChibiOS_ext/os/hal/boards/epuck2/cfg/board.chcfg`
). According to the spreadsheet: 
<https://www.hangpersonal.com/wp-content/uploads/2024/10/STM32F407-Alternate-Function-Mapping.pdf>
, showing the alternate mappings for the STM407, one can set the alternate
mappings corretly from the numbers. Code is then generated through STM32CubeMX.

# Data sheets

- Time of flight (vl53l0x): 
  <https://www.st.com/resource/en/datasheet/vl53l0x.pdf>
