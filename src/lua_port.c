#include <sys/time.h>
#include <hal.h> // For ChibiOS HAL functions

// This implements the Newlib function that Lua is ultimately trying to call.
int _gettimeofday(struct timeval *tv, void *tzp) {
    if (tv != NULL) {
        // Get the current time in system ticks (ChibiOS 4.x/5.x uses chTimeGet, 3.x used chVTGetSystemTimeX)
        // We'll use the current best practice for general time.
        // The time is usually in milliseconds or microseconds depending on your setup.
        
        // Use chVTGetSystemTime() for the current tick count.
        // We'll assume CH_CFG_ST_FREQUENCY is 1000 (1ms tick) for simplicity.
        // You might need to adjust this conversion based on your RTOS configuration.

        systime_t ticks = chVTGetSystemTime();
        
        // Convert ChibiOS ticks to seconds and microseconds
        // Note: For high precision, this might be better done with a separate RTC/Calendar driver.
        // For a basic stub, a millisecond-based tick conversion is usually adequate.
        
        // Assuming CH_CFG_ST_FREQUENCY is 1000 (1ms) for simplicity:
        #define TICKS_PER_SEC CH_CFG_ST_FREQUENCY
        
        // systime_t is usually 32-bit or 64-bit depending on configuration
        // Use a 64-bit intermediate for safe math if possible.
        uint64_t total_ms = (uint64_t)ticks * (1000 / TICKS_PER_SEC);

        tv->tv_sec = total_ms / 1000;
        tv->tv_usec = (total_ms % 1000) * 1000;
    }
    // Timezone information is usually ignored in embedded systems
    return 0; 
}