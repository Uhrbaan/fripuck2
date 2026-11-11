#include "main.h"
#include "stm32f4xx_hal.h"

extern I2C_TypeDef hi2c1;

volatile uint8_t bb_found[16];
volatile uint8_t bb_n;
volatile uint32_t bb_idle;  // bit0 = SCL high at idle, bit1 = SDA high at idle

#define SCL_H() (GPIOB->BSRR = GPIO_PIN_8)
#define SCL_L() (GPIOB->BSRR = GPIO_PIN_8 << 16)
#define SDA_H() (GPIOB->BSRR = GPIO_PIN_9)
#define SDA_L() (GPIOB->BSRR = GPIO_PIN_9 << 16)
#define SDA_RD() ((GPIOB->IDR & GPIO_PIN_9) != 0)
#define SCL_RD() ((GPIOB->IDR & GPIO_PIN_8) != 0)

static void dly(void) { for (volatile int i = 0; i < 400; i++); }  // roughly 20-50 kHz

static void bb_start(void) {
    SDA_H();
    dly();
    SCL_H();
    dly();
    SDA_L();
    dly();
    SCL_L();
    dly();
}
static void bb_stop(void) {
    SDA_L();
    dly();
    SCL_H();
    dly();
    SDA_H();
    dly();
}

static int bb_write(uint8_t b) {  // returns 1 if ACKed
    for (int i = 0; i < 8; i++) {
        if (b & 0x80)
            SDA_H();
        else
            SDA_L();
        b <<= 1;
        dly();
        SCL_H();
        dly();
        SCL_L();
        dly();
    }
    SDA_H();
    dly();
    SCL_H();
    dly();
    int ack = !SDA_RD();
    SCL_L();
    dly();
    return ack;
}

void bb_scan(void) {
    HAL_I2C_DeInit(&hi2c1);  // give the pins back to GPIO

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    SDA_H();
    SCL_H();
    dly();
    bb_idle = SCL_RD() | (SDA_RD() << 1);

    // real unstick: 9+ clocks, then STOP
    for (int i = 0; i < 16; i++) {
        SCL_L();
        dly();
        SCL_H();
        dly();
    }
    bb_stop();

    bb_n = 0;
    for (uint8_t a = 1; a < 128; a++) {
        bb_start();
        if (bb_write(a << 1) && bb_n < 16) bb_found[bb_n++] = a;
        bb_stop();
    }
    __BKPT(0);  // inspect bb_idle, bb_n, bb_found[]
}