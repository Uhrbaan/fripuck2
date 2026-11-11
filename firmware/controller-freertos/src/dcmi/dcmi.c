#include "dcmi.h"
#include "stm32f4xx_hal.h"

extern DCMI_HandleTypeDef hdcmi;
extern TIM_HandleTypeDef htim5;

volatile uint32_t xclk_edges, pclk_edges, href_edges, vsync_edges;

void cam_signal_probe(void) {
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &g);  // PCLK, HSYNC
    g.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &g);  // VSYNC

    uint32_t pa_prev = GPIOA->IDR, pb_prev = GPIOB->IDR;
    for (uint32_t i = 0; i < 2000000; i++) {
        uint32_t pa = GPIOA->IDR, pb = GPIOB->IDR;
        if ((pa ^ pa_prev) & GPIO_PIN_6) pclk_edges++;
        if ((pa ^ pa_prev) & GPIO_PIN_4) href_edges++;
        if ((pb ^ pb_prev) & GPIO_PIN_7) vsync_edges++;
        pa_prev = pa;
        pb_prev = pb;
    }
    __BKPT(0);
}

volatile uint32_t cnt[3][4];  // per test: xclk, pclk, href, vsync edges

static void count_edges(volatile uint32_t* c) {
    c[0] = c[1] = c[2] = c[3] = 0;
    uint32_t pa_prev = GPIOA->IDR, pb_prev = GPIOB->IDR;
    for (uint32_t i = 0; i < 2000000; i++) {
        uint32_t pa = GPIOA->IDR, pb = GPIOB->IDR;  // IDR is readable even in AF mode
        uint32_t da = pa ^ pa_prev, db = pb ^ pb_prev;
        if (da & GPIO_PIN_0) c[0]++;  // XCLK (PA0)
        if (da & GPIO_PIN_6) c[1]++;  // PCLK
        if (da & GPIO_PIN_4) c[2]++;  // HREF
        if (db & GPIO_PIN_7) c[3]++;  // VSYNC
        pa_prev = pa;
        pb_prev = pb;
    }
}

void xclk_test(void) {
    count_edges(cnt[0]);  // 21 MHz

    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);  // XCLK off
    HAL_Delay(100);
    count_edges(cnt[1]);

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);  // XCLK at 14 MHz
    __HAL_TIM_SET_AUTORELOAD(&htim5, 2);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1);
    HAL_Delay(100);
    count_edges(cnt[2]);

    __BKPT(0);
}