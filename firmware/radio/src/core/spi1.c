#include "spi1.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

static spi_bus_config_t spi_bus_config = {
    .miso_io_num = PIN_NUM_MISO, .mosi_io_num = PIN_NUM_MOSI, .sclk_io_num = PIN_NUM_CLK};

static spi_slave_interface_config_t spi_slave_config = {
    .mode = 0,                  // SPI mode0: CPOL=0, CPHA=0.
    .spics_io_num = PIN_NUM_CS, // CS pin.
    .queue_size = 8,            // We want to be able to queue 8 transactions at a time.
    .flags = 0,
    //.post_setup_cb=my_post_setup_cb,
    //.post_trans_cb=my_post_trans_cb
};

esp_err_t spi1_init() {
    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    return spi_slave_initialize(VSPI_HOST, &spi_bus_config, &spi_slave_config, SPI_DMA_CH_AUTO);
}