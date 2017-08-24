/*
 spi interface for nordinc nrf5, for use with cserial_flash
*/

#include "flash.h"
#include <nrf_drv_spi.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>

#define SPI_MISO_PIN    30
#define SPI_MOSI_PIN    29
#define SPI_SCK_PIN     0
#define SPI_SS_PIN      7

static int spi_init();
static int spi_deinit();
static void spi_begin_tx();
static void spi_end_tx();
static uint8_t spi_tx(uint8_t data);
static uint16_t spi_tx16(uint16_t data);
static void spi_tx_buf(void *buf, void *rdbuf, size_t count);
static void spi_cs_assert();
static void spi_cs_release();

spi_interface nrf_spi = 
{
        .init = spi_init,
        .deinit = spi_deinit,
        .begin_tx = spi_begin_tx,
        .end_tx = spi_end_tx,
        .tx = spi_tx,
        .tx16 = spi_tx16,
        .tx_buf = spi_tx_buf,
        .cs_assert = spi_cs_assert,
        .cs_release = spi_cs_release,
        .delay_us = nrf_delay_us,
};

static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);
static nrf_drv_spi_config_t config = NRF_DRV_SPI_DEFAULT_CONFIG;

static int spi_init()
{
    uint32_t err_code;

    // set SS pin as output
    nrf_gpio_pin_set(SPI_SS_PIN);
    nrf_gpio_cfg_output(SPI_SS_PIN);
    
    config.frequency = NRF_DRV_SPI_FREQ_1M;
    config.mode = NRF_DRV_SPI_MODE_0;
    config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    config.miso_pin = SPI_MISO_PIN;
    config.mosi_pin = SPI_MOSI_PIN;
    config.sck_pin = SPI_SCK_PIN;
    config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;

    err_code = nrf_drv_spi_init(&m_spi_master_0, &config, NULL);
    if (err_code != NRF_SUCCESS)
    {
        return RET_ERR;
    }

    return RET_OK;
}

static int spi_deinit()
{
    nrf_drv_spi_uninit(&m_spi_master_0);

    return RET_OK;
}

static void spi_begin_tx()
{
    // nothing to do
}

static void spi_end_tx()
{
    // nothing to do
}

static uint8_t spi_tx(uint8_t data)
{
    uint8_t rdata;
    nrf_drv_spi_transfer(&m_spi_master_0, &data, 1, &rdata, 1);

    return rdata;
}

static uint16_t spi_tx16(uint16_t data)
{
    uint16_t rdata;
    nrf_drv_spi_transfer(&m_spi_master_0, (uint8_t*)&data, 2, (uint8_t*)&rdata, 2);

    return rdata;
}

static void spi_tx_buf(void *buf, void *rdbuf, size_t count)
{
    nrf_drv_spi_transfer(&m_spi_master_0, buf, count, rdbuf, count);
}

static void spi_cs_assert()
{
    nrf_gpio_pin_clear(SPI_SS_PIN);
}

static void spi_cs_release()
{
    nrf_gpio_pin_set(SPI_SS_PIN);
}
