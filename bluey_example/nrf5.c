/*
 spi interface for nordinc nrf5, for use with cserial_flash
*/

#include "flash.h"
#include <nrf_drv_spi.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>

#include "app_error.h"
#define NRF_LOG_MODULE_NAME "NRF_SPI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define SPI_MISO_PIN    28
#define SPI_MOSI_PIN    26
#define SPI_SCK_PIN     27
#define SPI_SS_PIN      29

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
    
    config.frequency = NRF_DRV_SPI_FREQ_8M;
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
    // NRF_LOG_RAW_INFO("SPI_TX  : %8X", data);
    nrf_drv_spi_transfer(&m_spi_master_0, &data, 1, &rdata, 1);
    // NRF_LOG_RAW_INFO(" %8X\r\n", rdata);
    return rdata;
}

static uint16_t spi_tx16(uint16_t data)
{
    uint16_t rdata;
    // NRF_LOG_RAW_INFO("SPI_TX16: %8X", data);
    // nrf_drv_spi_transfer(&m_spi_master_0, (uint8_t*)&data, 2, (uint8_t*)&rdata, 2);
    rdata = spi_tx(data >> 8) << 8;
    rdata |= spi_tx(data & 0xff);
    // NRF_LOG_RAW_INFO(" %8X\r\n", rdata);
    return rdata;
}

static void spi_tx_buf(void *buf, void *rdbuf, size_t count)
{
    // nrf_drv_spi_transfer(&m_spi_master_0, buf, (buf==NULL)?0:count, rdbuf, (rdbuf==NULL)?0:count);
    size_t sent=0;
    while(count>0)
    {
        uint8_t blen;
        if(count > 255)
        {
            blen = 255;
        }
        else
        {
            blen = count;
        }

        // NRF_LOG_RAW_INFO("RX: %d\r\n", blen);
        nrf_drv_spi_transfer(&m_spi_master_0, buf+sent, blen, rdbuf+sent, blen);

        count -= blen; 
        sent += blen;
    }
    // NRF_LOG_RAW_INFO("RX: ");
    // NRF_LOG_RAW_HEXDUMP_INFO(rdbuf, count);
    // NRF_LOG_RAW_INFO("\r\n");
}

static void spi_cs_assert()
{
    nrf_gpio_pin_clear(SPI_SS_PIN);
}

static void spi_cs_release()
{
    nrf_gpio_pin_set(SPI_SS_PIN);
}
