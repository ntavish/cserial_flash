#ifndef __CERIAL_FLASH_H
#define __CERIAL_FLASH_H

/*
Generic winbond like SPI flash driver
based on code from https://github.com/PaulStoffregen/SerialFlash 
which is: Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com
*/
#include <stdint.h>
#include <stddef.h>

#define RET_OK 1
#define RET_ERR 0

typedef struct _spi_interface
{
    //  initialize SPI bus, returns 0 on success
    int (*init)();
    // releases SPI bus, returns 0 on success
    int (*deinit)();
    // spi data txfr functions
    void (*begin_tx)();  // spi should be set to MSBFIRST, SPI_MODE0
    void (*end_tx)();
    uint8_t (*tx)(uint8_t data);
    uint16_t (*tx16)(uint16_t data);
    void (*tx_buf)(void *buf, void *rdbuf, size_t count); // buf or rdbuf can be null
    // chip select functions
    void (*cs_assert)();
    void (*cs_release)();

    // delay function
    void (*delay_us)(uint32_t);
}spi_interface;

int sf_begin(spi_interface * spi_if);
uint32_t sf_capacity(const uint8_t *id);
uint32_t sf_blockSize();
void sf_sleep();
void sf_wakeup();
void sf_readID(uint8_t *buf);
void sf_readSerialNumber(uint8_t *buf);
void sf_read(uint32_t addr, void *buf, uint32_t len);
int sf_ready();
void sf_wait();
void sf_write(uint32_t addr, const void *buf, uint32_t len);
void sf_eraseAll();
void sf_eraseBlock(uint32_t addr);

#endif