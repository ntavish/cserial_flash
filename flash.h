#ifndef __CERIAL_FLASH_H
#define __CERIAL_FLASH_H

/*
Generic winbond like SPI flash driver
based on code from https://github.com/PaulStoffregen/SerialFlash 
which is: Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com
*/

#include <stdint.h>

typedef struct _spi_interface
{
    //  initialize SPI bus, returns 0 on success
    int (*init)();
    // releases SPI bus, returns 0 on success
    int (*deinit)();
    // spi data txfr functions
    uint8_t (*tx)(uint8_t data);
    uint16_t (*tx16)(uint16_t data);
    void (*tx_buf)(void *buf, size_t count);
    // chip select functions
    void (*cs_assert)();
    void (*cs_release)();
}spi_interface;

typedef struct _spi_flash_instance 
{
    int begin();
    uint32_t capacity(const uint8_t *id);
    uint32_t blockSize();
    void sleep();
    void wakeup();
    void readID(uint8_t *buf);
    void readSerialNumber(uint8_t *buf);
    void read(uint32_t addr, void *buf, uint32_t len);
    bool ready();
    void wait();
    void write(uint32_t addr, const void *buf, uint32_t len);
    void eraseAll();
    void eraseBlock(uint32_t addr);

	uint8_t flags;	// chip features
    uint8_t busy;
        // 1 = suspendable program operation
        // 2 = suspendable erase operation
        // 3 = busy for realz!!
    spi_interface * spi_if;
}flash_instance;

#endif