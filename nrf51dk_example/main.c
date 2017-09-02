/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include "flash.h"

// #include "app_util_platform.h"
#include "app_error.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

const char * id2chip(const unsigned char *id)
{
	if (id[0] == 0xEF) {
		// Winbond
		if (id[1] == 0x40) {
			if (id[2] == 0x14) return "W25Q80BV";
			if (id[2] == 0x15) return "W25Q16DV";
			if (id[2] == 0x17) return "W25Q64FV";
			if (id[2] == 0x18) return "W25Q128FV";
			if (id[2] == 0x19) return "W25Q256FV";
		}
	}
	if (id[0] == 0x01) {
		// Spansion
		if (id[1] == 0x02) {
			if (id[2] == 0x16) return "S25FL064A";
			if (id[2] == 0x19) return "S25FL256S";
			if (id[2] == 0x20) return "S25FL512S";
		}
		if (id[1] == 0x20) {
			if (id[2] == 0x18) return "S25FL127S";
		}
	}
	if (id[0] == 0xC2) {
		// Macronix
		if (id[1] == 0x20) {
			if (id[2] == 0x18) return "MX25L12805D";
		}
	}
	if (id[0] == 0x20) {
		// Micron
		if (id[1] == 0xBA) {
			if (id[2] == 0x20) return "N25Q512A";
			if (id[2] == 0x21) return "N25Q00AA";
		}
		if (id[1] == 0xBB) {
			if (id[2] == 0x22) return "MT25QL02GC";
		}
	}
	if (id[0] == 0xBF) {
		// SST
		if (id[1] == 0x25) {
			if (id[2] == 0x02) return "SST25WF010";
			if (id[2] == 0x03) return "SST25WF020";
			if (id[2] == 0x04) return "SST25WF040";
			if (id[2] == 0x41) return "SST25VF016B";
			if (id[2] == 0x4A) return "SST25VF032";
		}
		if (id[1] == 0x26) {
			if (id[2] == 0x01) return "SST26VF016";
			if (id[2] == 0x02) return "SST26VF032";
			if (id[2] == 0x43) return "SST26VF064";
		}
    }
    if (id[0] == 0x1F) {
		// adesto
		switch(id[1])
        {
            case 0x84: return "AT25SF041";
            case 0x85: return "AT25SF081";
            case 0x86: return "AT25SF161";
            case 0x87: return "AT25SF321";
            case 0x15: return "AT25SF321";
            case 0x16: return "AT25SF641";
            case 0x17: return "AT25SL128A";
        }
	}
	return "(unknown chip)";
}

/*
 * @brief tests flash chip
 */
int flash_test()
{
    unsigned char buf[256];
    unsigned long  chipsize, blocksize;

    // Read the chip identification
    NRF_LOG_RAW_INFO("Read Chip Identification:\r\n");
    sf_readID(buf);
    NRF_LOG_RAW_INFO("  JEDEC ID:     ");
    NRF_LOG_RAW_INFO("%x", buf[0]);
    NRF_LOG_RAW_INFO(" ");
    NRF_LOG_RAW_INFO("%x", buf[1]);
    NRF_LOG_RAW_INFO(" ");
    NRF_LOG_RAW_INFO("%x\r\n", buf[2]);
    NRF_LOG_RAW_INFO("  Part Number: ");
    NRF_LOG_RAW_INFO("%s\r\n", (int)id2chip(buf));
    NRF_LOG_RAW_INFO("  Memory Size:  ");
    chipsize = sf_capacity(buf);
    NRF_LOG_RAW_INFO("%u\r\n", chipsize);
    NRF_LOG_RAW_INFO(" bytes");
    if (chipsize == 0) return RET_ERR;
    NRF_LOG_RAW_INFO("  Block Size:   ");
    blocksize = sf_blockSize();
    NRF_LOG_RAW_INFO("%d", blocksize);
    NRF_LOG_RAW_INFO(" byte\r\n");
    void sf_readSerialNumber2(uint8_t *buf);

    // this is specific to adesto chips
    sf_readSerialNumber2(buf);
    NRF_LOG_RAW_INFO("Serial Number: %02X %02X \r\n", buf[0], buf[1]);

    NRF_LOG_FLUSH();

    uint8_t buf2 [256];
    uint8_t rdbuf[300];
    for(unsigned int i=0;i<sizeof(buf2);i++)
    {
        buf2[i] = i%20;
    }

#if 1

    NRF_LOG_RAW_INFO("\r\nBuffer: \r\n");
    NRF_LOG_RAW_HEXDUMP_INFO(buf2, sizeof(buf2));
    NRF_LOG_FLUSH();

    // sf_eraseBlock(0);
    // sf_eraseAll();
    
    sf_read(0, rdbuf, sizeof(rdbuf));
    NRF_LOG_RAW_INFO("\r\nBwrite: \r\n");
    NRF_LOG_RAW_HEXDUMP_INFO(rdbuf, sizeof(rdbuf));
    NRF_LOG_FLUSH();

    // const char teststr[] = {0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55};
    sf_write(0, buf2, sizeof(buf2));
    
    sf_read(0, rdbuf, sizeof(rdbuf));
    NRF_LOG_RAW_INFO("\r\nAwrite: \r\n");
    NRF_LOG_RAW_HEXDUMP_INFO(rdbuf, sizeof(rdbuf));
    NRF_LOG_FLUSH();

#else

    nrf_delay_ms(500);
    NRF_LOG_INFO("start");
    NRF_LOG_FLUSH();
    for(int i=0;i<chipsize;i+=256)
    {
        sf_read(i, rdbuf, 256);
        // NRF_LOG_RAW_HEXDUMP_INFO(rdbuf, 256);
        // NRF_LOG_FLUSH();    
    }
    NRF_LOG_INFO("stop");
    NRF_LOG_FLUSH();
#endif

    return RET_OK;
}
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /* Configure board. */
    bsp_board_leds_init();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    
    NRF_LOG_INFO("App starting\r\n");
    NRF_LOG_FLUSH();

    extern spi_interface nrf_spi;
    sf_begin(&nrf_spi);

    flash_test();

    NRF_LOG_FLUSH();

    /* Toggle LEDs. */
    while (true)
    {
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
            bsp_board_led_invert(i);
            nrf_delay_ms(500);
        }
    }
}

/**
 *@}
 **/
