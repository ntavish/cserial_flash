/* SerialFlash Library - for filesystem-like access to SPI Serial Flash memory
 * https://github.com/PaulStoffregen/SerialFlash
 * Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy.
 * Please support PJRC's efforts to develop open source software by purchasing
 * Teensy or other genuine PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * Modified 2017, Electronut labs, info@electronut.in
 */
#include <stdint.h>
#include <string.h>
#include "flash.h"

static uint8_t flags = 0;	// chip features
static uint8_t busy = 0;
    // 1 = suspendable program operation
    // 2 = suspendable erase operation
    // 3 = busy for realz!!
static spi_interface * spi_if;

#define CSASSERT()  spi_if->cs_assert()
#define CSRELEASE() spi_if->cs_release()

#define FLAG_32BIT_ADDR		0x01	// larger than 16 MByte address
#define FLAG_STATUS_CMD70	0x02	// requires special busy flag check
#define FLAG_DIFF_SUSPEND	0x04	// uses 2 different suspend commands
#define FLAG_MULTI_DIE		0x08	// multiple die, don't read cross 32M barrier
#define FLAG_256K_BLOCKS	0x10	// has 256K erase blocks
#define FLAG_DIE_MASK		0xC0	// top 2 bits count during multi-die erase

void sf_wait(void)
{
    uint32_t status;
    //Serial.print("wait-");
    while (1) {
        spi_if->begin_tx();
        CSASSERT();
        if (flags & FLAG_STATUS_CMD70) {
            // some Micron chips require this different
            // command to detect program and erase completion
            spi_if->tx(0x70);
            status = spi_if->tx(0);
            CSRELEASE();
            spi_if->end_tx();
            //Serial.printf("b=%02x.", status & 0xFF);
            if ((status & 0x80)) break;
        } else {
            // all others work by simply reading the status reg
            spi_if->tx(0x05);
            status = spi_if->tx(0);
            CSRELEASE();
            spi_if->end_tx();
            //Serial.printf("b=%02x.", status & 0xFF);
            if (!(status & 1)) break;
        }
    }
    busy = 0;
    //Serial.println();
}

void sf_read(uint32_t addr, void *buf, void *rdbuf, uint32_t len)
{
    uint8_t *p = (uint8_t *)buf;
    uint8_t b, f, status, cmd;

    memset(p, 0, len);
    f = flags;
    spi_if->begin_tx();
    b = busy;
    if (b) {
        // read status register ... chip may no longer be busy
        CSASSERT();
        if (flags & FLAG_STATUS_CMD70) {
            spi_if->tx(0x70);
            status = spi_if->tx(0);
            if ((status & 0x80)) b = 0;
        } else {
            spi_if->tx(0x05);
            status = spi_if->tx(0);
            if (!(status & 1)) b = 0;
        }
        CSRELEASE();
        if (b == 0) {
            // chip is no longer busy :-)
            busy = 0;
        } else if (b < 3) {
            // TODO: this may not work on Spansion chips
            // which apparently have 2 different suspend
            // commands, for program vs erase
            CSASSERT();
            spi_if->tx(0x06); // write enable (Micron req'd)
            CSRELEASE();
            spi_if->delay_us(1);
            cmd = 0x75; //Suspend program/erase for almost all chips
            // but Spansion just has to be different for program suspend!
            if ((f & FLAG_DIFF_SUSPEND) && (b == 1)) cmd = 0x85;
            CSASSERT();
            spi_if->tx(cmd); // Suspend command
            CSRELEASE();
            if (f & FLAG_STATUS_CMD70) {
                // Micron chips don't actually suspend until flags read
                CSASSERT();
                spi_if->tx(0x70);
                do {
                    status = spi_if->tx(0);
                } while (!(status & 0x80));
                CSRELEASE();
            } else {
                CSASSERT();
                spi_if->tx(0x05);
                do {
                    status = spi_if->tx(0);
                } while ((status & 0x01));
                CSRELEASE();
            }
        } else {
            // chip is busy with an operation that can not suspend
            spi_if->end_tx();	// is this a good idea?
            sf_wait();			// should we wait without ending
            b = 0;			// the transaction??
            spi_if->begin_tx();
        }
    }
    do {
        uint32_t rdlen = len;
        if (f & FLAG_MULTI_DIE) {
            if ((addr & 0xFE000000) != ((addr + len - 1) & 0xFE000000)) {
                rdlen = 0x2000000 - (addr & 0x1FFFFFF);
            }
        }
        CSASSERT();
        // TODO: FIFO optimize....
        if (f & FLAG_32BIT_ADDR) {
            spi_if->tx(0x03);
            spi_if->tx16(addr >> 16);
            spi_if->tx16(addr);
        } else {
            spi_if->tx16(0x0300 | ((addr >> 16) & 255));
            spi_if->tx16(addr);
        }
        spi_if->tx_buf(p, (void *)((uint8_t*)rdbuf+(p-(uint8_t*)buf)), rdlen);
        CSRELEASE();
        p += rdlen;
        addr += rdlen;
        len -= rdlen;
    } while (len > 0);
    if (b) {
        CSASSERT();
        spi_if->tx(0x06); // write enable (Micron req'd)
        CSRELEASE();
        spi_if->delay_us(1);
        cmd = 0x7A;
        if ((f & FLAG_DIFF_SUSPEND) && (b == 1)) cmd = 0x8A;
        CSASSERT();
        spi_if->tx(cmd); // Resume program/erase
        CSRELEASE();
    }
    spi_if->end_tx();
}

void sf_write(uint32_t addr, const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    uint32_t max, pagelen;

    //Serial.printf("WR: addr %08X, len %d\n", addr, len);
    do {
        if (busy) sf_wait();
        spi_if->begin_tx();
        CSASSERT();
        // write enable command
        spi_if->tx(0x06);
        CSRELEASE();
        max = 256 - (addr & 0xFF);
        pagelen = (len <= max) ? len : max;
        //Serial.printf("WR: addr %08X, pagelen %d\n", addr, pagelen);
        spi_if->delay_us(1); // TODO: reduce this, but prefer safety first
        CSASSERT();
        if (flags & FLAG_32BIT_ADDR) {
            spi_if->tx(0x02); // program page command
            spi_if->tx16(addr >> 16);
            spi_if->tx16(addr);
        } else {
            spi_if->tx16(0x0200 | ((addr >> 16) & 255));
            spi_if->tx16(addr);
        }
        addr += pagelen;
        len -= pagelen;
        do {
            spi_if->tx(*p++);
        } while (--pagelen > 0);
        CSRELEASE();
        busy = 4;
        spi_if->end_tx();
    } while (len > 0);
}

void sf_eraseAll()
{
    if (busy) sf_wait();
    uint8_t id[5];
    sf_readID(id);
    //Serial.printf("ID: %02X %02X %02X\n", id[0], id[1], id[2]);
    if (id[0] == 0x20 && id[2] >= 0x20 && id[2] <= 0x22) {
        // Micron's multi-die chips require special die erase commands
        //  N25Q512A	20 BA 20  2 dies  32 Mbyte/die   65 nm transitors
        //  N25Q00AA	20 BA 21  4 dies  32 Mbyte/die   65 nm transitors
        //  MT25QL02GC	20 BA 22  2 dies  128 Mbyte/die  45 nm transitors
        uint8_t die_count = 2;
        if (id[2] == 0x21) die_count = 4;
        uint8_t die_index = flags >> 6;
        //Serial.printf("Micron die erase %d\n", die_index);
        flags &= 0x3F;
        if (die_index >= die_count) return; // all dies erased :-)
        uint8_t die_size = 2;  // in 16 Mbyte units
        if (id[2] == 0x22) die_size = 8;
        spi_if->begin_tx();
        CSASSERT();
        spi_if->tx(0x06); // write enable command
        CSRELEASE();
        spi_if->delay_us(1);
        CSASSERT();
        // die erase command
        spi_if->tx(0xC4);
        spi_if->tx16((die_index * die_size) << 8);
        spi_if->tx16(0x0000);
        CSRELEASE();
        //Serial.printf("Micron erase begin\n");
        flags |= (die_index + 1) << 6;
    } else {
        // All other chips support the bulk erase command
        spi_if->begin_tx();
        CSASSERT();
        // write enable command
        spi_if->tx(0x06);
        CSRELEASE();
        spi_if->delay_us(1);
        CSASSERT();
        // bulk erase command
        spi_if->tx(0xC7);
        CSRELEASE();
        spi_if->end_tx();
    }
    busy = 3;
}

void sf_eraseBlock(uint32_t addr)
{
    uint8_t f = flags;
    if (busy) sf_wait();
    spi_if->begin_tx();
    CSASSERT();
    spi_if->tx(0x06); // write enable command
    CSRELEASE();
    spi_if->delay_us(1);
    CSASSERT();
    if (f & FLAG_32BIT_ADDR) {
        spi_if->tx(0xD8);
        spi_if->tx16(addr >> 16);
        spi_if->tx16(addr);
    } else {
        spi_if->tx16(0xD800 | ((addr >> 16) & 255));
        spi_if->tx16(addr);
    }
    CSRELEASE();
    spi_if->end_tx();
    busy = 2;
}


int sf_ready()
{
    uint32_t status;
    if (!busy) return RET_OK;
    spi_if->begin_tx();
    CSASSERT();
    if (flags & FLAG_STATUS_CMD70) {
        // some Micron chips require this different
        // command to detect program and erase completion
        spi_if->tx(0x70);
        status = spi_if->tx(0);
        CSRELEASE();
        spi_if->end_tx();
        //Serial.printf("ready=%02x\n", status & 0xFF);
        if ((status & 0x80) == 0) return RET_ERR;
    } else {
        // all others work by simply reading the status reg
        spi_if->tx(0x05);
        status = spi_if->tx(0);
        CSRELEASE();
        spi_if->end_tx();
        //Serial.printf("ready=%02x\n", status & 0xFF);
        if ((status & 1)) return RET_ERR;
    }
    busy = 0;
    if (flags & 0xC0) {
        // continue a multi-die erase
        sf_eraseAll();
        return RET_ERR;
    }
    return RET_OK;
}


#define ID0_WINBOND	0xEF
#define ID0_SPANSION	0x01
#define ID0_MICRON	0x20
#define ID0_MACRONIX	0xC2
#define ID0_SST		0xBF

//#define FLAG_32BIT_ADDR	0x01	// larger than 16 MByte address
//#define FLAG_STATUS_CMD70	0x02	// requires special busy flag check
//#define FLAG_DIFF_SUSPEND	0x04	// uses 2 different suspend commands
//#define FLAG_256K_BLOCKS	0x10	// has 256K erase blocks

int sf_begin(spi_interface * sif)
{
    uint8_t id[5];
    uint8_t f;
    uint32_t size;

    // file scope variable at top
    spi_if = sif;
    spi_if->init();

    CSRELEASE();
    sf_readID(id);
    f = 0;
    size = sf_capacity(id);
    if (size > 16777216) {
        // more than 16 Mbyte requires 32 bit addresses
        f |= FLAG_32BIT_ADDR;
        spi_if->begin_tx();
        if (id[0] == ID0_SPANSION) {
            // spansion uses MSB of bank register
            CSASSERT();
            spi_if->tx16(0x1780); // bank register write
            CSRELEASE();
        } else {
            // micron & winbond & macronix use command
            CSASSERT();
            spi_if->tx(0x06); // write enable
            CSRELEASE();
            spi_if->delay_us(1);
            CSASSERT();
            spi_if->tx(0xB7); // enter 4 byte addr mode
            CSRELEASE();
        }
        spi_if->end_tx();
        if (id[0] == ID0_MICRON) f |= FLAG_MULTI_DIE;
    }
    if (id[0] == ID0_SPANSION) {
        // Spansion has separate suspend commands
        f |= FLAG_DIFF_SUSPEND;
        if (!id[4]) {
            // Spansion chips with id[4] == 0 use 256K sectors
            f |= FLAG_256K_BLOCKS;
        }
    }
    if (id[0] == ID0_MICRON) {
        // Micron requires busy checks with a different command
        f |= FLAG_STATUS_CMD70; // TODO: all or just multi-die chips?
    }
    flags = f;
    sf_readID(id);
    return RET_OK;
}

// chips tested: https://github.com/PaulStoffregen/SerialFlash/pull/12#issuecomment-169596992
//
void sf_sleep()
{
    if (busy) sf_wait();
    spi_if->begin_tx();
    CSASSERT();
    spi_if->tx(0xB9); // Deep power down command
    CSRELEASE();
}

void sf_wakeup()
{
    spi_if->begin_tx();
    CSASSERT();
    spi_if->tx(0xAB); // Wake up from deep power down command
    CSRELEASE();
}

void sf_readID(uint8_t *buf)
{
    if (busy) sf_wait();
    spi_if->begin_tx();
    CSASSERT();
    spi_if->tx(0x9F);
    buf[0] = spi_if->tx(0); // manufacturer ID
    buf[1] = spi_if->tx(0); // memory type
    buf[2] = spi_if->tx(0); // capacity
    if (buf[0] == ID0_SPANSION) {
        buf[3] = spi_if->tx(0); // ID-CFI
        buf[4] = spi_if->tx(0); // sector size
    }
    CSRELEASE();
    spi_if->end_tx();
    //Serial.printf("ID: %02X %02X %02X\n", buf[0], buf[1], buf[2]);
}

void sf_readSerialNumber(uint8_t *buf) //needs room for 8 bytes
{
    if (busy) sf_wait();
    spi_if->begin_tx();
    CSASSERT();
    spi_if->tx(0x4B);			
    spi_if->tx16(0);	
    spi_if->tx16(0);
    for (int i=0; i<8; i++) {		
        buf[i] = spi_if->tx(0);
    }
    CSRELEASE();
    spi_if->end_tx();
//	Serial.printf("Serial Number: %02X %02X %02X %02X %02X %02X %02X %02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
}

uint32_t sf_capacity(const uint8_t *id)
{
    uint32_t n = 1048576; // unknown chips, default to 1 MByte

    if (id[2] >= 16 && id[2] <= 31) {
        n = 1ul << id[2];
    } else
    if (id[2] >= 32 && id[2] <= 37) {
        n = 1ul << (id[2] - 6);
    } else
    if ((id[0]==0 && id[1]==0 && id[2]==0) || 
        (id[0]==255 && id[1]==255 && id[2]==255)) {
        n = 0;
    } else
    if(id[0]==0x1f)
    {
        switch(id[1])
        {
            case 0x84: n = 4194304; break;
            case 0x85: n = 8388608; break;
            case 0x86: n = 16777216; break;
            case 0x87: n = 33554432; break;
            case 0x15: n = 33554432; break;
            case 0x16: n = 67108864; break;
            case 0x17: n = 134217728; break;
        }
    }
    //Serial.printf("capacity %lu\n", n);
    return n;
}

uint32_t sf_blockSize()
{
    // Spansion chips >= 512 mbit use 256K sectors
    if (flags & FLAG_256K_BLOCKS) return 262144;
    // everything else seems to have 64K sectors
    return 65536;
}


 

/*
Chip		Uniform Sector Erase
        20/21	52	D8/DC
        -----	--	-----
W25Q64CV	4	32	64
W25Q128FV	4	32	64
S25FL127S			64
N25Q512A	4		64
N25Q00AA	4		64
S25FL512S			256
SST26VF032	4
*/



//			size	sector			busy	pgm/erase	chip
// Part			Mbyte	kbyte	ID bytes	cmd	suspend		erase
// ----			----	-----	--------	---	-------		-----
// Winbond W25Q64CV	8	64	EF 40 17
// Winbond W25Q128FV	16	64	EF 40 18	05	single		60 & C7
// Winbond W25Q256FV	32	64	EF 40 19	
// Spansion S25FL064A	8	?	01 02 16
// Spansion S25FL127S	16	64	01 20 18	05
// Spansion S25FL128P	16	64	01 20 18
// Spansion S25FL256S	32	64	01 02 19	05			60 & C7
// Spansion S25FL512S	64	256	01 02 20
// Macronix MX25L12805D 16	?	C2 20 18
// Macronix MX66L51235F	64		C2 20 1A
// Numonyx M25P128	16	?	20 20 18
// Micron M25P80	1	?	20 20 14
// Micron N25Q128A	16	64	20 BA 18
// Micron N25Q512A	64	?	20 BA 20	70	single		C4 x2
// Micron N25Q00AA	128	64	20 BA 21		single		C4 x4
// Micron MT25QL02GC	256	64	20 BA 22	70			C4 x2
// SST SST25WF010	1/8	?	BF 25 02
// SST SST25WF020	1/4	?	BF 25 03
// SST SST25WF040	1/2	?	BF 25 04
// SST SST25VF016B	1	?	BF 25 41
// SST26VF016			?	BF 26 01
// SST26VF032			?	BF 26 02
// SST25VF032		4	64	BF 25 4A
// SST26VF064		8	?	BF 26 43
// LE25U40CMC		1/2	64	62 06 13