/* Copyright (c) 2012, Vitaly Bursov <vitaly<AT>bursov.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <string.h>

#include "nvram.h"
#include "xlibc.h"
#include "led.h"


char nvram_timer;

#define NVR_VERSION (0x81)

struct nvelement {
    uint8_t index;
    uint8_t data[NVR_DATA_COUNT];
    uint8_t cksum;
};

#define NVR_VERSION_IDX ((void*)0) /* size: 1 */
#define NVR_RING_IDX ((void*)1) /* size: sizeof(struct nvelement) x NVR_RING_ELEMENTS */

#define NVR_RING_ELEMENTS (17) /* struct nvelement's (must fit in ring_status.ring_index) */

#define NVR_INDEX_BITS (6) /* must be 2x larger than NVR_RING_ELEMENTS, and less than 8 */
#define NVR_INDEX_INVALID (NVR_RING_ELEMENTS)

static struct {
    uint8_t dirty:1;
    uint8_t write_scheduled:1;
    uint8_t ring_index:6;

    uint8_t index;
} ring_status;

static uint8_t ring_data[NVR_DATA_COUNT];

static char read_element(uint8_t idx, struct nvelement *e)
{
    uint8_t *buf = (uint8_t*)e;
    uint8_t i, cksum=0;

    for (i=0;i<sizeof(struct nvelement);i++){
	buf[i] = eeprom_read_byte(NVR_RING_IDX + sizeof(struct nvelement)*idx + i);
	cksum += buf[i];
	wdt_reset();
    }

    return cksum != 0;
}

static void write_element(uint8_t idx, struct nvelement *e)
{
    uint8_t *buf = (uint8_t*)e;
    uint8_t i;

    for (i=0;i<sizeof(struct nvelement);i++){
	eeprom_write_byte(NVR_RING_IDX + sizeof(struct nvelement)*idx + i, buf[i]);
	wdt_reset();
    }
}


static void rbuf_scan()
{
    uint8_t i, found=0;
    struct nvelement el;

    /* find first valid element */
    for (i=0;i<NVR_RING_ELEMENTS;i++){
	if (read_element(i, &el)){
	    /* bad, ignore */
	    continue;
	}

	ring_status.ring_index = i;
	ring_status.index = el.index;
	memcpy(ring_data, &el.data, NVR_DATA_COUNT);

	found = 1;
	break;
    }

    if (!found){
	return;
    }

    /* scan further, stop when index changes too much */
    for (;i<NVR_RING_ELEMENTS;i++){
	uint8_t diff;

	if (read_element(i, &el)){
	    /* bad, ignore */
	    continue;
	}

	if (ring_status.index < el.index){
	    diff = el.index - ring_status.index;
	} else {
	    diff = 0xff - ring_status.index + el.index + 1;
	}

	/* is next? */
	if (diff < NVR_RING_ELEMENTS){
	    ring_status.ring_index = i;
	    ring_status.index = el.index;
	    memcpy(ring_data, &el.data, NVR_DATA_COUNT);
	} else {
	    break;
	}
    }

}

void nvram_init()
{
    uint8_t buf;

    ring_status.dirty = 0;
    ring_status.write_scheduled = 0;
    ring_status.ring_index = NVR_RING_ELEMENTS;
    ring_status.index = 0;

    buf = eeprom_read_byte(NVR_VERSION_IDX);
    if (buf == NVR_VERSION){
	putl_P(PSTR("restore "));
	rbuf_scan();
	putn(ring_status.ring_index);
	putn(ring_status.index);
	putn(ring_data[0]);
	putl_P(PSTR("\n"));
    } else {
	putl_P(PSTR("!nvram\n"));
    }
}

char nvram_get(uint8_t ctl, uint8_t *value)
{
    if (ring_status.ring_index == NVR_RING_ELEMENTS)
	return 0;
    *value = ring_data[ctl];
    return 1;
}

void nvram_set(uint8_t ctl, uint8_t value)
{
    if (ring_data[ctl] != value){
	ring_data[ctl] = value;
	ring_status.dirty = 1;
	ring_status.write_scheduled = 0;
    }
}

static void nvram_write_data()
{
    struct nvelement el;
    uint8_t i;

    if (ring_status.ring_index == NVR_RING_ELEMENTS){
	eeprom_write_byte(NVR_VERSION_IDX, NVR_VERSION);
    } else {
	ring_status.ring_index += 1;
    }
    if (ring_status.ring_index >= NVR_RING_ELEMENTS)
	ring_status.ring_index = 0;

    ring_status.index++;

    el.index = ring_status.index;
    el.cksum = -ring_status.index;

    for (i=0;i<NVR_DATA_COUNT;i++){
	el.data[i] = ring_data[i];
	el.cksum -= ring_data[i];
    }

    write_element(ring_status.ring_index, &el);
}

void nvram_save()
{
    if (ring_status.dirty){

	if (!ring_status.write_scheduled){
	    /* schedule write */
	    cli();
	    nvram_timer = NVR_SAVE_DELAY/(1000/TICK_HZ) + 1;
	    sei();
	    ring_status.write_scheduled = 1;
	    return;
	}

	/* wait timer to elapse */
	cli();
	if (nvram_timer != 0){
	    sei();
	    return;
	}
	sei();


	putl_P(PSTR("save\n"));
	blink();

	nvram_write_data();

	ring_status.dirty = 0;
	ring_status.write_scheduled = 0;
    }
}
