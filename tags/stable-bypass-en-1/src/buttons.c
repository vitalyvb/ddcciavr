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
#include "buttons.h"

char button_clock;

struct btn_descr {
    uint8_t clicked:1;
    uint8_t repeat:1;
    uint8_t unused:6;
    uint8_t cnt;
};

static struct btn_descr button[BUTTONS_COUNT];

char button_down(uint8_t btn)
{
    struct btn_descr *b = &button[btn];
    char res = 0;

    if (b->clicked){
	uint8_t delay;

	if (b->cnt > button_clock){
	    delay = 0xff - b->cnt + button_clock + 1;
	} else {
	    delay = button_clock - b->cnt;
	}


	if (b->repeat){
	    if (delay >= ((1000/BUTTON_REPEAT_HZ)/(1000/TICK_HZ))){
		res = 1;
		b->cnt = button_clock;
	    }
	} else {
	    if (delay >= ((BUTTON_REPEAT_DELAY + (1000/TICK_HZ-1))/(1000/TICK_HZ))){
		res = 1;
		b->repeat = 1;
		b->cnt = button_clock;
	    }
	}

    } else {
	b->cnt++;
	if (b->cnt == 0xff){
	    b->clicked = 1;
	    res = 1;
	    b->cnt = button_clock;
	}
    }

    return res;
}

void button_up(uint8_t btn)
{
    struct btn_descr *b = &button[btn];

    b->cnt = 0;
    b->repeat = 0;
    b->unused = 0;
    b->clicked = 0;
}
