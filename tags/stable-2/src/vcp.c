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
#include <stdint.h>
#include "vcp.h"

#define VCP ("vcp(10) type(LCD) mccs_ver(1.1)")

const PROGMEM char vcp_features_P[sizeof(VCP)-1] = VCP;

#ifdef PROVIDE_EDID
const PROGMEM char monitor_edid_P[EDID_SIZE] = {
	0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,	0x04,0x68,0xFA,0x00,0x00,0x00,0x00,0x00,
	0x28,0x15,0x01,0x03,0xA5,0x3C,0x22,0x78,	0x22,0x6F,0xB1,0xA7,0x55,0x4C,0x9E,0x25,
	0x0C,0x50,0x54,0x00,0x00,0x00,0x01,0x01,	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	0x01,0x01,0x01,0x01,0x01,0x01,0x56,0x5E,	0x00,0xA0,0xA0,0xA0,0x29,0x50,0x30,0x20,
	0x35,0x00,0x55,0x50,0x21,0x00,0x00,0x1A,	0x00,0x00,0x00,0xFC,0x00,0x51,0x48,0x44,
	0x32,0x37,0x30,0x0A,0x20,0x20,0x20,0x20,	0x20,0x20,0x00,0x00,0x00,0xFC,0x00,0x51,
	0x48,0x44,0x32,0x37,0x30,0x0A,0x20,0x20,	0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xFC,
	0x00,0x51,0x48,0x44,0x32,0x37,0x30,0x0A,	0x20,0x20,0x20,0x20,0x20,0x20,0x01,0xB0,
};
#endif

struct vcp_handler vcp_handlers[VCP_HANDLERS_COUNT] = {
    [VCPH_LUMINANCE_INDEX] = {
	.opcode = VCPH_LUMINANCE,
	.max = VCPH_LUMINANCE_MAX,
	.value = 50,
    },
};
