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
#include <avr/pgmspace.h>
#include "debug.h"
#include "twi_proto.h"
#include "vcp.h"

/****************************************/

#define CMD_NOOP (0x00)

#define CMD_GET_VCP_FEATURE (0x01)
#define CMD_SET_VCP_FEATURE (0x03)
#define CMD_GET_CAPS (0xf3)
#define CMD_SAVE_SETTINGS (0x0c)

#define CHECKSUM_OK (0)

/****************************************/

#define STATE_INVALID (0)
#define STATE_READ (1)
#define STATE_WRITE (2)
#define STATE_ERROR (3)

/****************************************/

#define SUBSTATE_INIT (0x00)
#define SUBSTATE_END (0xfe)
#define SUBSTATE_ERR (0xff)

#define SUBSTATE_R_MEM (SUBSTATE_INIT)
#define SUBSTATE_R_MEM_LEN (SUBSTATE_R_MEM+1)
#define SUBSTATE_R_MEM_CMD (SUBSTATE_R_MEM+2)
#define SUBSTATE_R_MEM_RECVR (SUBSTATE_R_MEM+3)

#define SUBSTATE_R_MEM_EDID (SUBSTATE_R_MEM+4)

#define SUBSTATE_WR_MEM (SUBSTATE_INIT+0x10)
#define SUBSTATE_WR_MEM_LEN (SUBSTATE_WR_MEM+1)
#define SUBSTATE_WR_MEM_CMD (SUBSTATE_WR_MEM+2)
#define SUBSTATE_WR_MEM_SENDR (SUBSTATE_WR_MEM+3)
#define SUBSTATE_WR_MEM_SENDP (SUBSTATE_WR_MEM+4)
#define SUBSTATE_WR_MEM_CHK (SUBSTATE_WR_MEM+5)
#define SUBSTATE_WR_MEM_EOF (SUBSTATE_WR_MEM+6)

#define SUBSTATE_WR_MEM_EDID (SUBSTATE_WR_MEM+7)

/****************************************/

static char state, substate;

static char recv_xor, recv_len, recv_cmd;

static union vcp_buffer buffer;
static uint8_t buffer_ptr;


static uint8_t send_cmd;

static uint8_t *send;
static uint8_t send_ptr;
static uint8_t send_len;

static const uint8_t *send_pmem;
static uint8_t send_pmem_ptr;
static uint8_t send_pmem_len;

static uint8_t send_xor;

/****************************************/

void twi_proto_start_read(uint8_t i2c_addr)
{
    state = STATE_READ;
    substate = SUBSTATE_R_MEM;

#ifdef PROVIDE_EDID
    if (i2c_addr == 0xa0){
	substate = SUBSTATE_R_MEM_EDID;
    }
#endif

    TRACE(1); TRACE(i2c_addr);
}

void twi_proto_start_write(uint8_t i2c_addr)
{
    if (state == STATE_ERROR){
	/* read error data, do nothing on write */
	substate = SUBSTATE_WR_MEM_EOF;
	TRACE(3);
	return;
    }

    state = STATE_WRITE;
    substate = SUBSTATE_WR_MEM;

#ifdef PROVIDE_EDID
    if (i2c_addr == 0xa1){
	substate = SUBSTATE_WR_MEM_EDID;
    }
#endif

    TRACE(2); TRACE(i2c_addr);
}

void twi_proto_stop_write()
{
    state = STATE_INVALID;
    substate = SUBSTATE_INIT;
    TRACE(4);
}

void twi_proto_stop()
{
    state = STATE_INVALID;
    substate = SUBSTATE_INIT;
    TRACE(5);
}

static void twi_proto_message_received();

static void twi_proto_set_err()
{
    state = STATE_ERROR;
    substate = SUBSTATE_ERR;
    TRACE(6);
}

void twi_proto_recvbyte(uint8_t chr)
{

    if (state != STATE_READ){
	TRACE(7);
	return;
    }

    switch (substate){
#ifdef PROVIDE_EDID
    case SUBSTATE_R_MEM_EDID:
	recv_cmd = chr;

	send_pmem = monitor_edid_P;
	send_pmem_ptr = 0;
	send_pmem_len = 128;

	substate = SUBSTATE_WR_MEM_EDID;
	break;
#endif

    case SUBSTATE_INIT: /* receive prefix */

	if (chr != 0x51){
	    TRACE(8);
	    twi_proto_set_err();
	    return;
	}
	TRACE(9);
	substate = SUBSTATE_R_MEM_LEN;
	recv_xor = 0x6e ^ 0x51;
	break;
    
    case SUBSTATE_R_MEM_LEN: /* receive length byte */

	if ((chr & 0x80) == 0){
	    TRACE(10);
	    twi_proto_set_err();
	    return;
	}
	recv_len = chr & 0x7f;
	if (recv_len == 0){
	    /* null message */
	    recv_cmd = CMD_NOOP;
	    substate = SUBSTATE_R_MEM_RECVR;
	} else if (recv_len > sizeof(buffer)){
	    TRACE(11);
	    twi_proto_set_err();
	    return;
	} else {
	    substate = SUBSTATE_R_MEM_CMD;
	}
	TRACE(12);
	recv_xor ^= chr;

	break;

    case SUBSTATE_R_MEM_CMD: /* receive command byte */

	TRACE(13);

	buffer_ptr = 0;
	recv_cmd = chr;
	recv_xor ^= chr;
	recv_len--;
	substate = SUBSTATE_R_MEM_RECVR;
	break;

    case SUBSTATE_R_MEM_RECVR: /* receive data byte */

	if (recv_len == 0){
	    /* checksumm */
	    recv_xor ^= chr;
	    if (recv_xor != CHECKSUM_OK){
		TRACE(14);
		twi_proto_set_err();
		return;
	    }

	    substate = SUBSTATE_END;
	    TRACE(15);
	    twi_proto_message_received();

	} else {
	    /* data */
	    TRACE(16);
	    buffer.data[buffer_ptr++] = chr;
	    recv_xor ^= chr;
	    recv_len--;
	}

	break;
    case SUBSTATE_END:
    case SUBSTATE_ERR:
	/* no op */
	TRACE(17);
	break;
    }
}

void twi_proto_message_received()
{
    switch (recv_cmd){
    case CMD_GET_VCP_FEATURE:

	if (buffer_ptr == sizeof(struct vcp_get_features_req)){
	    uint8_t i, found=0;
	    uint8_t opcode;
	    struct vcp_get_features_req *req = &buffer.vcp_get_features_req;

	    TRACE(18);
	    opcode = req->opcode;

	    for (i=0;i<VCP_HANDLERS_COUNT;i++){
		if (vcp_handlers[i].opcode == opcode){
		    struct vcp_get_features_reply *reply = &buffer.vcp_get_features_reply;
		    found = 1;

		    substate = SUBSTATE_WR_MEM;
		    send_cmd = 0x02;

		    reply->result_code = VCP_GET_FEATURE_REPLY_RES_OK;
		    reply->opcode = opcode;
		    reply->vcp_type = VCP_GET_FEATURE_REPLY_MOMENT;
		    reply->maximum_h = vcp_handlers[i].max >> 8;
		    reply->maximum_l = vcp_handlers[i].max & 0xff;
		    reply->present_h = vcp_handlers[i].value >> 8;
		    reply->present_l = vcp_handlers[i].value & 0xff;

		    send = buffer.data;
		    send_ptr = 0;
		    send_len = sizeof(struct vcp_get_features_reply)+1;

		    send_pmem = 0;
		    send_pmem_ptr = 0;
		    send_pmem_len = 0;

		    break;
		}
	    }

	    if (!found){
		struct vcp_get_features_reply *reply = &buffer.vcp_get_features_reply;

		substate = SUBSTATE_WR_MEM;
		send_cmd = 0x02;

		reply->result_code = VCP_GET_FEATURE_REPLY_RES_UNK;
		reply->opcode = opcode;
		reply->vcp_type = VCP_GET_FEATURE_REPLY_MOMENT;
		reply->maximum_h = 0xff;
		reply->maximum_l = 0xff;
		reply->present_h = 0;
		reply->present_l = 0;

		send = buffer.data;
		send_ptr = 0;
		send_len = sizeof(struct vcp_get_features_reply)+1;

		send_pmem = 0;
		send_pmem_ptr = 0;
		send_pmem_len = 0;
		TRACE(19);
	    }

	} else {
	    TRACE(20);
	}
	break;

    case CMD_SET_VCP_FEATURE:

	if (buffer_ptr == sizeof(struct vcp_set_features)){
	    uint8_t i;
	    uint8_t opcode;
	    unsigned int value;
	    struct vcp_set_features *req = &buffer.vcp_set_features;

	    TRACE(21);
	    opcode = req->opcode;
	    value = ((unsigned int)req->value_h << 8) | req->value_l;

	    for (i=0;i<VCP_HANDLERS_COUNT;i++){
		if (vcp_handlers[i].opcode == opcode){
		    if (value <= vcp_handlers[i].max){
			vcp_handlers[i].value = value;
			vcp_handlers[i].dirty = 1;
		    }
		    break;
		}
	    }
	} else {
	    TRACE(22);
	}

	substate = SUBSTATE_WR_MEM_EOF;

	break;

    case CMD_GET_CAPS:

	if (buffer_ptr == sizeof(struct vcp_get_caps)){
	    unsigned int offs;
	    struct vcp_get_caps *ofs = &buffer.vcp_get_caps;

	    offs = (ofs->offs_h << 8) | ofs->offs_l;

	    ofs->offs_h = offs >> 8;
	    ofs->offs_l = offs & 0xff;

	    substate = SUBSTATE_WR_MEM;
	    send_cmd = 0xe3;

	    send = buffer.data;
	    send_ptr = 0;
	    send_len = 2+1;

	    send_pmem = &vcp_features_P[offs];
	    send_pmem_ptr = 0;
	    if (MY_VCP_SIZE > offs)
		send_pmem_len = MY_VCP_SIZE-offs;
	    else
		send_pmem_len = 0;

	    if (send_pmem_len > 0x32)
		send_pmem_len = 0x32;

	    TRACE(23);
	    break;
	} else {
	    TRACE(24);
	}
	/* fall through */
    default:
	substate = SUBSTATE_WR_MEM;

	send_cmd = 0;

	send = 0;
	send_ptr = 0;
	send_len = 0;

	send_pmem = 0;
	send_pmem_ptr = 0;
	send_pmem_len = 0;

	TRACE(25);

	break;
    }

}



uint8_t twi_proto_get_sendbyte()
{
    char res = PROTO_SEND_NOTHING;

    if (state != STATE_WRITE){
	TRACE(26);
	return res;
    }

    /* send answer */
    switch (substate){
    case SUBSTATE_WR_MEM:
	/* prefix */
	TRACE(27);
	res = 0x6e;
	send_xor = 0x50 ^ res;
	substate = SUBSTATE_WR_MEM_LEN;
	break;

    case SUBSTATE_WR_MEM_LEN:
	/* length */
	res = 0x80 | (send_len+send_pmem_len);
	send_xor ^= res;

	if (send_len > 0 || send_pmem_len > 0)
	    substate = SUBSTATE_WR_MEM_CMD;
	else
	    substate = SUBSTATE_WR_MEM_CHK;
	TRACE(28);
	break;

    case SUBSTATE_WR_MEM_CMD:
	/* send command byte */
	res = send_cmd;
	send_xor ^= res;

	if (send_len > 0){
	    send_len--;
	    substate = SUBSTATE_WR_MEM_SENDR;
	} else if (send_pmem_len > 0){
	    send_pmem_len--;
	    substate = SUBSTATE_WR_MEM_SENDP;
	}

	TRACE(29);
	break;

    case SUBSTATE_WR_MEM_SENDR:
	/* data from ram */
	res = send[send_ptr++];
	send_xor ^= res;

	if (send_ptr == send_len){
	    if (send_pmem_len > 0)
		substate = SUBSTATE_WR_MEM_SENDP;
	    else
		substate = SUBSTATE_WR_MEM_CHK;
	}

	TRACE(30);
	break;

    case SUBSTATE_WR_MEM_SENDP:
	/* data from flash */
	res = pgm_read_byte_near(&send_pmem[send_pmem_ptr++]);
	send_xor ^= res;

	if (send_pmem_ptr == send_pmem_len)
	    substate = SUBSTATE_WR_MEM_CHK;

	TRACE(31);
	break;

    case SUBSTATE_WR_MEM_CHK:
	/* send checksum */
	substate = SUBSTATE_WR_MEM_EOF;
	TRACE(32);
	return send_xor;

#ifdef PROVIDE_EDID
    case SUBSTATE_WR_MEM_EDID:
	res = pgm_read_byte_near(&send_pmem[send_pmem_ptr++]);
	if (send_pmem_ptr == send_pmem_len)
	    substate = SUBSTATE_WR_MEM_EOF;
	break;
    case SUBSTATE_WR_MEM_EOF:
#endif
    default:
	TRACE(33);
	return PROTO_SEND_NOTHING;
    }

    return res;
}

