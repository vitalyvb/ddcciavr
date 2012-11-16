#ifndef TWI_PROTO_H
#define TWI_PROTO_H

#include <stdint.h>

#define PROTO_SEND_NOTHING (0xff)

#define VCP_GET_FEATURE_REPLY_RES_OK (0)
#define VCP_GET_FEATURE_REPLY_RES_UNK (1)

#define VCP_GET_FEATURE_REPLY_SET (0)
#define VCP_GET_FEATURE_REPLY_MOMENT (1)


struct vcp_get_caps {
    uint8_t offs_h;
    uint8_t offs_l;
};

struct vcp_set_features {
    uint8_t opcode;
    uint8_t value_h;
    uint8_t value_l;
};

struct vcp_get_features_req {
    uint8_t opcode;
};

struct vcp_get_features_reply {
    uint8_t result_code; // 0 - no error, 1 - unsupported vcp
    uint8_t opcode;
    uint8_t vcp_type; // 0 - set parameter, 1 - momentary
    uint8_t maximum_h;
    uint8_t maximum_l;
    uint8_t present_h;
    uint8_t present_l;
};

union vcp_buffer {
    struct vcp_get_caps vcp_get_caps;
    struct vcp_set_features vcp_set_features;
    struct vcp_get_features_req vcp_get_features_req;
    struct vcp_get_features_reply vcp_get_features_reply;
    uint8_t data[1];
};

void twi_proto_start_read(uint8_t i2c_addr);
void twi_proto_start_write(uint8_t i2c_addr);
void twi_proto_stop_write();
void twi_proto_stop();

void twi_proto_recvbyte(uint8_t chr);
uint8_t twi_proto_get_sendbyte();

#endif /* TWI_PROTO_H */
