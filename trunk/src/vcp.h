#ifndef VCP_H
#define VCP_H

#include <stdint.h>
#include <avr/pgmspace.h>

#define VCP ( "(vcp(10) type(LCD) mccs_ver(1.1))" )

struct vcp_handler {
    uint8_t dirty:1;
    uint8_t _unused:7;

    uint8_t opcode;
    uint16_t value;
    uint16_t max;
};

#define VCP_VALUE_INTERNAL_SET(ivcp, val) do { \
	vcp_handlers[ivcp].value = (val);\
	vcp_handlers[ivcp].dirty = 0;\
	vcp_handlers[ivcp]._unused = 0;\
    } while (0)

#define VCPH_LUMINANCE (0x10)
#define VCPH_LUMINANCE_INDEX (0x0)
#define VCPH_LUMINANCE_MAX (99)

#define VCP_HANDLERS_COUNT (1)
extern struct vcp_handler vcp_handlers[VCP_HANDLERS_COUNT];

#define MY_VCP_SIZE (sizeof(VCP)-1)
extern const PROGMEM char vcp_features_P[MY_VCP_SIZE];

#ifdef PROVIDE_EDID
#define EDID_SIZE (128)
extern const PROGMEM char monitor_edid_P[EDID_SIZE];
#endif

#endif /* VCP_H */
