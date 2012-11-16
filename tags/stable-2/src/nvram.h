#ifndef NVRAM_H
#define NVRAM_H

#include <stdint.h>
#include "config.h"

#define NVR_LUMINANCE (0)
#define NVR_DATA_COUNT (1)

extern char nvram_timer;

void nvram_init();

char nvram_get(uint8_t ctl, uint8_t *value);
void nvram_set(uint8_t ctl, uint8_t value);
void nvram_save();

#endif /* NVRAM_H */

