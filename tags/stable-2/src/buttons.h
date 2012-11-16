#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdint.h>
#include "config.h"

#define BUTTONS_COUNT (2)

extern char button_clock;

char button_down(uint8_t btn);
void button_up(uint8_t btn);

#endif /* BUTTONS_H */

