#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define F_CPU 8000000UL
#define DEBUG 0

#define TICK_HZ (30)

#define BUTTON_REPEAT_DELAY (250) /* ms */
#define BUTTON_REPEAT_HZ (30)

#define NVR_SAVE_DELAY (2000) /* ms */

#define DEFAULT_LUMINANCE (50)
//#define PROVIDE_EDID

#define DCCCI_ADDRESS (0x37)
#define EDID_ADDRESS (0x50)

#define PWM_SETTLE_DELAY (100) /* ms */
#define PWM_KEEP_DELAY (10000) /* ms */

#define LED_BLINK_TIME (100) /* ms */

//#define SET_ALWAYS_EN

#endif /* CONFIG_H */

