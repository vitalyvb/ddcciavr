#ifndef DEBUG_H
#define DEBUG_H

#if DEBUG
void main_trace(uint8_t val);
 #define TRACE(x) main_trace(x)
#else
 #define TRACE(x)
#endif

#endif /* LED_H */

