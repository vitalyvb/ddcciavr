#ifndef XLIBC_H
#define XLIBC_H

#include <stdint.h>
#include "config.h"

#if DEBUG
void libcinit();

void putn(int n);
void putl_P(const char *s);
#else
#define libcinit()
#define putn(x)
#define putl_P(x)
#endif

#endif /* XLIBC_H */

