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
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>

#include "xlibc.h"

#if DEBUG
/******************/
/*      UART      */
/******************/
static int uart_trygetchar()
{
    if ((UCSR0A & (1<<RXC0)) == 0)
	return _FDEV_EOF;

    if (UCSR0A & _BV(FE0)){
	UCSR0A &= ~(_BV(FE0)|_BV(DOR0));
	return _FDEV_EOF;
    }

    if (UCSR0A & _BV(DOR0)){
	UCSR0A &= ~(_BV(FE0)|_BV(DOR0));
	return _FDEV_ERR;
    }

    return UDR0;
}

static int uart_putchar(char c, FILE *unused)
{
    if (c == '\n'){
	loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
	UDR0 = '\r';
    }
    loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
    UDR0 = c;
    return c;
}
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void putn(int n)
{
    char buf[12];
    int i;
    itoa(n, buf, 16);

    for (i=0;buf[i];i++)
	uart_putchar(buf[i],0);
    uart_putchar(' ', 0);
}

void putl_P(const char *s)
{
    int i;

    for (i=0;pgm_read_byte_near(&s[i]);i++)
	uart_putchar(pgm_read_byte_near(&s[i]),0);
//    uart_putchar('\n', 0);
}


void libcinit()
{
    stdout = &mystdout;
}
#endif
