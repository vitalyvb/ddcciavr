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
#include <avr/sfr_defs.h>
#include <compat/twi.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include "twi_slave.h"
#include "vcp.h"
#include "buttons.h"
#include "nvram.h"
#include "led.h"
#include "xlibc.h"
#include "pwm_table.h"
#include "debug.h"

#include <stdio.h>
#include <stdlib.h>

#define NOP asm volatile ("nop");				// skip one clock cycle


#if DEBUG
static volatile char tick;
volatile int tick2;
#endif

static char controls_enabled();

static inline void ioinit()
{

    /* - - - - - - */
    /*  INIT  I/O  */
    /* - - - - - - */
    DDRB = 0;
    DDRD = 0;

    /* OC0A PWM output */
    PORTD &= ~_BV(PD6); /* low */
    DDRD |= _BV(PD6); /* output */

    /* PB0 output - LED */
    PORTB &= ~_BV(PB0); /* low */
    DDRB |= _BV(PB0);

    /* EN_OUT */
    PORTD &= ~_BV(PD5); /* low */
    DDRD |= _BV(PD5);

    /* buttons input */
    DDRD &= ~(_BV(PD2)|_BV(PD4)); /* input */
    PORTD |= (_BV(PD2)|_BV(PD4)); /* pull-up */

    /* EN_IN */
    DDRD &= ~_BV(PD3); /* input */
    PORTD &= ~_BV(PD3); /* no pull-up */

    /* - - - - - - */
    /* INIT USART  */
    /* - - - - - - */
#if DEBUG
 #define BAUD 9600
 #include <util/setbaud.h>
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
 #if USE_2X
    UCSR0A |= _BV(U2X0);
 #else
    UCSR0A &= ~(_BV(U2X0));
 #endif
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
#endif


    /* - - - - - - - - - */
    /* pre-INIT PWM      */
    /* - - - - - - - - - */
    TCCR0B = (1<<CS00); /* no prescaler */
    TIMSK0 = 0; /* no interrupts */
    OCR0A = INITIAL_PWM;


    /* - - - - - - */
    /* CLOCK TIMER */
    /* - - - - - - */
    TCNT1H = 0; /* Clear timer */
    TCNT1L = 0;

    TCCR1A = 0; /* Configure timer 1 for CTC mode */
    TCCR1B |= (1 << WGM12);
    TIMSK1 |= (1 << OCIE1A); /* Enable CTC interrupt */

    OCR1A   = (F_CPU/64/TICK_HZ);
    TCCR1B |= ((1 << CS10) | (1 << CS11)); /* Start timer at Fcpu/64 */

    /* - - - - - - */
    /* INIT  I2C   */
    /* - - - - - - */
    TWI_Slave_Initialise(DCCCI_ADDRESS);
}


/******************/
/*       LED      */
/******************/
static volatile char blink_status;

static inline void led_on()
{
    PORTB |= _BV(PB0);
}

static inline void led_off()
{
    PORTB &= ~_BV(PB0);
}

void blink()
{
    cli();
    if (blink_status == 0)
	blink_status = 1;
    sei();
}

/******************/
/*  EN_IN  EN_OUT */
/******************/

static inline char get_en_in()
{
#ifdef SET_ALWAYS_EN
    return TRUE;
#else
    return ((PIND & _BV(PD3)) != 0);
#endif
}

static inline void set_en_out()
{
    PORTD |= _BV(PD5);
}

static inline void clear_en_out()
{
    PORTD &= ~_BV(PD5);
}

/***********************/
/*  PWM and luminance  */
/***********************/
static uint8_t ctl_luminance;

static void set_pwm(uint8_t value)
{
    OCR0A = value;
}

static void pwm_start()
{
    PORTD &= ~_BV(PD6); /* low */

    TCNT0 = 0; //clear timer
    /* fast pwm mode 3 */
    TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<WGM01) | (1<<WGM00); /* Clear OC0A on compare match, set OC0A at BOTTOM */
    DDRD |= _BV(PD6); /* output */
}

static void pwm_stop()
{
    TCCR0A = 0; /* disable pwm */
    PORTD &= ~_BV(PD6); /* low */
}

static void lumunance_set_int(uint8_t value)
{
    if (value <= VCPH_LUMINANCE_MAX){
	ctl_luminance = value;
	VCP_VALUE_INTERNAL_SET(VCPH_LUMINANCE_INDEX, value);

	value = pgm_read_byte_near(&pwm_curve_P[value]);
	set_pwm(value);
    }
}

void lumunance_set(uint8_t value)
{
    blink();
    nvram_set(NVR_LUMINANCE, value);

    lumunance_set_int(value);
}

void lumunance_inc()
{
    if (ctl_luminance < VCPH_LUMINANCE_MAX)
	lumunance_set(ctl_luminance+1);
}

void lumunance_dec()
{
    if (ctl_luminance > 0)
	lumunance_set(ctl_luminance-1);
}

/******************/
/*  VCP handlers  */
/******************/
static uint8_t vcp_check_idx;

void handle_vcp()
{

    cli();
    if (vcp_handlers[vcp_check_idx].dirty){
	uint8_t value;
	value = vcp_handlers[vcp_check_idx].value;
	vcp_handlers[vcp_check_idx].dirty = 0;
	sei();

	switch (vcp_check_idx){
	case VCPH_LUMINANCE_INDEX:
	    lumunance_set(value);
	    break;
	}

    } else {
	sei();
    }

    vcp_check_idx++;
    if (vcp_check_idx == VCP_HANDLERS_COUNT)
	vcp_check_idx = 0;
}

/******************/
/*    Buttons     */
/******************/

#define BUTTON1_CLICK (1<<0)
#define BUTTON2_CLICK (1<<1)

#define ANY_BUTTON (BUTTON1_CLICK|BUTTON2_CLICK)

static char buttons_status()
{
    char res = 0;

    if (!controls_enabled()){
	button_up(0);
	button_up(1);
	return res;
    }

    if ((PIND & _BV(PD2)) == 0){
	res |= button_down(0) << 0;
    } else {
	button_up(0);
    }

    if ((PIND & _BV(PD4)) == 0){
	res |= button_down(1) << 1;
    } else {
	button_up(1);
    }

    return res;
}

void restore()
{
    uint8_t tmp;

    /* skip settings restore if any button is pressed */
    if ((PIND & _BV(PD2)) == 0)
	return;
    if ((PIND & _BV(PD4)) == 0)
	return;

    if (nvram_get(NVR_LUMINANCE, &tmp)){
	if (tmp <= VCPH_LUMINANCE_MAX)
	    lumunance_set_int(tmp);
    }
}

/******************/
/*  EN_ logic    */
/******************/

/*
Should work as follows:

       | Slow start      | PWM disable delayed  | Fast resume
===============================================================
       |     /---------- | --\                  |     /-------
EN_IN  |    /            |    \                 |    /
       | __/             |     \_______________ | --/
===============================================================
       |            /--- | ---\                 |      /-----
EN_OUT |           /     |     >                |     /
       |----------/      | ___/ \______________ | ---/
===============================================================
       |      /--------- | ---------------\     | -----------
PWM_EN |     /           |                 \    |
       | ---/            |                  \__ |
===============================================================
             |-----|           |-----------|
           PWM settle            PWM keep
*/

#define EN_STOPPED (0)
#define EN_DEBOUNCE1 (EN_STOPPED+1)
#define EN_DEBOUNCE2 (EN_DEBOUNCE1+1)
#define EN_DEBOUNCE3 (EN_DEBOUNCE2+1)
#define EN_STARTING_PWM_WAIT (EN_DEBOUNCE3+1)
#define EN_STARTED (EN_STARTING_PWM_WAIT+1)
#define EN_STOPPING_PWM_KEEP (EN_STARTED+1)

#define PWM_SETTLE_TICKS ((PWM_SETTLE_DELAY+TICK_HZ-1)/TICK_HZ)
#define PWM_KEEP_TICKS ((PWM_KEEP_DELAY+TICK_HZ-1)/TICK_HZ)

static char status_en;
static volatile uint16_t status_en_timer;

static void set_status_en_timer(const uint16_t t)
{
    cli();
    status_en_timer = t;
    sei();
}

static char controls_enabled()
{
    return (status_en == EN_STARTED);
}

static void handle_en_in()
{
    char en_in = get_en_in();

    switch (status_en){
    case EN_STOPPED:
    case EN_DEBOUNCE1:
    case EN_DEBOUNCE2:
    case EN_DEBOUNCE3:
	if (en_in){
	    if (status_en != EN_DEBOUNCE3){
		status_en++;
		return;
	    }
	    pwm_start();
	    set_status_en_timer(PWM_SETTLE_TICKS);
	    status_en = EN_STARTING_PWM_WAIT;
	    TRACE(0x40);
	} else {
	    status_en = EN_STOPPED;
	}
	break;
    case EN_STARTING_PWM_WAIT:
	if (en_in){
	    if (status_en_timer == 0){
		set_en_out();
		status_en = EN_STARTED;
		TRACE(0x41);
	    }
	} else {
	    clear_en_out();
	    set_status_en_timer(PWM_KEEP_TICKS);
	    status_en = EN_STOPPING_PWM_KEEP;
	    TRACE(0x42);
	}
	break;
    case EN_STARTED:
	if (!en_in){
	    clear_en_out();
	    set_status_en_timer(PWM_KEEP_TICKS);
	    status_en = EN_STOPPING_PWM_KEEP;
	    TRACE(0x43);
	}
	break;
    case EN_STOPPING_PWM_KEEP:
	if (en_in){
	    set_en_out();
	    status_en = EN_STARTED;
	    TRACE(0x44);
	} else {
	    if (status_en_timer == 0){
		pwm_stop();
		status_en = EN_STOPPED;
		TRACE(0x45);
	    }
	}
	break;
    default:
	/* wtf? */
	TRACE(0x46);
	putl_P(PSTR("en err"));
    };
}

/******************/
/*  DEBUG Helpers */
/******************/

#if DEBUG
static uint8_t trace_data[0x80];
static uint8_t trace_tail, trace_head;
void main_trace(uint8_t val)
{
    trace_data[trace_tail] = val;
    trace_tail = (trace_tail + 1) & 0x7f;
}

void show_trace()
{
    if (trace_tail == trace_head)
	return;

    putl_P(PSTR("!> "));
    cli();
    while (trace_tail != trace_head){
	sei();
	putn(trace_data[trace_head]);
	trace_head = (trace_head + 1) & 0x7f;
	cli();
    }
    sei();
    putl_P(PSTR("<!\n"));
}

#endif

/******************/
/*  Main loop     */
/******************/

int main()
{
    char buttons;

#if !DEBUG
    wdt_enable(0x4); /* 32K cycles, 0.25s */
#endif

    ioinit();
    libcinit();

    sei();

    lumunance_set_int(DEFAULT_LUMINANCE);

    wdt_reset();

    nvram_init();

    restore();

    blink();
    putl_P(PSTR("Hello World!\n"));

    TWI_Start_Transceiver();

    wdt_reset();

    while (1){

	buttons = buttons_status();

	if (buttons == BUTTON1_CLICK){
	    lumunance_inc();
	    putl_P(PSTR("btn+ "));
	    putn(ctl_luminance);
	    putl_P(PSTR("\n"));
	}

	if (buttons == BUTTON2_CLICK){
	    lumunance_dec();
	    putl_P(PSTR("btn- "));
	    putn(ctl_luminance);
	    putl_P(PSTR("\n"));
	}

	handle_vcp();

	nvram_save();

	handle_en_in();

#if DEBUG
	show_trace();

	if (tick == (TICK_HZ-1)){
	    puts_P(PSTR("."));
	    tick = 0;

	    if (tick2){
		putl_P(PSTR("----> "));
		putn(tick2);
//		tick2 = 0;
		putl_P(PSTR("\n"));
	    }
	}
#endif

	wdt_reset();
    }
}

/******************/
/*  Clock timer   */
/******************/

ISR(TIMER1_COMPA_vect)
{
    if (blink_status == 0){
	/* do nothing */
    } else if (blink_status == 1){
	led_on();
	blink_status = 2;
    } else if (blink_status >= (LED_BLINK_TIME/(1000/TICK_HZ))){
	led_off();
	blink_status = 0;
    } else {
	blink_status++;
    }
#if DEBUG
    tick++;
#endif
    button_clock++;

    if (nvram_timer > 0)
	nvram_timer--;

    if (status_en_timer > 0)
	status_en_timer--;
}
