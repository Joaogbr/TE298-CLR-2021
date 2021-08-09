/*
 * Copyright (c) 2011, Swedish Institute of Computer Science
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         MSP430-specific rtimer code for MSP430X (ported from MSP430-specific rtimer code, author: Adam Dunkels).
 * \Contiki port, author
 *         Andrea Gaglione <and.gaglione@gmail.com>
 *         David Rodenas-Herraiz <dr424@cam.ac.uk>
 *
 */

#include "contiki.h"
#include "sys/energest.h"
#include "sys/rtimer.h"
#include "sys/process.h"
#include "dev/watchdog.h"
#include "isr_compat.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static volatile uint8_t timeout = 0;

/*---------------------------------------------------------------------------*/
ISR(TIMER1_A0, timer1a0)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  watchdog_start();

  rtimer_run_next();

  if(process_nevents() > 0) {
    LPM4_EXIT;
  }

  watchdog_stop();

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
ISR(TIMER0_A0, timer0a0)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  LPM4_EXIT;

  timeout = 1;

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  dint();

  /* CCR0 interrupt enabled, interrupt occurs when timer equals CCR0. */
  TA1CCTL0 = CCIE;
  TA0CCTL0 = CCIE;
  /* Enable interrupts. */
  eint();
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_now(void)
{
  rtimer_clock_t t1, t2;
  do {
    t1 = TA1R;
    t2 = TA1R;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  PRINTF("rtimer_arch_schedule time %u\n", t);

  TA1CCR0 = t;
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_sleep(rtimer_clock_t howlong)
{
#if RTIMER_ARCH_SECOND == 32768
  howlong = (howlong/2); // Timer A0 has 16384 Hz, so cut wait period in half
#endif

  TA0CCR0 = howlong;
  timeout = 0;

  watchdog_stop();

  ENERGEST_SWITCH(ENERGEST_TYPE_CPU, ENERGEST_TYPE_LPM);

  /* Start Timer0_A in up mode. */
  TA0CTL |= MC_1;

  _BIS_SR(GIE | LPM4_bits);
	while(timeout == 0);

  /* Stop Timer0_A. */
  TA0CTL &= ~MC;

  ENERGEST_SWITCH(ENERGEST_TYPE_LPM, ENERGEST_TYPE_CPU);

  /* Clear Timer0_A */
  TA0CTL |= TACLR;

  watchdog_start();
}
/*---------------------------------------------------------------------------*/
int
rtimer_arch_sleep_until(rtimer_clock_t howlong, bool *cond_var)
{
  TA0CCR0 = howlong;
  timeout = 0;

  watchdog_stop();

  ENERGEST_SWITCH(ENERGEST_TYPE_CPU, ENERGEST_TYPE_LPM);

  /* Start Timer0_A in up mode. */
  TA0CTL |= MC_1;

  _BIS_SR(GIE | LPM4_bits);
  while((*cond_var == false) && (timeout == 0)){
    _NOP();
  }

  /* Stop Timer0_A. */
  TA0CTL &= ~MC;

  ENERGEST_SWITCH(ENERGEST_TYPE_LPM, ENERGEST_TYPE_CPU);

  /* Clear Timer0_A */
  TA0CTL |= TACLR;

  watchdog_start();

  if(timeout == 1){
    PRINTF("Rtimer arch timeout\n");
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
