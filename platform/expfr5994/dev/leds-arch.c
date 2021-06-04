/*
 * Copyright (c) 2018, Bruno Kessler Foundation, Trento, Italy and
 * ETH, IIS, Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -----------------------------------------------------------------
 *
 *\original author
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 * Created : 2018-02-02
 *\modified by
 *         João Gabriel Pazinato de Bittencourt <joaogabrielpazinatobittencourt@gmail.com>
 */

#include "contiki-conf.h"
#include "dev/leds.h"

/* LED ports for MSP430FR5994 Launchpad */
#define LEDS_CONF_RED    BIT0
#define LEDS_CONF_GREEN  BIT1

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  /* only two LEDS are currently available on the platform, configured as RED and GREEN */
  P1DIR |= (LEDS_CONF_RED | LEDS_CONF_GREEN);
}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return ((P1OUT & LEDS_CONF_RED) ? LEDS_RED : 0)
    | ((P1OUT & LEDS_CONF_GREEN) ? LEDS_GREEN : 0);
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
  P1OUT = (P1OUT & ~(LEDS_CONF_RED | LEDS_CONF_GREEN))
  | ((leds & LEDS_RED) ? LEDS_CONF_RED : 0)
  | ((leds & LEDS_GREEN) ? LEDS_CONF_GREEN : 0);
}
/*---------------------------------------------------------------------------*/
