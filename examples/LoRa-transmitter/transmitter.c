
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
 *
 *  transmitter.c
 *  Simple LoRa packet transmitter example in Contiki (bare Radio TX)
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-28
 */

#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h>
#include "sx1276.h"
#include "sx1276-arch.h"
#include "spi.h"
/*---------------------------------------------------------------------------*/
#define LEDS_DEBUG 1
#if LEDS_DEBUG
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#define LEDS_TOGGLE(x) leds_toggle(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#define LEDS_TOGGLE(x)
#endif
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif
/*---------------------------------------------------------------------------*/
//#define CLEAR_TXBUF()           (buffer = 0)
/*---------------------------------------------------------------------------*/
static struct etimer tx_timer;
#define BUFFER_SIZE   64 // Define the payload size here
uint8_t buffer[BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
void SendPing() {
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0;
  buffer[3] = 0;
  buffer[4] = 'P';
  buffer[5] = 'I';
  buffer[6] = 'N';
  buffer[7] = 'G';

  sx1276_driver.send(buffer, 8);
}

/*---------------------------------------------------------------------------*/
PROCESS(tx_process, "TX process");
AUTOSTART_PROCESSES(&tx_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tx_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("Tx process has begun\n");

	//sx1276_driver.init();
	// Start infinite RX
	//sx1276_driver.on();

	etimer_set(&tx_timer, 3*CLOCK_SECOND);
  while( 1 )
  {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_TIMER) {
    	LEDS_ON(LEDS_ALL);
    	SendPing();
    	PRINTF("Sent the PING\n");
    	LEDS_OFF(LEDS_ALL);

		etimer_reset(&tx_timer);
    }
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
