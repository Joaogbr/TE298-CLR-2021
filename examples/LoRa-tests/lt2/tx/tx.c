
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
static uint16_t pkt_no = 1;
#define BUFFER_SIZE   128 // Define the payload size here
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
  buffer[8] = 'P';
  buffer[9] = 'I';
  buffer[10] = 'N';
  buffer[11] = 'G';
  buffer[12] = 'P';
  buffer[13] = 'I';
  buffer[14] = 'N';
  buffer[15] = 'G';
  buffer[16] = 'P';
  buffer[17] = 'I';
  buffer[18] = 'N';
  buffer[19] = 'G';
  buffer[20] = 'P';
  buffer[21] = 'I';
  buffer[22] = 'N';
  buffer[23] = 'G';
  buffer[24] = 'P';
  buffer[25] = 'I';
  buffer[26] = 'N';
  buffer[27] = 'G';
  buffer[28] = 'N';
  buffer[29] = 'G';
  buffer[30] = 'P';
  buffer[31] = 'I';
  buffer[32] = 'N';
  buffer[33] = 'G';
  buffer[34] = 'P';
  buffer[35] = 'I';
  buffer[36] = 'N';
  buffer[37] = 'G';
  buffer[38] = 'P';
  buffer[39] = 'I';
  buffer[40] = 'N';
  buffer[41] = 'G';
  buffer[42] = 'P';
  buffer[43] = 'I';
  buffer[44] = 'N';
  buffer[45] = 'G';
  buffer[46] = 'P';
  buffer[47] = 'I';
  buffer[48] = (pkt_no >> 8);
  buffer[49] = pkt_no;

  sx1276_driver.send(buffer,50);
}

/*---------------------------------------------------------------------------*/
PROCESS(tx_process, "Tx process");
AUTOSTART_PROCESSES(&tx_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tx_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("Test No. 2 (Tx) has begun\n");

	//sx1276_driver.init();
	// Start infinite RX
	//sx1276_driver.on();

	etimer_set(&tx_timer, (1/4)*CLOCK_SECOND); // 1/4, 1/2, 3/2
  while( pkt_no <= 1000 )
  {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_TIMER) {
    	LEDS_ON(LEDS_ALL);
    	SendPing();
    	printf(",Sent pkt %d\n", pkt_no);
    	LEDS_OFF(LEDS_ALL);
      pkt_no++;
		  etimer_reset(&tx_timer);
    }
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
