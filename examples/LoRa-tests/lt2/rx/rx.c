
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
 *  receiver.c
 *  Simple LoRa packet receiver example in Contiki (bare Radio RX)
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
static struct etimer rx_timer;
/*---------------------------------------------------------------------------*/
void MsgInfo(){
  uint8_t *rx_msg;
  uint8_t hdrsize = 0;
  uint8_t datasize = 0;
  uint16_t pkt_no = 0;
  uint16_t rssi = 0;
  hdrsize = packetbuf_hdrlen();
  datasize = packetbuf_datalen();
  rx_msg = packetbuf_hdrptr();
  rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  pkt_no = (*(rx_msg + hdrsize + datasize - 2) << 8) + *(rx_msg + hdrsize + datasize - 1);
  printf(",%d,%d,%d\n", pkt_no, rssi,(hdrsize+datasize));
}
/*---------------------------------------------------------------------------*/
PROCESS(rx_process, "Rx process");
AUTOSTART_PROCESSES(&rx_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rx_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("Rx process has begun\n");

  //sx1276_driver.init();
  //sx1276_driver.on();
  //sx1276_set_rx(0);

  etimer_set(&rx_timer, (1/4)*CLOCK_SECOND); // 1/4, 1/2, 3/2

  printf(",Packet No.,RSSI,No. of bytes\n");

  while(1)
  {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_TIMER) {
      LEDS_ON(LEDS_ALL);

      if(packetbuf_datalen() > 0){
        MsgInfo();
        packetbuf_clear();
        LEDS_OFF(LEDS_ALL);
      }
      etimer_reset(&rx_timer);
    }
  }
  PROCESS_END();
}
