
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
 *  CAD.c
 *  Simple LoRa CCA example in Contiki
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-28
 */

#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h>
#include "sx1276.h"
#include "sx1276-arch.h"
#include "spi.h"
#include "dev/button-sensor.h"
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
//static struct etimer cca_timer;
//static uint16_t cnt = 1;
//static uint8_t consec = 0;
/*---------------------------------------------------------------------------*/
PROCESS(cca_process, "cca process");
AUTOSTART_PROCESSES(&cca_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cca_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("Test No. 3 has begun\n");
  //etimer_set(&cca_timer, CLOCK_SECOND/20);

  sx1276_driver.off();
  delay_ms(20000);
  while(1)
  {
    //PROCESS_WAIT_EVENT();

    //if(ev == PROCESS_EVENT_TIMER) {
      LEDS_ON(LEDS_ALL);
      if(sx1276_driver.channel_clear()){
        /*if(consec>0){
          consec--;
        }*/
        printf(",0\n");
      }
      else{
        //if(consec==0){
          //printf(",Channel occupied %d\n", cnt); // Transmissions should be spaced out by several seconds
          //cnt++;
          //consec=30; // The amount of channel clear results before considering a different packet was detected
          printf(",1\n");
          LEDS_OFF(LEDS_ALL);
        //}
      }
      //etimer_reset(&cca_timer);
    //}
  }
  PROCESS_END();
}
