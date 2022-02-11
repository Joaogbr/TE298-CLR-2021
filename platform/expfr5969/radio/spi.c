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
 * spi.c
 *
 * \Contiki port, original author
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-12
 * \modified by
 *  João Gabriel Pazinato de Bittencourt <joaogabrielpazinatobittencourt@gmail.com>
 */

#include "contiki.h"
#include <msp430.h>
#include "spi.h"
#include <stdint.h>

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif
/*---------------------------------------------------------------------------*/
/*
 * This is SPI initialization code for the MSP430X architecture.
 *
 */
//  Mapping MSP430FR5969 Launchpad <-> SX1276 (May 2020)
//                   Launchpad
//                 -----------------
//                |                 |
//                |             P1.6|-> Data Out (UCB0SIMO)
//                |                 |
//                |             P1.7|<- Data In (UCB0SOMI)
//                |                 |
//                |             P2.2|-> Serial Clock Out (UCB0CLK)
//                |                 |
//                |             P3.0|-> CS
//                |                 |
//                |             P1.4|-> RESET
//                |                 |
//                |             P1.3|<-> DIO_0
//                |                 |
//                |             P3.4|<-> DIO_3
//                |                 |
//                 -----------------
/*
 * Initialize SPI bus.
 */
void spi_init(void)
{
  // Configure GPIOs
  P1DIR |= RESET;
  P1SEL0 &= ~RESET;
  P1SEL1 &= ~RESET;
  P1OUT &= ~RESET;

  P3DIR |= CS;
  P3SEL0 &= ~CS;
  P3SEL1 &= ~CS;
  P3OUT |= CS;

  // Configure SPI
  P2SEL0 &= ~SCLK;
  P2SEL1 |= SCLK;

  P1SEL0 &= ~(MOSI | MISO);
  P1SEL1 |= (MOSI | MISO);

  // Initialize ports for communication with SPI units.
  UCB0CTLW0 = UCSWRST;               // Put state machine in reset
  UCB0CTLW0 |= UCSSEL_2;             // SMCLK
  UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // MSB-first 8-bit, Master, Synchronous, 3 pin SPI master, no ste, watch-out for clock-phase UCCKPH

  UCB0BR0 = 0x00;
  UCB0BR1 = 0x00;

  UCB0CTLW0 &= ~UCSWRST;  // Initializing USCI and Remove RESET before enabling interrupts

}

void spi_enable(){
  P2SEL0 &= ~SCLK;
  P2SEL1 |= SCLK;

  P1SEL0 &= ~(MOSI | MISO);
  P1SEL1 |= (MOSI | MISO);
}

void spi_disable(){
  P2DIR &= ~SCLK; // Configure as input
  P2SEL0 &= ~SCLK;
  P2SEL1 &= ~SCLK;

  P1DIR &= ~(MOSI | MISO); // Configure as input
  P1SEL0 &= ~(MOSI | MISO);
  P1SEL1 &= ~(MOSI | MISO);
}

void dio0irq_init(){

  P1SEL0 &= ~DIO_0;
  P1SEL1 &= ~DIO_0;
  P1OUT &= ~DIO_0;   // Pull down
  P1DIR &= ~DIO_0;   // configure as input for the interrupt signal
  P1REN |= DIO_0;
  P1IES &= ~DIO_0;   // Interrupt on a low to high transition
  P1IE  &= ~DIO_0;   // Interrupt disabled
  P1IFG &= ~DIO_0;   // Clear DIO0 interrupt

}

void dio3irq_init(){

  P3SEL0 &= ~DIO_3;
  P3SEL1 &= ~DIO_3;
  P3OUT &= ~DIO_3;   // Pull down
  P3DIR &= ~DIO_3;   // configure as input for the interrupt signal
  P3REN |= DIO_3;
  P3IES &= ~DIO_3;   // Interrupt on a low to high transition
  P3IE  &= ~DIO_3;   // Interrupt disabled
  P3IFG &= ~DIO_3;   // Clear DIO3 interrupt

}

void spi_txready() {
  while (!(UCB0IFG & UCTXIFG)); // TX buffer ready?
}

void spi_rxready() {
  while (!(UCB0IFG & UCRXIFG)); // RX Received?
}

void spi_send(uint8_t data) {
  spi_txready();
  UCB0TXBUF = data;            // Send data over SPI to Slave
  PRINTF("SPI send: %u\n", data);
}

uint8_t spi_recv() {
  uint8_t spi_buf = 0;
  spi_rxready();
  spi_buf = UCB0RXBUF;         // Store received data
  PRINTF("SPI recv: %u\n", spi_buf);
  return spi_buf;
}

uint8_t spi_transfer(uint8_t data) {
  uint8_t recv = 0;
  spi_send(data);
  recv = spi_recv();
  return recv;
}

void spi_chipEnable() {
  P3OUT &= ~CS;
}

void spi_chipDisable() {
   P3OUT |= CS;
}
