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
 //  Mapping MSP430FR5994 Launchpad <-> SX1276 (Feb 2021)
 //                   Launchpad
 //                 -----------------
 //                |                 |
 //                |             P5.0|-> Data Out (UCB1SIMO)
 //                |                 |
 //                |             P5.1|<- Data In (UCB1SOMI)
 //                |                 |
 //                |             P5.2|-> Serial Clock Out (UCB1CLK)
 //                |                 |
 //                |             P4.4|-> CS
 //                |                 |
 //                |             P4.2|-> RESET
 //                |                 |
 //                |             P4.1|<-> DIO_0
 //                |                 |
 //                |             P6.3|<-> DIO_3
 //                |                 |
 //                 -----------------
/*
 * Initialize SPI bus.
 */
void spi_init(void)
{
  // Configure GPIOs
  P4DIR |= (RESET | CS);
  P4SEL0 &= ~(RESET | CS);
  P4SEL1 &= ~(RESET | CS);
  P4OUT &= ~RESET;
  P4OUT |= CS;

  // Configure SPI
  P5SEL0 |= (MOSI | MISO | SCLK);
  P5SEL1 &= ~(MOSI | MISO | SCLK);

  // Initialize ports for communication with SPI units.
  UCB1CTLW0 = UCSWRST;               // Put state machine in reset
  UCB1CTLW0 |= UCSSEL_2;             // SMCLK
  UCB1CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // MSB-first 8-bit, Master, Synchronous, 3 pin SPI master, no ste, watch-out for clock-phase UCCKPH

  UCB1BR0 = 0x00;
  UCB1BR1 = 0x00;

  UCB1CTLW0 &= ~UCSWRST;  // Initializing USCI and Remove RESET before enabling interrupts

}

void spi_enable(){
  P5SEL0 |= (MOSI | MISO |SCLK);
  P5SEL1 &= ~(MOSI | MISO | SCLK);
}

void spi_disable(){
  P5DIR &= ~(MOSI | MISO | SCLK); // Configure as input
  P5SEL0 &= ~(MOSI | MISO | SCLK);
  P5SEL1 &= ~(MOSI | MISO | SCLK);
}

void dio0irq_init(){

  P4SEL0 &= ~DIO_0;
  P4SEL1 &= ~DIO_0;
  P4OUT &= ~DIO_0;   // Pull down
  P4DIR &= ~DIO_0;   // configure as input for the interrupt signal
  P4REN |= DIO_0;
  P4IES &= ~DIO_0;   // Interrupt on a low to high transition
  P4IE  &= ~DIO_0;   // Interrupt disabled
  P4IFG &= ~DIO_0;   // Clear DIO0 interrupt

}

void dio3irq_init(){

  P6SEL0 &= ~DIO_3;
  P6SEL1 &= ~DIO_3;
  P6OUT &= ~DIO_3;   // Pull down
  P6DIR &= ~DIO_3;   // configure as input for the interrupt signal
  P6REN |= DIO_3;
  P6IES &= ~DIO_3;   // Interrupt on a low to high transition
  P6IE  &= ~DIO_3;   // Interrupt disabled
  P6IFG &= ~DIO_3;   // Clear DIO3 interrupt

}

void spi_txready() {
  while (!(UCB1IFG & UCTXIFG)); // TX buffer ready?
}

void spi_rxready() {
  while (!(UCB1IFG & UCRXIFG)); // RX Received?
}

void spi_send(uint8_t data) {
  spi_txready();
  UCB1TXBUF = data;            // Send data over SPI to Slave
  PRINTF("SPI send: %u\n", data);
}

uint8_t spi_recv() {
  uint8_t spi_buf = 0;
  spi_rxready();
  spi_buf = UCB1RXBUF;         // Store received data
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
  P4OUT &= ~CS;
}

void spi_chipDisable() {
   P4OUT |= CS;
}
