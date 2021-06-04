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
 * \file
 *          sx1276-arch.c that holds the LoRa radio abstraction Layer for ContikiOS
 * \author
 *          Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Updated : 2018-04-04
 */
/*---------------------------------------------------------------------------
Description: Contiki radio interface implementation for SX1276 Driver
-----------------------------------------------------------------------------*/
#include "contiki.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "sys/energest.h"
#include "sys/node-id.h"
#include "sx1276-arch.h"
#include "sx1276-config.h"
#include "sx1276.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"
#include "spi.h"
#include <string.h>
#include <stdio.h>
#include "sys/clock.h"
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
#define CLEAR_RXBUF()           (rx_msg_size = 0)
#define IS_RXBUF_EMPTY()        (rx_msg_size == 0)
#define CONSEC_CAD_NO						8 // Number of consecutive CAD processes for CCA
/*---------------------------------------------------------------------------*/
//unsigned short node_id;
/*---------------------------------------------------------------------------*/
int8_t rx_last_snr = 0;
int16_t rx_last_rssi = 0;
static bool CadDetected = false;
//static bool CadDone = false;

/* Packet Buffer setting*/
static uint8_t rx_msg_buf[RX_BUFFER_SIZE];
static uint16_t rx_msg_size = 0;

static int packet_is_prepared = 0;
static RadioEvents_t RadioEvents;
// static struct etimer et_reset_rx;
static packetbuf_attr_t last_rssi = 0;

/*---------------------------------------------------------------------------*/
/* SX1276 Radio Driver Static Functions */
static int sx1276_radio_init(void);
static int sx1276_radio_on(void);
static int sx1276_radio_off(void);
static int sx1276_radio_prepare(const void *payload, unsigned short payload_len);
static int sx1276_radio_transmit(unsigned short payload_len);
static int sx1276_radio_send(const void *data, unsigned short len);
static int sx1276_radio_read(void *buf, unsigned short bufsize);
static int sx1276_radio_channel_clear(void);
static int sx1276_radio_receiving_packet(void);
static int sx1276_radio_pending_packet(void);
/*---------------------------------------------------------------------------*/
PROCESS(sx1276_process, "SX1276 driver");
/*---------------------------------------------------------------------------*/
void OnTxDone(void)
{
	PRINTF("TX Done\n");
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  /* Enable RX */
  sx1276_set_rx(RX_TIMEOUT_VALUE_SEC);
	//sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	rx_msg_size = size;
	memcpy(rx_msg_buf, payload, rx_msg_size);

   // save Rssi and SNR
  rx_last_snr = snr;
  rx_last_rssi = rssi;

	printf("Incoming MSG: Size: %d bytes, RSSI: %d, SNR: %d\n", rx_msg_size, rssi, snr);

  process_poll(&sx1276_process);
  //sx1276_radio_on();

}
/*---------------------------------------------------------------------------*/
void OnTxTimeout(void)
{
	PRINTF("TX timeout\n");
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  sx1276_set_rx(RX_TIMEOUT_VALUE_SEC); // Rx, Tx or sleep?
  //sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnRxTimeout(void)
{
  PRINTF("RX Timeout\n");
	CLEAR_RXBUF();
  sx1276_set_rx(RX_TIMEOUT_VALUE_SEC);
  //sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnRxError(void)
{
	PRINTF("RX Error\n");
	CLEAR_RXBUF();
  sx1276_set_rx(RX_TIMEOUT_VALUE_SEC);
  //sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnCadDone( bool channelActivityDetected)
{
	PRINTF("CAD Done\n");
	sx1276_set_sleep();
	// User app
	if(channelActivityDetected)
	{
		//PRINTF("CAD true\n");
		CadDetected = true;
	}
	else
	{
		//PRINTF("CAD false\n");
		CadDetected = false;
	}
	//CadDone = true;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sx1276_process, ev, data)
{
  PROCESS_BEGIN();

	uint8_t len;

  while(1)
  {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
		//sx1276_driver.init();

		PRINTF("LoRa radio: polled\n");
    //sx1276_driver.on();

    /* Clear packetbuf to avoid leftovers from previous RX */
    packetbuf_clear();

    /* Copy the received frame to packetbuf */
    len = sx1276_radio_read(packetbuf_dataptr(), PACKETBUF_SIZE);

		if(len > 0){
			packetbuf_set_datalen(len);
			NETSTACK_RDC.input();
		}

    /* Turn on radio to keep listening */
    //sx1276_radio_on();

		if(!IS_RXBUF_EMPTY()){
			process_poll(&sx1276_process);
		}

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*                        CONTIKI INTERFACE                                  */
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_init(void)
{
  PRINTF("Initializing sx1276\n");

  spi_init();

  // Radio initialization
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
	RadioEvents.CadDone = OnCadDone;

  sx1276_init(&RadioEvents);

  sx1276_set_channel(RF_FREQUENCY);

  sx1276_set_txconfig(MODEM_LORA, TX_OUTPUT_POWER, LORA_FDEV,
																	LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                  LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                                  LORA_FIX_LENGTH_PAYLOAD_ON, LORA_CRC_ON, LORA_FREQ_HOP,
                                  LORA_FREQ_HOP_PERIOD, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE_SEC);


  sx1276_set_rxconfig(MODEM_LORA, LORA_BANDWIDTH,
																	LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_AFC_BANDWIDTH, LORA_PREAMBLE_LENGTH,
                                  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
																	LORA_PAYLOAD_LENGTH,
																	LORA_CRC_ON, LORA_FREQ_HOP, LORA_FREQ_HOP_PERIOD,
																	LORA_IQ_INVERSION_ON, RX_CONTINUOUS_MODE);


	printf("SX1276 initialized with Freq: %lu Hz, TX Pwr: %d dBm, BW: %lu Hz, SF: %d, CR: %d\n",
          RF_FREQUENCY, TX_OUTPUT_POWER, LORA_BANDWIDTH_HZ,
          LORA_SPREADING_FACTOR, LORA_CODINGRATE);

	CLEAR_RXBUF();
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  process_start(&sx1276_process, NULL);
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_on(void)
{
  /* Enable RX */
  sx1276_set_rx(RX_TIMEOUT_VALUE_SEC);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  PRINTF("Radio has been turned on\n");
	// Start reset timer
	 // etimer_set(&et_reset_rx, RESET_RX_DURATION );
	 // // assign it to correct process
	 // et_reset_rx.p = &sx1276_process;
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_off(void)
{
	// stop reset timer
	// etimer_stop(&et_reset_rx);
  // sx1276_set_rx(RX_TIMEOUT_VALUE_SEC); // need timers
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	sx1276_set_sleep();
  PRINTF("Radio has been turned off\n");
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_read(void *buf, unsigned short bufsize)
{
	if (IS_RXBUF_EMPTY()){
		PRINTF("Rx buffer is empty\n");
	}
	else {
	  if (bufsize < rx_msg_size) {
			PRINTF("WARNING: Buffer size is small\n");
		}
		else {
      RIMESTATS_ADD(llrx);
			memcpy(buf, rx_msg_buf, rx_msg_size);
      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, last_rssi);
			bufsize = rx_msg_size;
			CLEAR_RXBUF();
			return bufsize;
		}
	}
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_channel_clear(void)	// TODO: Implement function timeout
{
	bool PacketDetected = false;
	uint8_t i = 0;
	sx1276_set_stby(); // Need to change mode from Rx to work properly
	PRINTF("Channel Activity Detection\n"); // Needs 1 ms to work?
	while(i < CONSEC_CAD_NO){
		sx1276_start_cad();
		sx1276_set_stby(); // Need to change mode for CAD to finish
		_BIS_SR(GIE | SCG0 | SCG1 | CPUOFF | OSCOFF); // LPM4
		PacketDetected = (PacketDetected | CadDetected); // TODO: Implement minimum detection threshold using a counter
		i++;
	}
	// sx1276_set_sleep(); // Sleep only after all consecutive CADs? Doesn't seem to work
	/*if(PacketDetected){
		process_poll(&cad_process);
	}*/
	return !PacketDetected;
}
/*---------------------------------------------------------------------------*/
static uint8_t *packet_payload = NULL;
static uint16_t packet_payload_len = 0;

static int
sx1276_radio_prepare(const void *payload, unsigned short payload_len)
{
   /* Checks if the payload length is supported */
  if(payload_len > RX_BUFFER_SIZE) {
    packet_is_prepared = 0;
    return RADIO_TX_ERR;
  }

  RIMESTATS_ADD(lltx);

  packet_payload = (uint8_t *) payload;
  packet_payload_len = payload_len;
  packet_is_prepared = 1;
  PRINTF("Payload prepared in: %u bytes\n", payload_len);

  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_transmit(unsigned short payload_len)
{
	PRINTF("Transmission has begun\n");

	if(!packet_is_prepared) {
    return RADIO_TX_ERR;
  }

  sx1276_send(packet_payload, packet_payload_len);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
  packet_is_prepared = 0;

  PRINTF("Transmission has ended\n");
  //process_poll(&sx1276_process);
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_send(const void *payload, unsigned short payload_len)
{
	if(sx1276_radio_prepare(payload, payload_len) == RADIO_TX_ERR) {
    return RADIO_TX_ERR;
  }
	return sx1276_radio_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_receiving_packet(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_pending_packet(void)
{
  PRINTF("Pending packet\n");
  return !IS_RXBUF_EMPTY();
}
/*---------------------------------------------------------------------------*/
const struct radio_driver sx1276_driver =
{
  sx1276_radio_init,
  sx1276_radio_prepare,
  sx1276_radio_transmit,
  sx1276_radio_send,
  sx1276_radio_read,
  sx1276_radio_channel_clear,
  sx1276_radio_receiving_packet,
  sx1276_radio_pending_packet,
  sx1276_radio_on,
  sx1276_radio_off,
};
/*---------------------------------------------------------------------------*/
