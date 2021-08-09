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
#include <dev/watchdog.h>
#include "sys/clock.h"
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
#define CLEAR_RXBUF()           (rx_msg_size = 0)
#define IS_RXBUF_EMPTY()        (rx_msg_size == 0)
#define CAD_THRESHOLD						1 // Min number of CAD detections
#define WAIT_FOR_TXDONE					1

// Number of max consecutive CAD processes for CCA. Assume CCA function is called twice.
#if (LORA_SPREADING_FACTOR == 12)
#define CONSEC_CAD_NO						3*2
#elif (LORA_SPREADING_FACTOR == 11) // Reference no. (8 CADs)
#define CONSEC_CAD_NO						4*2
#elif (LORA_SPREADING_FACTOR == 10)
#define CONSEC_CAD_NO						6*2
#elif (LORA_SPREADING_FACTOR == 9) // Double from SF11
#define CONSEC_CAD_NO						8*2
#elif (LORA_SPREADING_FACTOR == 8)
#define CONSEC_CAD_NO						12*2
#elif (LORA_SPREADING_FACTOR == 7) // Double from SF9
#define CONSEC_CAD_NO						16*2
#endif
/*---------------------------------------------------------------------------*/
//unsigned short node_id;
/*---------------------------------------------------------------------------*/
//int8_t rx_last_snr = 0;
static bool TxDone = false;
static bool TxTimeout = false;
static bool CadDetected = false;
static bool CadDone = false;

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
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
	PRINTF("TX Done\n");
	TxDone = true;
  //sx1276_set_rx(0);
	//sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	/*if(!RX_CONTINUOUS_MODE){
		ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	}*/
	rx_msg_size = size;
	memcpy(rx_msg_buf, payload, rx_msg_size);

   // save Rssi and SNR
  //rx_last_snr = snr;
  last_rssi = rssi;

	PRINTF("Incoming MSG: Size: %d bytes, RSSI: %d, SNR: %d\n", rx_msg_size, rssi, snr);

  process_poll(&sx1276_process);
  //sx1276_radio_on();
}
/*---------------------------------------------------------------------------*/
void OnTxTimeout(void)
{
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
	PRINTF("TX Timeout\n");
	TxDone = true;
	TxTimeout = true;
  //sx1276_set_rx(0); // Rx, Tx or sleep?
  //sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnRxTimeout(void)
{
  //sx1276_set_rx(RX_TIMEOUT_VALUE_SEC);
  sx1276_set_sleep();
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	PRINTF("RX Timeout\n");
	CLEAR_RXBUF();
}
/*---------------------------------------------------------------------------*/
void OnRxError(void)
{
	printf("RX Error\n");
	CLEAR_RXBUF();
  sx1276_set_rx(0);
  //sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnCadDone( bool channelActivityDetected)
{
	//PRINTF("CAD Done\n");
	sx1276_set_sleep();
	// User app
	if(channelActivityDetected)
	{
		PRINTF("CAD true\n");
		CadDetected = true;
	}
	/*else
	{
		//PRINTF("CAD false\n");
		CadDetected = false;
	}*/
	CadDone = true;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sx1276_process, ev, data)
{
  PROCESS_BEGIN();

	uint8_t len = 0;

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

		/*if(!IS_RXBUF_EMPTY()){
			process_poll(&sx1276_process);
		}*/

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
  sx1276_set_rx(0);
  ENERGEST_SWITCH(ENERGEST_TYPE_TRANSMIT, ENERGEST_TYPE_LISTEN);
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
	sx1276_set_sleep();
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  PRINTF("Radio has been turned off\n");
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_read(void *buf, unsigned short bufsize)
{
	if (IS_RXBUF_EMPTY()){
		PRINTF("Rx buffer is empty\n");
		return 0;
	}
	else {
	  if (bufsize < rx_msg_size) {
			PRINTF("WARNING: Buffer size is small\n");
			bufsize = 0;
		}
		else {
      RIMESTATS_ADD(llrx);
			memcpy(buf, rx_msg_buf, rx_msg_size);
      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, last_rssi);
			bufsize = rx_msg_size;
			//CLEAR_RXBUF();
		}
	}
	CLEAR_RXBUF();
  return bufsize;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_channel_clear(void)	// TODO: Make function blocking (already is?)
{																	// and implement timeout
	uint8_t PacketDetected = 0;
	uint8_t i = 0;
	uint8_t was_off = 0;

	if(sx1276.Settings.State == RF_IDLE){
		was_off = 1;
	}

	sx1276_set_stby(); // Need to change mode from Rx to work properly
	PRINTF("Channel Activity Detection\n"); // Needs 1 ms to work?
	ENERGEST_SWITCH(ENERGEST_TYPE_LISTEN, ENERGEST_TYPE_CAD);
	while(i < CONSEC_CAD_NO){ // TODO: Contikimac executes this function twice,
														// so maybe adjust the number of repeated CADs
														// according to SF to optimize duration and reliability.
		CadDone = false;
		CadDetected = false;
		sx1276_start_cad();
		//sx1276_set_stby(); // Need to change mode for CAD to finish
		#if RTIMER_ARCH_SECOND == 32768
			rtimer_arch_sleep_until((RTIMER_ARCH_SECOND/20), &CadDone); // Timer A0 has 16384 Hz, so cut wait period in half
		#elif RTIMER_ARCH_SECOND == 16384
			rtimer_arch_sleep_until(RTIMER_ARCH_SECOND/10, &CadDone);
		#endif
		if(CadDetected){
			PacketDetected++;
		}
		if(PacketDetected >= CAD_THRESHOLD){ // Immediately exit function and listen
			break;
		}
		i++;
	}

	ENERGEST_OFF(ENERGEST_TYPE_CAD);
	if(was_off){
		sx1276_radio_off();
	}
	else{
		sx1276_radio_on();
	}

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
	uint8_t was_off = 0;
	PRINTF("Transmission has begun\n");

	if(!packet_is_prepared) {
    return RADIO_TX_ERR;
  }

	if(sx1276.Settings.State == RF_IDLE){
		was_off = 1;
	}
	sx1276_set_stby();
	TxDone = false;
  sx1276_send(packet_payload, packet_payload_len);
  ENERGEST_SWITCH(ENERGEST_TYPE_LISTEN, ENERGEST_TYPE_TRANSMIT);
  packet_is_prepared = 0;

#if WAIT_FOR_TXDONE // Wait for 4 seconds
#if RTIMER_ARCH_SECOND == 32768
	TxTimeout = !(rtimer_arch_sleep_until(2*RTIMER_ARCH_SECOND-1, &TxDone)); // Timer A0 has 16384 Hz, so cut wait period in half
#elif RTIMER_ARCH_SECOND == 16384
	TxTimeout = !(rtimer_arch_sleep_until(4*RTIMER_ARCH_SECOND-1, &TxDone));
#endif
#endif
  PRINTF("Transmission has ended\n");

	if(was_off){
		sx1276_radio_off();
	}
	else{
		sx1276_radio_on();
	}

	if(TxTimeout == true) {
		TxTimeout = false;
    return RADIO_TX_ERR;
  }

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
  if((sx1276_read(REG_LR_MODEMSTAT) & 0x08)){ //0x08 - Valid header,
                                              //0x02 - Signal synchronized,
                                              //0x01 - Signal detected
    PRINTF("Receiving packet\n");
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_pending_packet(void)
{
  if(!IS_RXBUF_EMPTY()){
    PRINTF("Pending packet\n");
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/*static radio_result_t
sx1276_radio_get_value(radio_param_t param, radio_value_t *value)
{
	switch(param) {
  case RADIO_PARAM_POWER_MODE:
		if(sx1276_read(REG_OPMODE) == RF_OPMODE_SLEEP){
			*value = RADIO_POWER_MODE_OFF;
		}
		else{
			*value = RADIO_POWER_MODE_ON;
		}
		return RADIO_RESULT_OK;
	//case RADIO_PARAM_CHANNEL:
	//	*value = RF_FREQUENCY;
	//	return RADIO_RESULT_OK;
	case RADIO_PARAM_PAN_ID:
		return RADIO_RESULT_NOT_SUPPORTED;
	case RADIO_PARAM_16BIT_ADDR:
		return RADIO_RESULT_NOT_SUPPORTED;
		return RADIO_RESULT_OK;
	case RADIO_PARAM_TXPOWER:
		*value = TX_OUTPUT_POWER;
		return RADIO_RESULT_OK;
	case RADIO_PARAM_RSSI:
		*value = sx1276_read(REG_LR_RSSIVALUE) - 137;
		return RADIO_RESULT_OK;
	case RADIO_PARAM_LAST_RSSI:
		//	*value = sx1276_read(REG_LR_PKTRSSIVALUE) - 137;
		*value = last_rssi;
		return RADIO_RESULT_OK;
	//case RADIO_CONST_CHANNEL_MIN:
	//	*value = 0;
	//	return RADIO_RESULT_OK;
	//case RADIO_CONST_CHANNEL_MAX:
	//	*value = (uint8_t) ((RF_FREQUENCY_MAX - RF_FREQUENCY_MIN)/(LORA_BANDWIDTH_HZ) - 1);
	//	return RADIO_RESULT_OK;
	case RADIO_CONST_TXPOWER_MIN:
		//if((sx1276_read(REG_PACONFIG) & RF_PACONFIG_PASELECT_PABOOST) == 0){
		//	*value = 0x02; // 2 dBm
		//}
		//else{
			*value = 0x00; // 0 dBm
		//}
		return RADIO_RESULT_OK;
	case RADIO_CONST_TXPOWER_MAX:
		//if((sx1276_read(REG_PACONFIG) & RF_PACONFIG_PASELECT_PABOOST) == 0){
		//	*value = 0x14; // 20 dBm
		//}
		//else{
			*value = 0x0E; // 14 dBm
		//}
		return RADIO_RESULT_OK;
	default:
    return RADIO_RESULT_NOT_SUPPORTED;
	}
}*/
/*---------------------------------------------------------------------------*/
/*static radio_result_t
sx1276_radio_set_value(radio_param_t param, radio_value_t value)
{
	//uint8_t opmode = 0;
	switch(param) {
  case RADIO_PARAM_POWER_MODE:
		if(value == RADIO_POWER_MODE_ON){
			sx1276_radio_on();
			return RADIO_RESULT_OK;
		}
		else if(value == RADIO_POWER_MODE_OFF){
			sx1276_radio_off();
			return RADIO_RESULT_OK;
		}
		return RADIO_RESULT_INVALID_VALUE;
	case RADIO_PARAM_CHANNEL:
		//opmode = sx1276_read(REG_OPMODE);
		sx1276_set_sleep();
		sx1276_set_channel(value);
		//sx1276_write(REG_OPMODE, opmode);
		return RADIO_RESULT_OK;
	case RADIO_PARAM_PAN_ID:
		if(value == 0x34){
			return RADIO_RESULT_INVALID_VALUE;
		}
		//opmode = sx1276_read(REG_OPMODE);
		sx1276_set_sleep();
		sx1276_write(REG_LR_SYNCWORD, (uint8_t) value);
		//sx1276_write(REG_OPMODE, opmode);
		return RADIO_RESULT_OK;
	case RADIO_PARAM_16BIT_ADDR:
		node_id = (uint16_t) value; // No way to change SX1276 Address on Lora mode?
		return RADIO_RESULT_OK;
	case RADIO_PARAM_TXPOWER:
		//opmode = sx1276_read(REG_OPMODE);
		sx1276_set_sleep();
		sx1276_set_txconfig(MODEM_LORA, value, 0, LORA_BANDWIDTH,
																		LORA_SPREADING_FACTOR, LORA_CODINGRATE,
																		LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
																		true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
		//sx1276_write(REG_OPMODE, opmode);
		return RADIO_RESULT_OK;
	default:
  	return RADIO_RESULT_NOT_SUPPORTED;
	}
}*/
/*---------------------------------------------------------------------------*/
/*static radio_result_t
sx1276_radio_get_object(radio_param_t param, void *dest, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}*/
/*---------------------------------------------------------------------------*/
/*static radio_result_t
sx1276_radio_set_object(radio_param_t param, const void *src, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}*/
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
	//sx1276_radio_get_value,
  //sx1276_radio_set_value,
  //sx1276_radio_get_object,
  //sx1276_radio_set_object,
};
/*---------------------------------------------------------------------------*/
