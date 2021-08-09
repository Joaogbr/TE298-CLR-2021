/**
 * \Contiki port for LoRa MSP430FR5994 platform,
 *\original author
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 *\modified by
 *         João Gabriel Pazinato de Bittencourt <joaogabrielpazinatobittencourt@gmail.com>
 */

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#define HAVE_STDINT_H
#include "msp430def.h"

/* CPU target speed in Hz */
#define F_CPU 8000000uL /* 8MHz by default */
// #define F_CPU 1000000uL /* 1MHz Option */

/* Our clock resolution, this is the same as Unix HZ. */
#define CLOCK_CONF_SECOND 128UL
#define RTIMER_CONF_SECOND (4096U*8)

#define BAUD2UBR(baud) ((F_CPU/baud))

#define CCIF
#define CLIF

#define HAVE_STDINT_H
#include "msp430def.h"

#define PLATFORM_HAS_LEDS    1
#define PLATFORM_HAS_BUTTON  1
#define PLATFORM_HAS_NTC     1
#define PLATFORM_HAS_ACCEL   0

#define IS_RADIO_LORA 1 // LoRa communications are being used

#define FRAM_START_ADDR_CONF      0x04400
#define FRAM_END_ADDR_CONF        0x063FF

/* Types for clocks and uip_stats */
typedef unsigned short uip_stats_t;
typedef unsigned long clock_time_t;
typedef unsigned long off_t;

/* the low-level radio driver */
#define NETSTACK_CONF_RADIO     sx1276_driver

//#define NETSTACK_CONF_RDC     	contikimac_driver

//#define NETSTACK_CONF_MAC     	csma_driver

//#define NETSTACK_CONF_FRAMER  	framer_802154

#define CFS_CONF_OFFSET_TYPE    long

/* MSP430 function to enter LPMx.5 mode */
void msp430_enter_LPMx_5(unsigned char LPM_bits);


#endif /* __PLATFORM_CONF_H__ */
