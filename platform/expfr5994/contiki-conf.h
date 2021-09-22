/* -*- C -*- */

#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#include "platform-conf.h"

#ifndef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC       nullmac_driver
#endif /* NETSTACK_CONF_MAC */

#ifndef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC       nullrdc_driver
#endif /* NETSTACK_CONF_RDC */

#ifndef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8 //8
#endif /* NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE */

#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO     sx1276_driver
#endif /* NETSTACK_CONF_RADIO */

#ifndef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER    framer_nullmac //framer_802154
#endif /* NETSTACK_CONF_FRAMER */

#ifndef IS_RADIO_LORA
#define IS_RADIO_LORA 1 // LoRa communications are being used
#endif

/* NullRDC config */
#define NULLRDC_CONF_802154_AUTOACK      0
#define NULLRDC_CONF_ACK_WAIT_TIME                (RTIMER_SECOND) //(RTIMER_SECOND / 400)
#define NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME (RTIMER_SECOND) //(RTIMER_SECOND / 1000)
#define NULLRDC_CONF_CCA_COUNT_MAX_TX    1 //6
#define NULLRDC_CONF_CCA_CHECK_TIME      0 //RTIMER_ARCH_SECOND / 8192
#define NULLRDC_CONF_CCA_SLEEP_TIME      RTIMER_ARCH_SECOND / 2000 //RTIMER_ARCH_SECOND / 2000
/* NullRDC config */

/* CSMA config */
#define CSMA_CONF_MIN_BE        2 //0
#define CSMA_CONF_MAX_BE        5 //4
//#define CSMA_CONF_MAX_BACKOFF   5 //5 (Max)

//#define CSMA_CONF_MAX_FRAME_RETRIES 7 //7 (Max)

//#define QUEUEBUF_CONF_NUM                8 //8
/* CSMA config */

/* ContikiMAC config */
//#undef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
//#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE    4 //8

//#define RDC_CONF_MCU_SLEEP           1 //0

//#define CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION   1 //0

#define WITH_FAST_SLEEP              1 //1
//#define CONTIKIMAC_CONF_MAX_SILENCE_PERIODS         5 //5 (WITH_FAST_SLEEP)
//#define CONTIKIMAC_CONF_MAX_NONACTIVITY_PERIODS     10 //10

//#define CONTIKIMAC_CONF_INTER_PACKET_DEADLINE   CLOCK_SECOND / 2 //CLOCK_SECOND / 32

//#define CONTIKIMAC_CONF_CCA_COUNT_MAX       2 //2 (Min?)
#define CONTIKIMAC_CONF_CCA_COUNT_MAX_TX    2 //6
#define CONTIKIMAC_CONF_CCA_CHECK_TIME      0 //RTIMER_ARCH_SECOND / 8192
#define CONTIKIMAC_CONF_CCA_SLEEP_TIME      RTIMER_ARCH_SECOND / 10 //RTIMER_ARCH_SECOND / 2000

//#define CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED   RTIMER_ARCH_SECOND / 2 //RTIMER_ARCH_SECOND / 80
//#define CONTIKIMAC_CONF_AFTER_ACK_DETECTED_WAIT_TIME        RTIMER_SECOND / 150 //RTIMER_SECOND / 1500

//#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL     RTIMER_SECOND / 1000 //RTIMER_SECOND / 2500

//#define CONTIKIMAC_CONF_SEND_SW_ACK         1 //0

//#define CONTIKIMAC_CONF_COMPOWER            0 //0

//#define CONTIKIMAC_FRAMER_CONF_SHORTEST_PACKET_SIZE     43 //43
/* ContikiMAC config */

#if NETSTACK_CONF_WITH_IPV6
/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver

/* Specify a minimum packet size for 6lowpan compression to be
   enabled. This is needed for ContikiMAC, which needs packets to be
   larger than a specified size, if no ContikiMAC header should be
   used. */
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD 63

#else /* NETSTACK_CONF_WITH_IPV6 */

/* Network setup for non-IPv6 (rime). */
#define NETSTACK_CONF_NETWORK       rime_driver
#define COLLECT_CONF_ANNOUNCEMENTS       1
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0
#define CONTIKIMAC_CONF_ANNOUNCEMENTS    0

#define CONTIKIMAC_CONF_COMPOWER         0
#define XMAC_CONF_COMPOWER               0
#define CXMAC_CONF_COMPOWER              0

#ifndef COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS
#define COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS     2
#endif /* COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS */

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                2 //16
#endif /* QUEUEBUF_CONF_NUM */

#endif /* NETSTACK_CONF_WITH_IPV6 */


#ifndef QUEUEBUF_CONF_FRAM
#define QUEUEBUF_CONF_FRAM               0
#ifndef QUEUEBUF_CONF_FRAM_ADDR
#define QUEUEBUF_CONF_FRAM_ADDR          0xB000
#endif  /* QUEUEBUF_CONF_FRAM_ADDR */
#endif /* QUEUEBUF_CONF_FRAM */

#define PACKETBUF_CONF_ATTRS_INLINE 1

#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT 0

#define PROFILE_CONF_ON 0
#define ENERGEST_CONF_ON 1
#define PROCESS_CONF_NUMEVENTS 8    // Each takes 8 B RAM
#define PROCESS_CONF_STATS 1        // less: 18 B ROM, 0 RAM if set to 1

#define ENERGEST_CONF_PLATFORM_ADDITIONS ENERGEST_TYPE_CAD

#define ELFLOADER_CONF_TEXT_IN_ROM 0
#ifndef ELFLOADER_CONF_DATAMEMORY_SIZE
#define ELFLOADER_CONF_DATAMEMORY_SIZE 0x400
#endif /* ELFLOADER_CONF_DATAMEMORY_SIZE */
#ifndef ELFLOADER_CONF_TEXTMEMORY_SIZE
#define ELFLOADER_CONF_TEXTMEMORY_SIZE 0x800
#endif /* ELFLOADER_CONF_TEXTMEMORY_SIZE */

#define AODV_COMPLIANCE
#define AODV_NUM_RT_ENTRIES 8

#define WITH_ASCII 1


#ifdef NETSTACK_CONF_WITH_IPV6

#define LINKADDR_CONF_SIZE              8

//#ifndef UIP_CONF_DS6_DEFAULT_PREFIX
//#define UIP_CONF_DS6_DEFAULT_PREFIX 0xfe80
//#endif /* UIP_CONF_DS6_DEFAULT_PREFIX */

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

#define UIP_CONF_ROUTER                 1

/* configure number of neighbors and routes */
#ifndef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS     4 //2
#endif /* NBR_TABLE_CONF_MAX_NEIGHBORS */
#ifndef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES   4 //2
#endif /* UIP_CONF_MAX_ROUTES */

//#define UIP_CONF_ND6_SEND_RA    0
//#define UIP_CONF_ND6_SEND_NA    0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000 //10000 (original),30000

#define NETSTACK_CONF_WITH_IPV6                   1
#ifndef UIP_CONF_IPV6_QUEUE_PKT
#define UIP_CONF_IPV6_QUEUE_PKT         0
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_IP_FORWARD             0
#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE    128
#endif

/* RPL config */
#define RPL_CONF_STATS                  1 //0
#define RPL_CONF_MAX_DAG_PER_INSTANCE   1 //2

//#define RPL_CONF_DIO_INTERVAL_MIN         13 //12
//#define RPL_CONF_DIO_INTERVAL_DOUBLINGS  7 //8
//#define RPL_CONF_DIO_REDUNDANCY          8 //10

//#define RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME   1 //1
//#define RPL_CONF_DEFAULT_LIFETIME_UNIT       60 //60
//#define RPL_CONF_DEFAULT_LIFETIME            60 //30

#define RPL_CONF_DAO_DELAY                 (CLOCK_SECOND * 10) //(CLOCK_SECOND * 4)
//#define RPL_CONF_WITH_DAO_ACK                 0 //0

//#define RPL_CONF_DIO_REFRESH_DAO_ROUTES       1 //1  (!RPL_CONF_WITH_DAO_ACK)

//#define RPL_CONF_PROBING_INTERVAL             (120 * CLOCK_SECOND) //(120 * CLOCK_SECOND)
//#define RPL_CONF_DIS_INTERVAL                60 //60
//#define RPL_CONF_DIS_START_DELAY             10 //5

//#define RPL_CONF_MIN_HOPRANKINC             128 //128 (MRHOF)
/* RPL config */

/* 6LowPAN */
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8
#endif /* SICSLOWPAN_CONF_FRAG */
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
#else /* NETSTACK_CONF_WITH_IPV6 */
#define UIP_CONF_IP_FORWARD      1
#define UIP_CONF_BUFFER_SIZE     16
#endif /* NETSTACK_CONF_WITH_IPV6 */

#define UIP_CONF_TCP 0

#define UIP_CONF_ICMP_DEST_UNREACH 1

#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LLH_LEN         0
#ifndef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  48 //48 (original), 60, 128, 300
#endif
// #ifndef UIP_CONF_TCP_MSS
// #define UIP_CONF_TCP_MSS         0
// #endif
#define UIP_CONF_MAX_CONNECTIONS 2 //2 (original), 4
#define UIP_CONF_MAX_LISTENPORTS 2 //2 (original), 4
#define UIP_CONF_UDP_CONNS       6
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0 //0

#define UIP_CONF_TCP_SPLIT       0

#define PROCESS_CONF_NO_PROCESS_NAMES 0

/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef   PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */


#endif /* CONTIKI_CONF_H */
