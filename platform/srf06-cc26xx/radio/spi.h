/*
 * spi.h
 *
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-12
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "board.h"

extern uint8_t spi_buf;

#define CS          BOARD_IOID_CS
#define RESET       BOARD_IOID_DIO12
//#define SCLK        BOARD_IOID_SPI_CLK_FLASH
//#define MOSI        BOARD_IOID_SPI_MOSI
//#define MISO        BOARD_IOID_SPI_MISO
#define DIO_0       BOARD_IOID_DIO15

/*#define CS          BIT0
#define RESET       BIT4
#define SCLK        BIT5
#define MOSI        BIT0
#define MISO        BIT1
#define DIO_0       BIT3*/

/*
 * Initialize the hardware
 */
void spi_reset_config(uint32_t DIO_Number);
void spi_cs_config(uint32_t DIO_Number);
void dio0irq_config(uint32_t DIO_Number);
void spi_init(void);
void spi_enable(void);
void spi_disable(void);
void spi_ready(void);
void spi_send(uint8_t data);
void spi_recv(void);
void spi_transfer(uint8_t data);
void spi_chipEnable(void);
void spi_chipDisable(void);
void dio0irq_init(void);

#endif /* SPI_H_ */
