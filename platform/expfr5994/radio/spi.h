/*
 * spi.h
 *
 *\original author
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 *\modified by
 *         João Gabriel Pazinato de Bittencourt <joaogabrielpazinatobittencourt@gmail.com>
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

extern volatile uint8_t spi_buf;

#define CS          BIT4
#define RESET       BIT2
#define SCLK        BIT2
#define MOSI        BIT0
#define MISO        BIT1
#define DIO_0       BIT1
#define DIO_3       BIT3

/*
 * Initialize the hardware
 */
void spi_init(void);
void spi_enable(void);
void spi_disable(void);
void spi_txready();
void spi_rxready();
void spi_send(uint8_t data);
uint8_t spi_recv();
uint8_t spi_transfer(uint8_t data);
void spi_chipEnable();
void spi_chipDisable();
void dio0irq_init();
void dio3irq_init();

#endif /* SPI_H_ */
