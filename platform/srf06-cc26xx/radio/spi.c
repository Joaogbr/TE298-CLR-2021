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
 * \Contiki port, author
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-12
 *  Adapted for CC1310 by Joao Bittencourt <joaogabrielpazinatobittencourt@gmail.com>
 *  Last modified: 2020-01-30
 */
#include "contiki.h"
#include "spi.h"
#include "board.h"
#include "board-spi.h"
#include "ti-lib.h"
#include "sys/clock.h"

#include <stdint.h>
#include <stdio.h>

uint8_t spi_buf = 0;

void spi_reset_config(uint32_t DIO_Number)
{
  // Configure RESET
  ti_lib_ioc_pin_type_gpio_output(DIO_Number);   // Output config
  ti_lib_gpio_clear_dio(DIO_Number);   // Initially 0
}

void spi_cs_config(uint32_t DIO_Number)
{
  ti_lib_ioc_pin_type_gpio_output(DIO_Number);
  ti_lib_gpio_set_dio(DIO_Number);
}

void dio0irq_config(uint32_t DIO_Number)
{
  ti_lib_ioc_pin_type_gpio_input(DIO_Number); //ti_lib_ioc_io_input_set(DIO_Number, IOC_INPUT_ENABLE);
  ti_lib_ioc_io_port_pull_set(DIO_Number, IOC_IOPULL_DOWN);
  ti_lib_ioc_io_int_set(DIO_Number, IOC_INT_DISABLE, IOC_RISING_EDGE);
  ti_lib_ioc_int_clear(DIO_Number);
  while(ti_lib_ioc_int_status(DIO_0)); // Dummy read after clearing interrupt (datasheet)
}

/*
 * Initialize SPI bus.
 */
void spi_init(void)
{
  // board SPI init
  board_spi_open(4000000, BOARD_IOID_SPI_CLK_FLASH);

  ti_lib_ioc_io_port_pull_set(BOARD_IOID_SPI_MISO, IOC_IOPULL_DOWN);

  //ti_lib_ioc_io_drv_strength_set(BOARD_IOID_SPI_MISO, IOC_CURRENT_2MA, IOC_STRENGTH_MIN);
  //ti_lib_ioc_io_drv_strength_set(BOARD_IOID_SPI_CLK_FLASH, IOC_CURRENT_2MA, IOC_STRENGTH_MIN);
  //ti_lib_ioc_io_drv_strength_set(BOARD_IOID_SPI_MOSI, IOC_CURRENT_2MA, IOC_STRENGTH_MIN);
}

void spi_enable(void){
  board_spi_enable(4000000, BOARD_IOID_SPI_CLK_FLASH);

  ti_lib_ioc_io_port_pull_set(BOARD_IOID_SPI_MISO, IOC_IOPULL_DOWN);
}

void spi_disable(void){
  /* Restore pins to a low-consumption state */
  //ti_lib_ioc_pin_type_gpio_input(RESET);
  //ti_lib_ioc_io_port_pull_set(RESET, IOC_IOPULL_DOWN);

  //ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_FLASH_CS);
  //ti_lib_ioc_io_port_pull_set(BOARD_IOID_FLASH_CS, IOC_IOPULL_DOWN);

  board_spi_close();
}

void dio0irq_init(){

  dio0irq_config(DIO_0);
}

void spi_ready() {
  while (ti_lib_ssi_busy(SSI0_BASE));
}

void spi_send(uint8_t data) {
  spi_ready();
  if(!board_spi_write(&data, 1)) {
    puts("Spi_send failed!");
  }
  /*else {
    puts("Spi_send done");
  }*/
}

void spi_recv() {
  spi_ready();
  if(!board_spi_read(&spi_buf, 1)) {
    puts("Spi_recv failed!");
  }
  /*else {
    printf("Spi_recv done");
  }*/
}

void spi_chipEnable() {
  ti_lib_gpio_clear_dio(CS); // Output is low
}

void spi_chipDisable() {
  ti_lib_gpio_set_dio(CS); // Output is high
}

void spi_transfer(uint8_t data) {
  spi_send(data);
  spi_recv();
  //spi_chipDisable(); // FSS must pulse high
  //clock_delay_usec(1);
  //spi_chipEnable();
}
