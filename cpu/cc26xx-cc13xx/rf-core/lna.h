#ifndef POWER_CONTROL
#define POWER_CONTROL 1

static void init_pin(void){
  ti_lib_rom_ioc_pin_type_gpio_output(BOARD_IOID_DIO29);
  ti_lib_rom_ioc_pin_type_gpio_output(BOARD_IOID_DIO30);
  ti_lib_rom_ioc_pin_type_gpio_output(BOARD_IOID_DIO28);
}

/*static void RX_HGM(void) {

    ti_lib_gpio_clear_dio(BOARD_IOID_DIO30);
    ti_lib_gpio_set_dio(BOARD_IOID_DIO29);
    ti_lib_gpio_set_dio(BOARD_IOID_DIO28);

}*/

static void RX_LGM(void) {

    ti_lib_gpio_clear_dio(BOARD_IOID_DIO30);
    ti_lib_gpio_set_dio(BOARD_IOID_DIO29);
    ti_lib_gpio_clear_dio(BOARD_IOID_DIO28);

}

static void RX_CLEAR(void) {

    ti_lib_gpio_clear_dio(BOARD_IOID_DIO29);

}

/*static void TX_HGM(void) {
    ti_lib_gpio_set_dio(BOARD_IOID_DIO30);
    ti_lib_gpio_clear_dio(BOARD_IOID_DIO29);
    ti_lib_gpio_set_dio(BOARD_IOID_DIO28);
}*/

static void TX_LGM(void) {
    ti_lib_gpio_set_dio(BOARD_IOID_DIO30);
    ti_lib_gpio_clear_dio(BOARD_IOID_DIO29);
    ti_lib_gpio_clear_dio(BOARD_IOID_DIO28);
}

static void POWER_DOWN(void) {
 
    ti_lib_gpio_clear_dio(BOARD_IOID_DIO30);
    ti_lib_gpio_clear_dio(BOARD_IOID_DIO28);
    ti_lib_gpio_clear_dio(BOARD_IOID_DIO29);
}

#endif

