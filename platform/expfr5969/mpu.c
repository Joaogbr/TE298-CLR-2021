#include "contiki.h"
#include "dev/mpu.h"
#include <stdio.h>

#define __PRE_INIT __attribute__((naked, section(".crt_0042"), used)) // Add to function that should run during system startup

void MPU_enable(void){
  /* Disable Watchdog timer to prevent reset during*/
  /* long variable initialization sequences.*/
  WDTCTL = WDTPW | WDTHOLD;

  MPUCTL0 = MPUPW;     // Write PWD to access MPU registers

  MPUCTL0 |= MPUENA;   // Enable MPU
}

void MPU_disable(void){
  /* Disable Watchdog timer to prevent reset during*/
  /* long variable initialization sequences.*/
  WDTCTL = WDTPW | WDTHOLD;

  MPUCTL0 = MPUPW;     // Write PWD to access MPU registers

  MPUCTL0 &= ~MPUENA;   // Disable MPU
}

void __PRE_INIT MPU_setTwoSegments(void){
  /* Disable Watchdog timer to prevent reset during*/
  /* long variable initialization sequences.*/
  WDTCTL = WDTPW | WDTHOLD;

  MPUCTL0 = MPUPW;     // Write PWD to access MPU register

  MPUSEGB1 = 0x0640; // Region boundary for .bss section shifted right 4x

  MPUSEGB2 = 0x0640; // Region boundary for .bss section shifted right 4x

  MPUSAM = (MPUSEG1WE | MPUSEG1RE | MPUSEG2RE | MPUSEG2XE | MPUSEG3RE | MPUSEG3XE); // Seg1 is rw only, seg2 and seg3 are rx only

  MPUCTL0 |= MPUPW | MPUENA | MPUSEGIE;   // Enable MPU protection and lock until BOR
}
