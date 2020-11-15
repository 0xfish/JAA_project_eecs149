
/*
   Test code for the pixy2
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf51_bitfields.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_pwm.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "boards.h"


#define BASE 0x40003000
#define CSN 0x40003514
#define SCK 0x40003508
#define MISO 0x4000350C
#define MOSI 0x40003510
#define RXD 0x40003534
#define TXD 0x40003544


// Application main function.
int main(void)
{
  uint32_t* pins = (uint32_t*) BASE;
  //Set the MOSI pin to 14 and connect
  pins =(uint32_t*) CSN;
  *pins  = 0;
  *pins  = *pins | 0xE;
  //Set the clock pin to 13 then connect
  pins = (uint32_t*)SCK;
  *pins = 0;
  *pins = *pins | 0xD;
  //Set the MISO pin to 12 and connect
  pins = (uint32_t*)MISO;
  *pins  = 0;
  *pins  = *pins | 0xC;
  //Set the MOSI pin to 11 and connect
  pins =(uint32_t*) MOSI;
  *pins  = 0;
  *pins  = *pins | 0xB;
  //Receive pointer
  uint32_t* RXD_PTR = (uint32_t*) RXD;
  uint32_t* TXD_PTR = (uint32_t*) TXD;

  *TXD_PTR = 0xAE;
  nrf_delay_ms(10);
  *TXD_PTR = 0xc1;
  nrf_delay_ms(10);
  *TXD_PTR = 0x0e;
  nrf_delay_ms(10);
  *TXD_PTR = 0x00;

    // main loop:
    while(1) {
        printf("The value in the pixy2 is:%ld \n", *RXD_PTR);
    }
}
