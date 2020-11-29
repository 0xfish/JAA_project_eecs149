// Display app
//
// Write messages to a Newhaven OLED display over SPI

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "HCSR04.h"
#include "virtual_timer_linked_list.h"
#include "virtual_timer.h"



int main(void) {
  printf("Initialilzed!\n");
  setupPins();
  virtual_timer_init();
  virtual_timer_start_repeated(100, timerCounter);
  printf("Initialilzed!\n");
  while(1) {
    // get HC-SR04 distance
    float dist;

    if(getDistance(&dist)) {

      // enable to print to serial port
      //printf("dist = %f cm\n", dist);

      // send distance via NUS
      printf("Distance is :%f\n", dist);
    }

    // delay
    nrf_delay_ms(250);

  }


}
