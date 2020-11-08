// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <gpio.h>
#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "software_interrupt.h"

#include "buckler.h"

void SWI1_EGU1_IRQHandler(void) {
    NRF_EGU1->EVENTS_TRIGGERED[0] = 0;
    printf("Oh no! You found me! SWI1_EGU1_IRQHandler\n");
}
//23,24,25 LEDS
void GPIOTE_IRQHandler(void) {
    NRF_GPIOTE->EVENTS_IN[0] = 0;
    printf("Oh no! You found me! GPIOTE_IRQHandler\n");
    //gpio_set(23);
    //nrf_delay_ms(500);
    //gpio_clear(23);
    //printf("end of interrupt\n");
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  //Do stuff for checkoff 5.2.2
  NRF_GPIOTE->CONFIG[0] = NRF_GPIOTE->CONFIG[0] | 1;
  NRF_GPIOTE->CONFIG[0] = NRF_GPIOTE->CONFIG[0] | (1 << 17);
  NRF_GPIOTE->CONFIG[0] = NRF_GPIOTE->CONFIG[0] | (28 << 8);
  NRF_GPIOTE->INTENSET = NRF_GPIOTE->INTENSET | 1;
  NVIC_EnableIRQ (GPIOTE_IRQn);
  gpio_config(28, INPUT);
  gpio_config(22, INPUT);
  gpio_config(23, OUTPUT);
  //Do stuff for last checkoff
  NVIC_SetPriority(0, GPIOTE_IRQn);
  NVIC_SetPriority(5, SWI1_EGU1_IRQn);

  software_interrupt_init ();
  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // loop forever
  while (1) {
    //printf("Looping\n");
    software_interrupt_generate ();
    //if (!gpio_read(22))
    //__WFI();


    nrf_delay_ms(1000);

  }
}
