// Virtual timers
//
// Uses a single hardware timer to create an unlimited supply of virtual
//  software timers

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "virtual_timer.h"

void led0_toggle() {
    nrf_gpio_pin_toggle(BUCKLER_LED0);
}

void led1_toggle() {
    nrf_gpio_pin_toggle(BUCKLER_LED1);
}

void led2_toggle() {
    nrf_gpio_pin_toggle(BUCKLER_LED2);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Board initialized!\n");

  // You can use the NRF GPIO library to test your timers
  nrf_gpio_pin_dir_set(BUCKLER_LED0, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(BUCKLER_LED1, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(BUCKLER_LED2, NRF_GPIO_PIN_DIR_OUTPUT);

  // Don't forget to initialize your timer library
  virtual_timer_init();
  nrf_delay_ms(3000);
  uint32_t id;
  // Setup some timers and see what happens
//  id = virtual_timer_start_repeated(1000000, led0_toggle);
//   virtual_timer_start_repeated(2000000, led0_toggle);

  //virtual_timer_start(5000000, led0_toggle);
  //virtual_timer_start_repeated(2000000, led1_toggle);
  // loop forever
  /*
  virtual_timer_start(1000000, led0_toggle);
  virtual_timer_start(2000000, led1_toggle);
  virtual_timer_start(3000000, led2_toggle);
  virtual_timer_start(4000000, led0_toggle);
  virtual_timer_start(5000000, led1_toggle);
  virtual_timer_start(6000000, led2_toggle);
  */
  virtual_timer_start_repeated (1000 , led0_toggle );
  virtual_timer_start_repeated (2000 , led1_toggle );
  uint32_t i = 0;
  while (1) {
    uint32_t timer4_val = read_timer();
    printf("Current time is %ld\n", timer4_val);
    nrf_delay_ms(1000);
  //  i ++;
  //  if (i == 5) {
      //virtual_timer_cancel(id);
  //  }
  }
}
