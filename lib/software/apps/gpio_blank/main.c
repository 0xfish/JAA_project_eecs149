// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "gpio.h"

//Macros for GPIO
#define OUT 0x50000504
#define dIR 0x50000514

uint32_t* out = (uint32_t*) OUT;
uint32_t* dir = (uint32_t*) dIR;

typedef struct {
  uint32_t out;
  uint32_t outset;
  uint32_t outclr;
  uint32_t in;
  uint32_t dir;
  uint32_t dirset;
  uint32_t dirclr;
  uint32_t latch;
  uint32_t detectmode;
  uint8_t dummy[0x700 - 0x524 - 4];
  uint32_t pin_cnf[32];
} registers;

registers* reg = (registers*) OUT;

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");



  printf("Out Value:%x DIR Value:%x\n", *out, *dir);
  printf("Out Address:%x DIR Address:%x\n", out, dir);
  // loop forever
/*
  *dir = *dir | (1 << 23);
  *dir = *dir | (1 << 24);
  *dir = *dir | (1 << 25);
  //Button and switches
  reg->dir = reg->dir & ~(1 << 28);
  reg->dir = reg->dir & ~(1 << 22);
  reg->pin_cnf[28] = reg->pin_cnf[28] & ~(1 << 1);
  reg->pin_cnf[22] = reg->pin_cnf[22] & ~(1 << 1);
  */
  gpio_config(22, INPUT);
  gpio_config(24, OUTPUT);
while (1) {
  /*nrf_delay_ms(500);
  *out = *out ^ (1 << 23);
  *out = *out ^ (1 << 24);
  *out = *out ^ (1 << 25);

  uint32_t b0 = reg->in & (1 << 28);
  uint32_t s0 = reg->in & (1 << 22);
  printf("addr: %b\n", reg->in);
  printf("B0:%x , S0:%x\n", b0 >> 28, s0 >> 22);
  */
  printf("BUTTON0 %d\n", gpio_read(28));
  if (gpio_read(22) == 1) {
    gpio_set(24);
  } else {
    gpio_clear(24);
  }
}
}
