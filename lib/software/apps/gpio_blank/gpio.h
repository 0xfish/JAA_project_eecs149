#pragma once

#include "nrf.h"
#include "stdbool.h"

typedef enum {
    INPUT = 0,
    OUTPUT,
} gpio_direction_t;

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
} pins;

// Inputs:
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir);

// Inputs:
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num);

// Inputs:
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num);

// Inputs:
//  gpio_num - gpio number 0-31
// Returns:
//  current state of the specified gpio pin
bool gpio_read(uint8_t gpio_num);
