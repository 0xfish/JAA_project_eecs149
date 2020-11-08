#include "gpio.h"
#define OUT 0x50000504
pins* p = (pins*) OUT;

// Inputs:
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
  if (dir == INPUT) {
    p->pin_cnf[gpio_num] = p->pin_cnf[gpio_num] | 1;
    p->pin_cnf[gpio_num] = p->pin_cnf[gpio_num] | (1 << 1);
  } else {
    p->pin_cnf[gpio_num] = p->pin_cnf[gpio_num] & ~((uint32_t) 1 << 1);
    p->pin_cnf[gpio_num] = p->pin_cnf[gpio_num] & ~((uint32_t) 1);
  }
}

// Set gpio_num high
// Inputs:
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
  p->pin_cnf[gpio_num] = 0x0001;
  //p->out = p->out | (1 << gpio_num);
}

// Set gpio_num low
// Inputs:
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
//  p->out = p->out & ~(1 << gpio_num);
  p->pin_cnf[gpio_num] = 0x0000;
}

// Inputs:
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    // should return pin state
    p->pin_cnf[gpio_num] = 0x0000;
    return (p->in & (1 << gpio_num)) == 0;//>> gpio_num;
}
