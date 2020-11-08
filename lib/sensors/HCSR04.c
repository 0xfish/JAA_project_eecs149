#ifndef HCSR04
#define HCSR04

#include "HCSR04.h"

void sendTrigger() {
    // send 12us trigger pulse
    //    _
    // __| |__
    nrf_gpio_pin_clear(pinTrig);
    nrf_delay_us(20);
    nrf_gpio_pin_set(pinTrig);
    nrf_delay_us(12);
    nrf_gpio_pin_clear(pinTrig);
    nrf_delay_us(20);
}

float getDistance() {


    
}


#endif