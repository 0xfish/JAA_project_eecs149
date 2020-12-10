#include <stdint.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"

#include "HCSR04.h"

//Global Variables
// count to us (micro seconds) conversion factor
// set in start_timer()
static volatile float countToUs = 1;
static volatile uint32_t tCount = 0;
APP_TIMER_DEF(pulse_timer);

// HC-SR04 Trigger - P0.03
uint32_t pinTrig = 4;
// HC-SR04 Echo - P0.04
uint32_t pinEcho = 3;
/*Call back function for the interrupt handler.*/
void timerCounter() {
  tCount++;
}

/*Only setups the pins.*/
void setupPins() {
  //Set the direction of the pins
  //PinTrig is input to the sensor.
  nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
  //pinEcho is the output to be read from.
  nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);
}

/*Setup distance sensor.*/
void setupUS() {
  //Set the direction of the pins
  //PinTrig is input to the sensor.
  nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
  //pinEcho is the output to be read from.
  nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);
  //Setup timer for 100 miliseconds.
  app_timer_init();
  app_timer_create(&pulse_timer, APP_TIMER_MODE_REPEATED, timerCounter);
  app_timer_start(pulse_timer, APP_TIMER_TICKS(100), NULL);
}

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

bool getDistance(float *dist) {
  sendTrigger();

  // wait till Echo pin goes high
  while(!nrf_gpio_pin_read(pinEcho));

  // reset counter
  tCount = 0;
  // wait till Echo pin goes low
  while(nrf_gpio_pin_read(pinEcho));
  __disable_irq();
  // calculate duration in us
  float duration = countToUs*tCount;

  // dist = duration * speed of sound * 1/2
  // dist in cm = duration in us * 10^-6 * 340.29 * 100 * 1/2
  float distance = duration*0.017*100;
  // save
  *dist = distance;
  // check value
  printf("TimerCount is:%ld\t", tCount);
  if(distance < 400.0) {
    return true;
  }
  else {
    return false;
  }
}
