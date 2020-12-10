#pragma once

#include <stdbool.h>



/*Setups the entire distance sensor.*/
void setup_dist();

/*Setups only the pins*/
void setupPins();

/*Get Distance then return true if distance is
valid, false otherwise.*/
bool getDistance(float* dist);

/*Sends the pulse trigger*/
void sendTrigger();

/*Increments the time as the call back function.*/
void timerCounter();
