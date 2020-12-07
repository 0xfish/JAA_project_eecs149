#pragma once

#include <stdbool.h>




/*Get Distance then return true if distance is
valid, false otherwise.*/
bool getDistance(float* dist);

/*Sends the pulse trigger*/
void sendTrigger();

/*Increments the time as the call back function.*/
void timerCounter();
