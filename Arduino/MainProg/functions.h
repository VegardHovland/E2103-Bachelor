//Header file for global functions
#include <Arduino.h>
#include "actuator.h"
#include "constants.h"

#ifndef functions_h
#define functions_h



void controllActuators(Actuator acts[]);    //comutes the PID algorithm on every actuator
void printText(Actuator acts[]);                //Displays data on oled 1
void drawGraph(Actuator acts[]);                //Draw grap on oled 2
void setupOled();                          //initialize oled displays


#endif