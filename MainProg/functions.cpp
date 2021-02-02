//Her defineres alle funksjonene
#include "functions.h"
#include "actuator.h"
#include "Arduino.h"
#include "constants.h"

void controllActuators(Actuator actuators[]){
    for (int i = 0; i < numActuators; i++)
    {
        actuators[i].getAngle();   //Get the actuators angle
        actuators[i].computePID(); //comeputes output using PID 
        actuators[i].setOutput();  //sets out the putput to the actuator
    }
}