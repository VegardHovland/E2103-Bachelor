//Cpp file for gloable functions


#include "functions.h"
#include "actuator.h"
#include "Arduino.h"
#include "constants.h"


//Controlls all the acuators
void controllActuators(Actuator actuators[]){
    for (int i = 0; i < numActuators; i++)              // Loops over the 4 actuator objects    
    {
        actuators[i].getAngle();                        //Get the actuators angle
        actuators[i].computePID();                      //comeputes output using PID 
        actuators[i].setOutput();                       //sets out the putput to the actuator
    }
}