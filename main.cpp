//Hovedprogrammet, her skal minst mulig kode skrives og brukes est mulig funksjoner!
//Regler vi bruker upperLowerCase
//HUSK Å KOMMENTERE!!


#include <Arduino.h>
#include "functions.h"
#include "constants.h"
#include "actuator.h"



Actuator actuators[] = {Actuator(2 , 3), Actuator(4 ,5),Actuator(5 ,6), Actuator(7 ,8) };

  
void setup() {


}

void loop() {

     for (int i = 0; i < numActuators ; i++)
    {
        actuators[i].getAngle();
        actuators[i].computePID();
        actuators[i].setOutput();
    }
}