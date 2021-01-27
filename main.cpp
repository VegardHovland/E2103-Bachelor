//Hovedprogrammet, her skal minst mulig kode skrives og brukes est mulig funksjoner!
//Regler vi bruker upperLowerCase
//HUSK Å KOMMENTERE!!


#include <Arduino.h>
#include "functions.h"
#include "constants.h"
#include "actuator.h"



Actuator actuators[] = {Actuator(2 , 3), Actuator(4 ,5),Actuator(5 ,6), Actuator(7 ,8) };

  
void setup() {

    Serial.begin(9600);



}

void loop() {
if (Serial.available() > 0) {

    char inByte = Serial.read();

    switch (inByte) {

      case 'a':{
        Serial.print("Skriv in nr på motor");
        int i = Serial.read();
        Serial.print("nytt setpunkt i grader");
        double ang = Serial.read();
        actuators[i].setSetpoint(ang);
        break;
      }
      case 'b': {
        Serial.print("Skriv in nr på motor");
        int i = Serial.read();
        Serial.print("kp");
        double kp = Serial.read();
        Serial.print("ti");
        double ti = Serial.read();
        Serial.print("td");
        double td = Serial.read();
        actuators[i].setParameters(kp, ti, td);
        break;
      }
      case 'c': break;
        //???

      default :
      {
        controllActuators(actuators); //Pid controll on all the acuators
        break;
      }
    }
    }
}