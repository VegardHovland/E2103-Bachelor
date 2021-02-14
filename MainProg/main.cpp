//Hovedprogrammet, her skal minst mulig kode skrives og brukes est mulig funksjoner!
//Regler vi bruker upperLowerCase (camelCase)
//HUSK Å KOMMENTERE!!!


#include <Arduino.h>
#include "functions.h"
#include "constants.h"
#include "actuator.h"



Actuator actuators[] = {Actuator(2 , 3), Actuator(4 ,5),Actuator(5 ,6), Actuator(7 ,8) };     //Generates a actuator list contaning 4 actuators

  
void setup() {

    Serial.begin(9600);                                                                       //Starts the serial monitor
}

void loop() {
if (Serial.available() > 0) {

    char ch = Serial.read();                                                                  //Gets user input

    switch (ch) {               

      case 'a':{                                                                             //Updates the setpoint for a given actuator
        Serial.print("Skriv in nr på motor");
        int i = Serial.read();
        Serial.print("nytt setpunkt i grader");
        double ang = Serial.read();
        actuators[i].setSetpoint(ang);
        break;
      }
      case 'b': {                                                                             //Updates the PID controller parameters for a given actuator
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
      case 'q': break;
        //returner til start konfigurasjon

      default :
      {
        controllActuators(actuators);                                                           //Pid controll on all the acuators by default
        break;
      }
    }
    }
}