#include "Arduino.h"
#include "actuator.h"

Actuator::Actuator(int enPin, int oPin){
        pinMode(enPin, INPUT);
        pinMode(oPin, OUTPUT);

        encoderPin=enPin;
        outPin=oPin;
}


