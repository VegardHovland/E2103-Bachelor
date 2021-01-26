//Deklerasjon for motor klassen
#ifndef actuator_h
#define actuator_h

#include "Arduino.h"

class Actuator {
    private:
        int encoderPin;
        int outPin;
        int maxAngle;
        int minAngle;
        float currentAngle;
        float desieredAngle;
        float Kp;
        float Ti;
        float Td;
        float Dn;




    public:
        Actuator(int enPin, int uPin);
        void getCurrentAngle();
        void getDesieredAngle();
        void setDesieredAngle();
        

};

#endif