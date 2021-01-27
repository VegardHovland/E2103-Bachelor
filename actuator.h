//Deklerasjon for motor klassen
#ifndef actuator_h
#define actuator_h

#include "Arduino.h"

class Actuator {
    private:
        int encoderPin; //Witch pin the encoder is connected to
        int outPin;     //what pin is the power of the motor connected to

        int maxAngle=170;     // Max operating angle for the motor
        int minAngle=10;     // Min -----------------------------
        double kp=2;     //Controller constants
        double ki=10;
        double kd=5;
 
        unsigned long currentTime;
        unsigned long previousTime;  //Keep track of the scantime
        double elapsedTime;                        //Scan time 
        double error;
        double lastError;
        double input, output, setPoint;
        double cumError, rateError;




    public:
        Actuator(int enPin, int uPin);
        void getCurrentAngle();
        void getDesieredAngle();
        void setDesieredAngle();
        void setSetpoint(int r);
        double computePID(double inp);                   
        

};

#endif