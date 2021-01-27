//Deklerasjon for motor klassen
#ifndef actuator_h
#define actuator_h

#include <Arduino.h>

class Actuator {
    private:
        int encoderPin; //Witch pin the encoder is connected to
        int outPin;     //what pin is the power of the motor connected to

        double angle;
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
        void setSetpoint(int r);       //Updates the setpoint (0-360)
        void setOutput(); // Calculate the analog value for the angle
        void getAngle(); // Calculate the analog value for the angle
        void computePID();  //Computes the pid value, this is an angle                 
        

};

#endif