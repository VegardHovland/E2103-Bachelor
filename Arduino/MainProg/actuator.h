//Header file for acuator class
#ifndef actuator_h
#define actuator_h
#include <Arduino.h>
#include <Wire.h>

class Actuator {
    private:
        int encoderPin;           //pin for encoder
        int outPin;               //pin for output
        int encData;
        byte slaveadress;

        float angle;
        int maxAngle=170;        // Max operating angle for the motor
        int minAngle=10;         // Min -----------------------------
        double _kp=2;            // Controller constants
        double _ki=10;
        double _kd=5;
 
        unsigned long currentTime;                     
        unsigned long previousTime;                //Keep track of the previousTime for this object
        double elapsedTime;                        //Scan time 
        double error;
        double lastError;
        double input, output, _setPoint;
        double cumError, rateError;




    public:
        Actuator(byte encAddr, int uPin);                             // Constructor function for actuator class
        void setSetpoint(int r);                                   //Updates the setpoint (0-360)
        void setOutput();                                          // Calculate the analog value for the angle
        void setParameters(double kp, double ti, double td);       //Set new PID values    
        float getAngle();                                           // gets the joint angle of the actuator           
        void readAngle();             // Calculate the analog value for the angle
        void computePID();                                         //Computes the pid value, this is an angle   
                
        
};

#endif
