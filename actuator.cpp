#include "Arduino.h"
#include "actuator.h"

Actuator::Actuator(int enPin, int oPin){
        pinMode(enPin, INPUT);
        pinMode(oPin, OUTPUT);

        encoderPin =enPin;
        outPin=oPin;
}


double Actuator::computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
        
        if (out > maxAngle)
        {
                out = maxAngle;
        }

        if (out < minAngle)
        {
                out = minAngle;
        }
        
        
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}

void Actuator::setSetpoint(int r){
        setPoint = r;
}