#include "Arduino.h"
#include "actuator.h"

Actuator::Actuator(int enPin, int oPin){
        pinMode(enPin, INPUT);
        pinMode(oPin, OUTPUT);

        encoderPin =enPin;
        outPin=oPin;
}


void Actuator::computePID(){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = _setPoint - angle;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = _kp*error + _ki*cumError + _kd*rateError;                //PID output               
        
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
 
         angle = out;                                        //have function return the PID output
}

void Actuator::setSetpoint(int r){
        _setPoint = r;
}

void Actuator::setOutput(){
        output= (angle*255.0)/360.0;
        analogWrite(outPin, output);
}

void Actuator::getAngle(){
        input = analogRead(encoderPin) ; //Reads a value between 0 and 1023
        angle = (input * 360) / 1023 ;
}

void Actuator::setParameters(double kp, double ti, double td ){  //Oppdaterer regulator parameterene
        _kp=kp;
        _ki=ti;
        _kd=td;
}