//This is the Cpp file for the actuator class
#include <Arduino.h>
#include "actuator.h"
#include <Wire.h>

//Constructer function for the class, 
Actuator::Actuator(byte encAddr){
        slaveadress = encAddr;                                      // Store actuator slave adress, (arduino nano slave adress)
}

//PID algorithm function
void Actuator::computePID(){     
        currentTime = millis();                                    // Get current time
        elapsedTime = (double)(currentTime - previousTime);        // Compute time elapsed from previous computation
        
        error = _setPoint - angle;                                 // Determine error
        cumError += error * elapsedTime;                           // Compute integral
        rateError = (error - lastError)/elapsedTime;               // Compute derivative

        float ui= _ki*cumError;//

        if ( ui > 400){                                           // Anti windup for integrator ui
          ui = 400;
        }

        if (ui < -400){
          ui= -400;
        }
 
        double out = _kp*error + ui + _kd*rateError;             // PID output               
        
        if (out > 400)                                           // Actuation Saturation for our speed driver. 
        {
                out = 400;
        }

        if (out < -400)
        {
                out = 400;
        }
        
        
        lastError = error;                                        // Remember current error
        previousTime = currentTime;                               // Remember current time

        output = out;                                             // Store output. MAX 400, MIN -400
}


//Set function for setpoint
void Actuator::setSetpoint(float r){
        _setPoint = r;                                                //Updates setpoint
}


//Set function for parameters
void Actuator::setParameters(double kp, double ti, double td ){  
        _kp=kp;                                                       //Updates Kp
        _ki=ti;                                                       //Updates Ti
        _kd=td;                                                       //Updates Td
}

// Get function for angle
float Actuator::getAngle(){
        return angle;
}       

//Get function for setoint
float Actuator::getSetpoint(){
        return _setPoint;
}

//Get function for speed
int Actuator::getSpeed(){
  return output;
}

//Reads encoder counter from slave and converts to angle
void Actuator::readAngle(){                                     
    Wire.beginTransmission(slaveadress);                           // Starts transmition with slaver acturtor slave
    int available = Wire.requestFrom(slaveadress, (uint8_t)2);     // Requests bytes from slave
  
    if(available == 2)                                             // Checks if 2 bytes are avavible
    {
       counter = Wire.read() << 8 | Wire.read();                   // Reads upper and lower byte for encoder counter and converts to int (0-1023)
    }
    else                                                           // Error in transmition
    {
      Serial.print("Unexpected number of bytes received: ");
      Serial.println(available);
    }

    int result = Wire.endTransmission();                           // End transmition, store result
    if(result)                                                     // check if sucessfulll
    {
      Serial.print("Unexpected endTransmission result: ");
      Serial.println(result);
    }
     
     angle = (360.0*(float)counter)/9600;                          // Converts to degrees (0-360)
}
