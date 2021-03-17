//This is the Cpp file for the actuator class
#include <Arduino.h>
#include "actuator.h"
#include <Wire.h>

//Constructer function for the class,
Actuator::Actuator(byte encAddr) {
  slaveadress = encAddr;                                     // Store actuator slave adress, (arduino nano slave adress)
}

//PID algorithm function
void Actuator::computePID() {
  currentTime = millis();                                    // Get current time
  elapsedTime = (float)(currentTime - previousTime);         // Compute time elapsed from previous computation

  error = setPoint - ang;                                  // Determine error

  if (!windup && Ti > 0.0) {
    cumError += error * elapsedTime;                         // Compute integral
    ui =  cumError / Ti;                                     //Calc integator part if not windup
  }
  if (Td > 0.0) {
    beta = Tf / (elapsedTime + Tf);
    ud = (beta * ud) - (((1.0 - beta) * (output - prevOut) * (Kp * Td)) / elapsedTime);
  }
  float  out = Kp * error + ui + ud;                          // PID output

  if ( out < 400 && out > -400) {                             // not windup if not within bounds
    windup = false;
  }

  if (out > 400) {                                            // if  out of bounds, set activate windup
    out = 400;
    windup = true;
  }

  if (out < -400) {
    out = -400;
    windup = true;
  }

  velocity = ((ang - prevAngle) / 180 * 3.14 ) / (elapsedTime * 1000);

  previousTime = currentTime;                                // Remember current time
  prevOut = out;
  lastError = error;                                         // Remember last error
  output = out;                                              // Store output. MAX 400, MIN -400
  prevAngle = ang;
}


//Set function for setpoint
void Actuator::setSetpoint(float r) {
  setPoint = r;                                              //Updates setpoint
}


//Set function for parameters
void Actuator::setParameters(float p, float ki) {
  Kp = p;                                                    // Updates Kp
  Ti = ki;                                                   // Updates Ti
}

// Get function for angle
float Actuator::getAngle() {
  return ang;
}

//Get function for setoint
float Actuator::getSetpoint() {
  return setPoint;
}
// Get function for current velocity
float Actuator::getVelocity() {
  return velocity;
}

// Get functions for PID parameters
float Actuator::getKp() {
  return Kp;
}
float Actuator::getTi() {
  return Ti;
}
//Get function for speed
int Actuator::getEffort() {
  return (int)output;
}

//Reads encoder counter from slave and converts to angle
void Actuator::readAngle() {
  Wire.beginTransmission(slaveadress);                           // Starts transmition with slaver acturtor slave
  int available = Wire.requestFrom(slaveadress, (uint8_t)2);     // Requests bytes from slave

  if (available == 2) {                                          // Checks if 2 bytes are avavible
    counter = Wire.read() << 8 | Wire.read();                    // Reads upper and lower byte for encoder counter and converts to int (0-1023)
  }
  else {                                                         // Error in transmition
    Serial.print("Unexpected number of bytes received: ");
    Serial.println(available);
  }
  int result = Wire.endTransmission();                           // End transmition, store result
  if (result) {                                                  // check if sucessfulll
    Serial.print("Unexpected endTransmission result: ");
    Serial.println(result);
  }
  ang = (360.0 * (float)counter) / 9600;                       // Converts to degrees (0-360)
}
