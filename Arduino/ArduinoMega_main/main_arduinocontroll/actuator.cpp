//This is the Cpp file for the actuator class
#include <Arduino.h>
#include "actuator.h"
#include <Wire.h>

//Constructor function for the class,
Actuator::Actuator(byte encAddr, float p, float i, float d, int gr) {
  slaveadress = encAddr;                                     // Store actuator slave adress, (arduino nano slave adress)
  Kp = p;
  Ti = i;
  Td = d;
  gearRatio = gr;
}

//PID algorithm function
void Actuator::computePID() {
  currentTime = millis();                                    // Get current time
  elapsedTime = (float)(currentTime - previousTime);         // Compute time elapsed from previous computation. THIS IS in ms!
  rateLimit = abs(velRef / (elapsedTime));                // Set ratelimit to desiered velocity from moveit, scaled down by 1/1
  setpointRateLimit();
  error = setpointRated - ang;                               // Determine error

  if (!windup && Ti > 0.0) {
    cumError += error * elapsedTime;                         // Compute integral
    ui =  cumError / Ti;                                     // Calc integrator part if not windup
  }
  if (Td > 0.0) {
    ud = ((error - lastError) * (Kp * Td)) / elapsedTime;
  }
  float  out = Kp * error + ui + ud;                         // PID output

  if ( out < 400 && out > -400) {                            // not windup if not within bounds
    windup = false;
  }

  if (out > 400) {                                           // if  out of bounds, set activate windup
    out = 400;
    windup = true;
  }

  if (out < -400) {
    out = -400;
    windup = true;
  }

  velocity = ((ang - prevAngle) / 180 * 3.14 ) / (elapsedTime * 1000);    // Calculate angular velocity rad/s

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
//Set function for rated setpoint
void Actuator::setRatedSetpoint(float r) {
  setpointRated = r;                                              //Updates setpoint
}
//Set function for parameters
void Actuator::setParameters(float p, float ki) {
  Kp = p;                                                    // Updates Kp
  Ti = ki;                                                   // Updates Ti
}
//Set function for current [mA]
void Actuator::setAmps(unsigned int amp) {
  amps = amp;
}
//Set function for the ratelimit
void Actuator::setDesieredVelocity(float vel){
  velRef = vel;
}
// Get function for angle
float Actuator::getAngle() {
  return ang;
}

//Get function for setoint
float Actuator::getSetpoint() {
  return setPoint;
}

float Actuator::getRatedSetPoint() {
  return setpointRated;
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
//Get function for current [mA]
unsigned int Actuator::getAmps() {
  return amps;
}

//Reads encoder counter from slave and converts to angle
void Actuator::readAngle() {
  Wire.beginTransmission(slaveadress);                           // Starts transmission with slaver actuator slave
  int available = Wire.requestFrom(slaveadress, (uint8_t)2);     // Requests bytes from slave

  if (available == 2) {                                          // Checks if 2 bytes are avavible
    counter = Wire.read() << 8 | Wire.read();                    // Reads upper and lower byte for encoder counter and converts to int (0-1023)
  }
  else {                                                         // Error in transmission
    Serial.print("Unexpected number of bytes received: ");
    Serial.println(available);
  }
  int result = Wire.endTransmission();                           // End transmission, store result
  if (result) {                                                  // check if sucessfulll
    Serial.print("Unexpected endTransmission result: ");
    Serial.println(result);
  }
  ang = (360.0 * (float)counter) / gearRatio;                       // Converts to degrees (0-360)
}

//Rate limit the setpoint
void Actuator::setpointRateLimit() {
  float diff = setPoint - setpointRated;

  if (setPoint > setpointRated) {                             // If setpoint larger then rated increase rated setpoint
    if (diff > rateLimit) {
      setpointRated = setpointRated + rateLimit;
    }
    else {
      setpointRated = setpointRated + diff;
    }
  }

  if (setPoint < setpointRated) {                             // If setpoint smaller then rated decrease rated setpoint
    diff = diff * -1.0;
    if (diff > rateLimit) {
      setpointRated = setpointRated - rateLimit;
    }
    else {
      setpointRated = setpointRated - diff;
    }
  }
}
