//Header file for acuator class
#ifndef actuator_h
#define actuator_h
#include <Arduino.h>
#include <Wire.h>

class Actuator {
  private:
    int counter;              // Store counter from incremental encoder. 
    byte slaveadress;         // Slave adress for a given actuator.
    bool windup;              // indicates windup on integrator.

    float angle;
    float Kp = 2;                                              // Controller constants, tuned for fastest respons without overshoot
    float Ti = 1500 ;
    float ui;                                                   // Integrator part
    unsigned long currentTime;
    unsigned long previousTime;                                 // Keep track of the previousTime for given actuator
    float elapsedTime;                                          // Scan time
    float error;
    float lastError;                                            // Store last error
    float input, output, setPoint;
    float cumError;                                             // Integral

  public:
    Actuator(byte encAddr);                                     // Constructor function for actuator class
    void setSetpoint(float r);                                  // Set function for setpoint
    void setParameters(double kp, double ti);                   // Set fucntion for PID parameters
    float getKp();                                              // Get function for PID parameters
    float getTi();                                              // Get function for PID parameters
    float getAngle();                                           // Get function for angle
    void readAngle();                                           // Calculate the angle(-360,360) from the counter given by the encoder in the slave
    void computePID();                                          // Computes the pid value, this is an angle
    float getSetpoint();                                        // Get function for actuator setpoint
    int getSpeed();                                             // Get function for actuator speed
};

#endif
