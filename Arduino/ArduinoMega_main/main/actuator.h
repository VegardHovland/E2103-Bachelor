//Header file for acuator class
#ifndef actuator_h
#define actuator_h
#include <Arduino.h>
#include <Wire.h>

class Actuator {
  private:
    int counter;              // Store counter from incremental encoder
    byte slaveadress;         // Slave adress for a given actuator

    float angle;
    int maxAngle = 170;      // Max operating angle for the motor
    int minAngle = 10;       // Min -----------------------------
    float Kp = 200;           // Controller constants
    float Ti = 0.005;
    float Td = 0;

    unsigned long currentTime;
    unsigned long previousTime;                                 // Keep track of the previousTime for given actuator
    float elapsedTime;                                          // Scan time
    float error;
    float lastError;                                            // Store last error
    float input, output, setPoint;
    float cumError, rateError;                                  // Integral and derivative variables

  public:
    Actuator(byte encAddr);                                     // Constructor function for actuator class
    void setSetpoint(float r);                                  // Set function for setpoint
    void setParameters(double kp, double ti, double td);        // Set fucntion for PID parameters
    float getKp();                                              // Get function for PID parameters
    float getTi();                                              // Get function for PID parameters
    float getTd();                                              // Get function for PID parameters
    float getAngle();                                           // Get function for angle
    void readAngle();                                           // Calculate the angle(-360,360) from the counter given by the encoder in the slave
    void computePID();                                          // Computes the pid value, this is an angle
    float getSetpoint();                                        // Get function for actuator setpoint
    int getSpeed();                                             // Get function for actuator speed
};

#endif
