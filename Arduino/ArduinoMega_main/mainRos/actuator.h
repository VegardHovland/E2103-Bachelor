//Header file for acuator class
#ifndef actuator_h
#define actuator_h
#include <Arduino.h>
#include <Wire.h>

class Actuator {
  private:
    byte slaveadress;                    // Slave adress for a given actuator
    bool windup;                         // windup indication flag

    int counter;                         // Store counter from incremental encoder
    int gearRatio;                       // The acuators gear ratio
    unsigned int amps;                   // Store how much amps it is drawing

    float ang, prevAngle;                // The acuators angle andprevious angle
    float Kp, Ti, Td;                    // Controller constants
    float ui = 0;                        // Integrator part
    float ud = 0;                        // Derivative part
    float elapsedTime;                   // Scan time
    float error, lastError, cumError ;   // Store last error
    float prevOut, output, setPoint;     // setpoints and outputs
    float velocity;
    float rateLimit;
    float setpointRated;                 // Rate limited setpoint
    float velRef;                        // Desiered velocity

    unsigned long currentTime;           // store current time
    unsigned long previousTime;          // Keep track of the previousTime for given actuator

  public:
    Actuator(byte encAddr, float p, float i, float d, int gr);  // Constructor function for actuator class

    //Set functions
    void setSetpoint(float r);                                  // Set function for setpoint
    void setRatedSetpoint(float r);                             // Set function for rate limited setpoint
    void setParameters(float kp, float ti);                     // Set fucntion for PID parameters
    void setDesieredVelocity(float lim);                        // Set function for desiered velocity
    void setAmps(unsigned int amp);                             // Set function for current readings in [mA]
    void setSetpointRateLimit();                                // Set funciton for current rate limit for seetpoint

    // Get functions
    float getKp();                                              // Get function for PID parameters
    float getTi();
    float getTd();
    float getRatedSetPoint();                                   // Get function for the rated setpoint
    float getAngle();                                           // Get function for angle
    float getSetpoint();                                        // Get function for actuator setpoint
    float getVelocity();                                        // Get function for current velocity
    int getEffort();                                            // Get function for actuator speed / effort
    unsigned int getAmps();                                     // Get function for current readings in [mA]

    void readAngle();                                           // Calculate the angle(-360,360) from the counter given by the encoder in the slave
    void computePID();                                          // Computes the pid value, this is an angle


};

#endif
