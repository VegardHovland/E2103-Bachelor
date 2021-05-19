/* Main scketch for our bachelorthesis, group E2103 at NTNU.
 *  Control from serial monitor
 
   Title: Development of Biomimetic Robot Leg With ROS Implementation.
   Group members: Henrike Moe Arnesen, Kristian Grinde, Vegard Hovland, Even vestland,
   Supervisor: Torleif Anstensrud
*/
#include "actuator.h"
#include "DualG2HighPowerMotorShield.h"
#include "variables.h"                                                                                                            
#include <stdlib.h>

// Init motordriverers as md1(motor driver 1) and md2(motor driver 2)
DualG2HighPowerMotorShield24v14 md1(M11nSLEEP, M11DIR, M11PWM,  M11nFAULT,  M11CS, M12nSLEEP,  M12DIR,  M12PWM,  M12nFAULT, M12CS);                // Declaration for Motor driver 1
DualG2HighPowerMotorShield24v14 md2(M21nSLEEP, M21DIR, M21PWM,  M21nFAULT,  M21CS, M22nSLEEP,  M22DIR,  M22PWM,  M22nFAULT, M22CS);                // Declaration for Motor driver 2  

//Object array of the 4 actuators
Actuator actuators[4] = {Actuator(8, 80, 0, 0.005, 3200), Actuator(9, 70, 0, 30, 4480), Actuator(10, 70, 0, 0.30, 4480), Actuator(11, 50, 0 , 30, 4480)};                                                                   // Generates an actuator list contaning 4 actuators and their i2c address

//Declare functions--------------------------------------------------------------------------------------------------------------------------------------------------------------
void controllActuators(Actuator acts[]);               // Comutes the PID algorithm on every actuator
void stopIfFault();                                    // Disable the motordrivers if there is a fault
void setupDrivers();                                   // Start the motor drivers
void shutDown();                                       // Return to start and turn MOSFET off
void updateSetpointSerial();                           // Update setpoint for an actuator using serial input
void updateParametersSerial();                         // Update parameters for an actuator using serial input
void printMenu();                                      // Print menu for switch case
void serialPrintData();
void serialPlot();

//Setup function for Arduino-------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();                                       // Initialize wire
  setupDrivers();
  printMenu();
  delay(10);
}

// Main Loop----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
    if (Serial.available() > 0) {
      char ch = Serial.read();                     // Gets user input if there is an input
      switch (ch) {
        case 'a': {                                // Updates the setpoint for a given actuator
            updateSetpointSerial();
            break;
          }
        case 'b': {                                // Updates the PID controller parameters for a given actuator using serial inputs
            updateParametersSerial();
            break;
          }
        case 'd': {                                // Print all data for actuators
            serialPrintData();
            break;
          }
        case 'p': {                                // Print menu
            printMenu();
            break;
          }
        case 'c': {                                // Run calibration cyclus
            //  calibrationData();
            break;
          }
        case 't': {                                // Run follow trajectory test
            //followTrajTest();
            break;
          }
        case 'q': {                                // Return to startposition and turn off
            shutDown();
            break;
          }
  
        default : break;
      }
    } 
  stopIfFault();                                  // Turn of drivers and stop program
  controllActuators(actuators);                   // Pid controll on all the acuators by default
  serialPlot();                                   // Plot serial information, cosumize this as needed
}


//Function definitions--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//Print menu for user inputs
void printMenu() {
  Serial.println("Press 'a' to update setpoint");
  Serial.println("Press 'b' to update PID parameters");
  Serial.println("Press 'd' to display information");
  Serial.println("Press 'p' to print menu");
  Serial.println("Press 't' to run follow trajectory test");
  Serial.println("Press 'q' to exit");
}

//Function to update setpoint for a given actuator using serial read
void updateSetpointSerial() {
  while (Serial.available()) {
    int trash = Serial.read(); //Clear trash from input buffer
  }

  Serial.println("Skriv in nr på motor (1 - 4)");                // Ask for input
  while (!Serial.available()) {};                                // Wait for input
  int i = Serial.parseInt() - 1;                                 // Get act index (0-3)

  Serial.println("nytt setpunkt i grader");                      // Ask for input
  while (!Serial.available()) {};                                // Wait for input
  double ang = Serial.parseFloat();                              // Get new setpoint

  actuators[i].setSetpoint(ang);                                 // Update setpoint for giver actuator
  Serial.println("setpoint updated, press d to display data");   // confirm success
}

//Function to update parameters for a given actuator using serial read
void updateParametersSerial() {
  while (Serial.available()) {
    int trash = Serial.read();                                   //Clear trash from input buffer
  }
  md1.setSpeeds(0, 0);                                           // Set speed to 0 for motor 1 and 2.
  md2.setSpeeds(0, 0);                                           // Set speed to 0 for motor 3 and 4.

  Serial.println("Skriv in nr på motor (1 - 4)");                // Ask for input
  while (!Serial.available()) {};                                // Wait for input
  int i = Serial.parseInt() - 1;                                 // Get act indx (1 - 3)

  Serial.println("kp");
  while (!Serial.available()) {};
  float kp = Serial.parseFloat();                                // Get new kp

  Serial.println("ti");
  while (!Serial.available()) {};
  float ti = Serial.parseFloat();                                // Get new Ti

  actuators[i].setParameters(kp, ti);                            // Set new parameters for given actuator
  Serial.println("parameters updated, press d to display data"); // confirm success
}

//Controlls all the actuators
void controllActuators(Actuator actuators[]) {
  for (int i = 0; i < numActuators; i++) {                       // Loops over the 4 actuator objects
    actuators[i].readAngle();                                    // Get the actuators angle
    actuators[i].computePID();                                   // Computes output using PID
    if ( i == 0) {
      md1.setM1Speed(actuators[i].getEffort());                  // Motor 1 is driver 1 M1
      actuators[i].setAmps(md1.getM1CurrentReading());
    }
    else if ( i == 1) {
      md1.setM2Speed(actuators[i].getEffort());                  // Motor 2 is driver 1 M2
      actuators[i].setAmps(md1.getM2CurrentReading());
    }
    else  if ( i == 2) {
      md2.setM1Speed(actuators[i].getEffort());                  // Motor 3 is driver 2 M1
      actuators[i].setAmps(md2.getM1CurrentReading());
    }
    else if ( i == 3) {
      md2.setM2Speed(actuators[i].getEffort());                  // Motor 4 is driver 2 M2
      actuators[i].setAmps(md2.getM2CurrentReading());
    }
  }
}


//Disable motordriver if fault
void stopIfFault() {
  if (md1.getM1Fault() || md1.getM2Fault()) {                     // Checks if fault on driver 1
    md1.disableDrivers();                                         // Disable driver 1
    delay(1);
    Serial.println("M fault");
    while (1);                                                    // Stop program
  }
  if (md2.getM2Fault() || md2.getM1Fault()) {                     // Checks if fault on driver 2
    md2.disableDrivers();                                         // Disable driver 2
    delay(1);
    Serial.println("M fault");
    while (1);                                                    // Stop program
  }
}

//Setup function for motor drivers
void setupDrivers() {
  md1.init();                                                   // Init pinmodes driver 1
  md1.calibrateCurrentOffsets();
  md2.init();                                                   // Init pinmodes driver 2
  md2.calibrateCurrentOffsets();
  md1.enableDrivers();                                          // Enable mosfet 1
  md2.enableDrivers();                                          // Enable mosfet 2
  for (int i = 0; i < numActuators; i++) {
    actuators[i].setSetpoint(startPos[i]);                      //Initialize start setpoints
    actuators[i].setRatedSetpoint(startPos[i]);
  }
  delay(50);                                                    // Enableing needs some time so delay
}

//return to start position and shutdown
void shutDown() {
  Serial.println("Shutting down");
  for (int i = 0; i < numActuators; i++) {
    actuators[i].setSetpoint(startPos[i]);                      // Updates setpoints to startposition
  }
  int shutdowntime = millis() + 5000;                           //Generate a shutdown time
  int currTime = millis();
  while (shutdowntime > currTime) {                             // Computes pid for actuators during shutdown time
    controllActuators(actuators);                               // Pid controll on all the acuators
    currTime = millis();                                        // Print data to be plotted in serial plot
  }
  md1.setSpeeds(0, 0);                                          // Set speed to 0 for motor 1 and 2. NB! should already be 0
  md2.setSpeeds(0, 0);                                          // Set speed to 0 for motor 3 and 4. NB! should already be 0
  delay(50);
  md1.disableDrivers();                                         // Turn of mosfet 1
  md2.disableDrivers();                                         // Turn of mosfet 2
  Serial.print("shutdown complete");
  while (1);                                                    // Stops program, require restart.
}

// Print information in serial monitor, display command
void serialPrintData() {
  for (int i = 0; i < numActuators; i++) {
    Serial.print(i + 1);                                         //Print motor nr (1-4)
    Serial.println(" : ");

    Serial.print("angle ");                                      // Prints joint angle in serial monitor
    Serial.print(": ");
    Serial.println(actuators[i].getAngle());

    Serial.print("setpoint ");
    Serial.print(i + 1);                                         // Prints setpoint in serial monitor
    Serial.print(": ");
    Serial.println(actuators[i].getSetpoint());

    Serial.print("Kp: ");                                        //Print parameters
    Serial.println(actuators[i].getKp());
    Serial.print("Ti: ");
    Serial.println(actuators[i].getTi());

  }
  int amps [4];
  amps[0] = md1.getM1CurrentMilliamps();                         // Get current for motor 1
  amps[1] = md1.getM2CurrentMilliamps();                         // Get current for motor 2
  amps[2] = md2.getM1CurrentMilliamps();                         // Get current for motor 3
  amps[3] = md2.getM2CurrentMilliamps();                         // Get current for motor 4

  for (int i = 0; i < numActuators; i++) {
    Serial.print("current ");
    Serial.print(i + 1);                                         // Prints the motors current i serial
    Serial.print(": ");
    Serial.println(amps[i]);
  }
}

//Plot a variable using serial plotter
void serialPlot() {                                         
  //int currTime = millis();                              // Calculate scan time
  //int scantime = currTime - prevtime;

  //Serial.println(scantime);                             // Print scan time
  //Serial.println(actuators[x].getSetpoint());           // print motor x setpoint
  //Serial.println(actuators[x].getAngle());              // Print motor x angle
  //prevtime = currTime;                                  // Update prev time
}
