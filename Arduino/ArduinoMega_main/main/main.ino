/* Main scketch for our bachelorthesis, group E2103 at NTNU.
   We are controlling 4 brushed Dc motors with encoders connected
   to an I2C bus, witch we get position readings from.
   Running as a rode serial node communicating with an actionserver connected
   to moveit. We are running an independent-joint PD(I) controller.

   Title: Development of an 4 DOF robotic leg.
   Group members: Vegard Hovland, Even vestland, Kristian Grinde, Henrike Moe Arnesen
   Supervisor: Torlef Anstensrud.
*/
//#include <Arduino.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "actuator.h"
#include "DualG2HighPowerMotorShield.h"
#include "variables.h"
#include <ros.h>
#include <std_msgs/Float64.h>                                                                                                                     // Import msgs for subscribing
#include <sensor_msgs/JointState.h>                                                                                                               // Import sesnor msgs for joint states
#include <stdlib.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>

sensor_msgs::JointState robot_state;
std_msgs::UInt16 curr_reading1;
std_msgs::UInt16 curr_reading2;
std_msgs::UInt16 curr_reading3;
std_msgs::UInt16 curr_reading4;

//Class declerations
ros::NodeHandle  nh;

//Adafruit_SSD1306 display_1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);                                                                                // Declaration for display 1
//Adafruit_SSD1306 display_2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);                                                                                // Declaration for display 2

DualG2HighPowerMotorShield24v14 md1(M11nSLEEP, M11DIR, M11PWM,  M11nFAULT,  M11CS, M12nSLEEP,  M12DIR,  M12PWM,  M12nFAULT, M12CS);                // Declaration for Motor driver 1
DualG2HighPowerMotorShield24v14 md2(M21nSLEEP, M21DIR, M21PWM,  M21nFAULT,  M21CS, M22nSLEEP,  M22DIR,  M22PWM,  M22nFAULT, M22CS);                // Declaration for Motor driver 2                                                     //defines the two motor drivers

Actuator actuators[4] = {Actuator(8, 80, 0, 0.005, 3200), Actuator(9, 50, 0, 30, 4480), Actuator(10, 50, 0, 0.30, 4480), Actuator(11, 50, 0 ,30, 4480)};                                                                    // Generates an actuator list contaning 4 actuators and their i2c address

//Declare functions-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void controllActuators(Actuator acts[]);               // Comutes the PID algorithm on every actuator
void printText(Actuator acts[]);                       // Displays data on oled 1
void drawGraph(Actuator acts[]);                       // Draw graph on oled 2
void stopIfFault();                                    // Disable the motordrivers if there is a fault
void setupDrivers();                                   // Start the motor drivers
void shutDown();                                       // Return to start and turn MOSFET off
void updateSetpointSerial();                           // Update setpoint for an actuator using serial input
void updateParametersSerial();                         // Update parameters for an actuator using serial input
void printMenu();                                      // Print menu for switch case
void setupRos();                                       // Initialize ros node
void legCb(const trajectory_msgs::JointTrajectoryPoint& leg);
void rosPub();                                         // Publishes joint states to joint_states topic
void legCb(const std_msgs::Float32MultiArray& leg);    // Calback function for subscriber
void serialPrintData();
void serialPlot();
ros::Publisher pubJoint("/joint_states", &robot_state);                                          // Define  topic for publishing and datatype
ros::Publisher pubCurr1("/current_reading1", &curr_reading1);                                     // Define  topic for publishing and datatype
ros::Publisher pubCurr2("/current_reading2", &curr_reading2);                                     // Define  topic for publishing and datatype
ros::Publisher pubCurr3("/current_reading3", &curr_reading3);                                     // Define  topic for publishing and datatype
ros::Publisher pubCurr4("/current_reading4", &curr_reading4);                                     // Define  topic for publishing and datatype
ros::Subscriber <std_msgs::Float32MultiArray> sub("setpoint2arduino", &legCb);                   // Define callback function and topic/data type for subscribing

void setup() {
  setupRos();
  //Serial.begin(9600);
  Wire.begin();                                  // Initialize wire
  setupDrivers();
  //printMenu();
  delay(10);
}

void loop() {
//  if (Serial.available() > 0) {
//    char ch = Serial.read();                     // Gets user input if there is an input
//    switch (ch) {
//      case 'a': {                                // Updates the setpoint for a given actuator
//          updateSetpointSerial();
//          break;
//        }
//      case 'b': {                                // Updates the PID controller parameters for a given actuator using serial inputs
//          updateParametersSerial();
//          break;
//        }
//      case 'd': {                                // Print all data for actuators
//          serialPrintData();
//          break;
//        }
//      case 'p': {                                // Print menu
//          printMenu();
//          break;
//        }
//      case 'c': {                                // Run calibration cyclus
//          //  calibrationData();
//          break;
//        }
//      case 't': {                                // Run follow trajectory test
//          //followTrajTest();
//          break;
//        }
//      case 'q': {                                // Return to startposition and turn off
//          shutDown();
//          break;
//        }
//
//      default : break;
//    }
//  }
 // stopIfFault();
  controllActuators(actuators);                 // Pid controll on all the acuators by default
  rosPub();                                     // Publish joint states and current readings to ros
  nh.spinOnce();
  //serialPlot();
}


//Function definitions--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Setup function for roscomunication
void setupRos() {
  nh.getHardware()->setBaud(115200);                           // Set baud rate
  nh.initNode();                                               // Initialize serial node
  nh.advertise(pubJoint);                                      // Set publisher function as advertiser
  nh.advertise(pubCurr1);                                       // Set publisher function as advertiser
  nh.advertise(pubCurr2);                                       // Set publisher function as advertiser
  nh.advertise(pubCurr3);                                       // Set publisher function as advertiser
  nh.advertise(pubCurr4);                                       // Set publisher function as advertiser
  nh.subscribe(sub);                                           // Set subscriber for setpoints
  nh.spinOnce();                                               // Sync with ros

 // Fulfill the sensor_msg/JointState msg
  robot_state.name_length = 4;
  robot_state.velocity_length = 4;
  robot_state.position_length = 4;
  robot_state.effort_length = 4;
  robot_state.header.frame_id = "";
  robot_state.name = joint_name;
}


//Print menu for user inputs
void printMenu() {
  Serial.println("Press 'a' to update setpoint");
  Serial.println("Press 'b' to update PID parameters");
  Serial.println("Press 'd' to display information");
  Serial.println("Press 'p' to print menu");
  Serial.println("Press 'c' to run calibration cyclus");
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
  nh.spinOnce();                                                 // Get latest data from subscribed setpoints
  for (int i = 0; i < numActuators; i++) {                       // Loops over the 4 actuator objects
    actuators[i].readAngle();                                    // Get the actuators angle
    actuators[i].computePID();                                   // Computes output using PID
    nh.spinOnce();
    if ( i == 0) {
      md1.setM1Speed(actuators[i].getEffort());                  // Motor 1 is driver 1 M1
      actuators[i].setAmps(md1.getM1CurrentReading());
       curr_reading1.data = actuators[i].getAmps();
    }
    else if ( i == 1) {
      md1.setM2Speed(actuators[i].getEffort());                  // Motor 2 is driver 1 M2
      actuators[i].setAmps(md1.getM2CurrentReading());
      curr_reading2.data = actuators[i].getAmps();
    }
    else  if ( i == 2) {
      md2.setM1Speed(actuators[i].getEffort());                  // Motor 3 is driver 2 M1
      actuators[i].setAmps(md2.getM1CurrentReading());
       curr_reading3.data = actuators[i].getAmps();
    }
    else if ( i == 3) {
      md2.setM2Speed(actuators[i].getEffort());                  // Motor 4 is driver 2 M2
      actuators[i].setAmps(md2.getM2CurrentReading());
       curr_reading4.data = actuators[i].getAmps();
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

// Function for publishing joint states on the joint_state topic
void rosPub() {
  float pos[4];                                                                                                 // Expected size for topic
  float vel[4];
  float eff[4];

  for (int i = 0; i < 4; i++) {                                                                                 // Fulfill the arrays whith motor readings converted to radians
    pos[i] = (float)((actuators[i].getAngle()) / 180.0) * 3.14;                                                 // Calc angle in radians
    vel[i] = actuators[i].getVelocity();                                                                        // Get the actuators velocity in rad/s
    eff[i] = actuators[i].getEffort();                                                                          // Get effort from PID -400 to 400
  }

  // Fulfill the sensor_msg/JointState msg
  robot_state.header.stamp = nh.now();
  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;

  pubJoint.publish( &robot_state);                                                                             // Publish joint states
  pubCurr1.publish( &curr_reading1);                                                                            // Publish current readings
  pubCurr2.publish( &curr_reading2);                                                                            // Publish current readings
  pubCurr3.publish( &curr_reading3);                                                                            // Publish current readings
  pubCurr4.publish( &curr_reading4);                                                                            // Publish current readings
  nh.spinOnce();                                                                                               // Sync with ros
}

void legCb(const std_msgs::Float32MultiArray& leg){
  for (int i = 0; i < 4; i++) {
    float rad = leg.data[i];
    float deg = (rad/3.14)*180.0;
    actuators[i].setSetpoint(deg);   //Get angles from publisher nod
  }
}

//Find offset from pid that is lost through gear ratio 150:1
void calibrationData() {
  for (int i = 0; i < numActuators; i++) {
    actuators[i].setSetpoint(startPos[i]);                      // Updates setpoints to startposition
  }
  int shutdowntime = millis() + 5000;                           //Generate a shutdown time
  int currTime = millis();
  while (shutdowntime > currTime) {                             // Comutes pid for actuators during shutdown time
    controllActuators(actuators);                               // Pid controll on all the acuators
    currTime = millis();
  }

  for (int i = 0 ; i < numActuators; i++) {                     //Get offset speed for calibration calculations, write to monitor
    Serial.print(i);
    Serial.println(" :");
    for (int j = 0; j <= 100; j++) {
      int shutdowntime = millis() + 100;                        //Gives actuator time to reach setpoint
      int currTime = millis();
      actuators[i].setSetpoint(j);
      while (shutdowntime > currTime) {                         // Comutes pid for actuators during shutdown time
        controllActuators(actuators);                           // Pid controll on all the acuators
        currTime = millis();
      }
      Serial.println(actuators[i].getEffort());
    }
  }
}

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

void serialPlot() {                                          //We will tune the parameters using the first motor
  //int currTime = millis();                                 // Calculate scan time
  //int scantime = currTime - prevtime;

  //Serial.println(scantime);
  //Serial.println(actuators[2].getSetpoint());           // print motor 1 setpoint
  //Serial.println(actuators[2].getAngle());              // Print motor 1 angle
  Serial.println("pådrag:");
  for (int i = 0; i < numActuators; i++) {
    Serial.println(actuators[i].getEffort());                 // Print motor 1 speed
  }
  //prevtime = currTime;                                    // Update prev time
}
