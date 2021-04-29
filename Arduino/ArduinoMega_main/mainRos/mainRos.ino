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
#include <std_msgs/Float32.h>

sensor_msgs::JointState robot_state;
std_msgs::UInt16 curr_reading1;
std_msgs::UInt16 curr_reading2;
std_msgs::UInt16 curr_reading3;
std_msgs::UInt16 curr_reading4;

std_msgs::Float32 rated_setpoint1;
std_msgs::Float32 rated_setpoint2;
std_msgs::Float32 rated_setpoint3;
std_msgs::Float32 rated_setpoint4;

//Create node handle for ros communication
ros::NodeHandle  nh;

// Init motordriverers as md1(motor driver 1) and md2(motor driver 2)
DualG2HighPowerMotorShield24v14 md1(M11nSLEEP, M11DIR, M11PWM,  M11nFAULT,  M11CS, M12nSLEEP,  M12DIR,  M12PWM,  M12nFAULT, M12CS);                // Declaration for Motor driver 1
DualG2HighPowerMotorShield24v14 md2(M21nSLEEP, M21DIR, M21PWM,  M21nFAULT,  M21CS, M22nSLEEP,  M22DIR,  M22PWM,  M22nFAULT, M22CS);                // Declaration for Motor driver 2

//Object array of the 4 actuators, pid parameters, slaveadress and gear ratio as parameters
Actuator actuators[4] = {Actuator(8, 80, 0, 0.005, 3200), Actuator(9, 70, 0, 30, 4480), Actuator(10, 70, 0, 0.30, 4480), Actuator(11, 50, 0 , 30, 4480)};


//Declare functions-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void controllActuators(Actuator acts[]);               // Comutes the PID algorithm on every actuator
void stopIfFault();                                    // Disable the motordrivers if there is a fault
void setupDrivers();                                   // Start the motor drivers
void setupRos();                                       // Initialize ros comunication
void velCb(const std_msgs::Float32MultiArray& vel);    // callbackfunction for planned joint velocities from moveit
void rosPub();                                         // Publishes joint states to joint_states topic
void legCb(const std_msgs::Float32MultiArray& leg);    // Calback function for planned joint positions from moveit

//Declare ros publishers
ros::Publisher pubJoint("/joint_states", &robot_state);                                           // Robot state publisher on joint_state topic
ros::Publisher pubCurr1("/current_reading1", &curr_reading1);                                     // Publisher for current reading 1 to 4
ros::Publisher pubCurr2("/current_reading2", &curr_reading2);
ros::Publisher pubCurr3("/current_reading3", &curr_reading3);
ros::Publisher pubCurr4("/current_reading4", &curr_reading4);

ros::Publisher pubSetPoint1("/ratedsetpoint1", &rated_setpoint1);                                 // Publisher for rated_setpoint 1 to 4 reccived by pid
ros::Publisher pubSetPoint2("/ratedsetpoint2", &rated_setpoint2);
ros::Publisher pubSetPoint3("/ratedsetpoint3", &rated_setpoint3);
ros::Publisher pubSetPoint4("/ratedsetpoint4", &rated_setpoint4);

//Decalre subscribers
ros::Subscriber <std_msgs::Float32MultiArray> sub("setpoint2arduino", &legCb);                    // subscriber for planned setpoints
ros::Subscriber <std_msgs::Float32MultiArray> sub2("velocities2arduino", &velCb);                 // subscriber for planned velocities


void setup() {
  setupRos();                                        // initialize ros comunication
  Wire.begin();                                      // Initialize wire
  setupDrivers();                                    // Init drivers and startpositions
  delay(10);
}

void loop() {
  // stopIfFault();
  controllActuators(actuators);                 // Pid controll on all the acuators by default
  rosPub();                                     // Publish joint states and current readings to ros
  nh.spinOnce();                                // suync with ros, needs to happen frequently
}

//Setup function for roscomunication
void setupRos() {
  nh.getHardware()->setBaud(115200);                           // Set baud rate
  nh.initNode();                                               // Initialize serial node

  // Set publishers
  nh.advertise(pubJoint);
  nh.advertise(pubCurr1);
  nh.advertise(pubCurr2);
  nh.advertise(pubCurr3);
  nh.advertise(pubCurr4);
  nh.advertise(pubSetPoint1);
  nh.advertise(pubSetPoint2);
  nh.advertise(pubSetPoint3);
  nh.advertise(pubSetPoint4);

  // Set subscribers
  nh.subscribe(sub);
  nh.subscribe(sub2);

  nh.spinOnce();

  // Fulfill sensor_msg/JointState msg
  robot_state.name_length = 4;
  robot_state.velocity_length = 4;
  robot_state.position_length = 4;
  robot_state.effort_length = 4;
  robot_state.header.frame_id = "";
  robot_state.name = joint_name;
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
      rated_setpoint1.data = actuators[i].getRatedSetPoint();
    }
    else if ( i == 1) {
      md1.setM2Speed(actuators[i].getEffort());                  // Motor 2 is driver 1 M2
      actuators[i].setAmps(md1.getM2CurrentReading());
      curr_reading2.data = actuators[i].getAmps();
      rated_setpoint2.data = actuators[i].getRatedSetPoint();
    }
    else  if ( i == 2) {
      md2.setM1Speed(actuators[i].getEffort());                  // Motor 3 is driver 2 M1
      actuators[i].setAmps(md2.getM1CurrentReading());
      curr_reading3.data = actuators[i].getAmps();
      rated_setpoint3.data = actuators[i].getRatedSetPoint();
    }
    else if ( i == 3) {
      md2.setM2Speed(actuators[i].getEffort());                  // Motor 4 is driver 2 M2
      actuators[i].setAmps(md2.getM2CurrentReading());
      curr_reading4.data = actuators[i].getAmps();
      rated_setpoint4.data = actuators[i].getRatedSetPoint();
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
  md1.init();                                                    // Init pinmodes driver 1
  md1.calibrateCurrentOffsets();
  md2.init();                                                    // Init pinmodes driver 2
  md2.calibrateCurrentOffsets();
  md1.enableDrivers();                                           // Enable mosfet 1
  md2.enableDrivers();                                           // Enable mosfet 2
  for (int i = 0; i < numActuators; i++) {
    actuators[i].setSetpoint(startPos[i]);                       //Initialize start setpoints
    actuators[i].setRatedSetpoint(startPos[i]);
  }
  delay(50);                                                     // Enableing needs some time so delay
}

// Function for publishing joint states on the joint_state topic
void rosPub() {
  float pos[4];                                                  // Expected size for topic
  float vel[4];
  float eff[4];

  for (int i = 0; i < 4; i++) {                                    // Fulfill the arrays whith motor readings converted to radians
    pos[i] = (float)((actuators[i].getAngle()) / 57.32);           // store angle in radians
    vel[i] = actuators[i].getVelocity();                           // stpre the actuators velocity in rad/s
    eff[i] = actuators[i].getEffort();                             // store effort from PID -400 to 400
  }

  // Fulfill the sensor_msg/JointState msg
  robot_state.header.stamp = nh.now();
  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;

  pubJoint.publish( &robot_state);                                 // Publish joint states
  pubCurr1.publish( &curr_reading1);                               // Publish current readings 1 to 4
  pubCurr2.publish( &curr_reading2);
  pubCurr3.publish( &curr_reading3);
  pubCurr4.publish( &curr_reading4);

  pubSetPoint1.publish( &rated_setpoint1);                         // Publish rate limited setpoint 1 to 4
  pubSetPoint2.publish( &rated_setpoint2);
  pubSetPoint3.publish( &rated_setpoint3);
  pubSetPoint4.publish( &rated_setpoint4);

  nh.spinOnce();                                                   // Sync with ros
}
// calback for setponts
void legCb(const std_msgs::Float32MultiArray& leg) {
  for (int i = 0; i < 4; i++) {
    float rad = leg.data[i];                                       // read radians planned in moveit
    float deg = rad  * 57.32;                                      // Convert to degrees (180/3.14)
    actuators[i].setSetpoint(deg);                                 // Get angles from publisher nod
  }
}
// calback for velocities
void velCb(const std_msgs::Float32MultiArray& vel) {
  for (int i = 0; i < 4; i++) {
    actuators[i].setDesieredVelocity(vel.data[i]);                 // Set desiered velocity
  }
}
