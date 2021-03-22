#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

ros::NodeHandle nh;
sensor_msgs::JointState robot_state;
ros::Publisher pub("/joint_states", &robot_state);

float angle[4] = {20.4,10.2,5.5,20.0};

void setup()
{ 

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub);
}

void loop()
{
  char robot_id = "robotleg";                                                                                   // Robot namespace for topic
  char *joint_name[4] = {"joint1", "joint2", "joint3", "joint4"};       // Name of joints for topic
  float pos[4];                                                                                                 // Expected size for topic 
  float vel[4];
  float eff[4];

  for (int i = 0; i < 4; i++) {                                                                                 // Fulfill the arrays whith motor readings converted to radians
    pos[i] = 0.9;//(float)((actuators[i].getAngle())/180.0) * 3.14;
    vel[i] = 0.9; //actuators[i].getSpeed();
    eff[i] = 0.9; // Value only for testing
     nh.spinOnce();
  }

  // Fulfill the sensor_msg/JointState msg
  robot_state.name_length = 4;
  robot_state.velocity_length = 4;
  robot_state.position_length = 4;
  robot_state.effort_length = 4;

  robot_state.header.stamp = nh.now();
  robot_state.header.frame_id = "";
  robot_state.name = joint_name;
  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;

  pub.publish( &robot_state);
  nh.spinOnce();
}
