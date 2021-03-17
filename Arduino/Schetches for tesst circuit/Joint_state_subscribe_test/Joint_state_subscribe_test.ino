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

float angle[4] = {0,0,0,0};
float ut;

void leg_cb(const sensor_msgs::JointState& leg)
{
  angle[0] = leg.position[0];
  angle[1] = leg.position[1];
  angle[2] = leg.position[2];
  angle[3] = leg.position[3];
 }

ros::Subscriber<sensor_msgs::JointState> sub("/move_group/pos2arduino", leg_cb);


void setup()
{
  Serial.begin(9600);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  
  Serial.println(angle[2],5);
  nh.spinOnce();
  delay(1);
}
