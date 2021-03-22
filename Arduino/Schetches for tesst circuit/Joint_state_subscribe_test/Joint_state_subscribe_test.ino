#include <Arduino.h>
#include <ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

ros::NodeHandle nh;

float angle[4] = {0,0,0,0};

void legCb(const trajectory_msgs::JointTrajectoryPoint& leg)
{
  angle[0] = leg.positions[0];   //Get angles from Moveit
  angle[1] = leg.positions[1];
  angle[2] = leg.positions[2];
  angle[3] = leg.positions[3];
 }

ros::Subscriber<trajectory_msgs::JointTrajectoryPoint> sub("robotleg/robotleg_controller/follow_joint_trajectory/goal", legCb);

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  
  Serial.println(angle[2],5);
  nh.spinOnce();
  delay(25);
}
