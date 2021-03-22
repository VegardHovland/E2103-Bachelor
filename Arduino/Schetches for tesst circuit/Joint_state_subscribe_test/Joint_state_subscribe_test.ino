#include <Arduino.h>
#include <ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>  
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

ros::NodeHandle nh;

float angle[4] = {0,0,0,0};

void legCb(const control_msgs::FollowJointTrajectoryActionGoal& leg)
{
  angle[0] = leg.goal.trajectory.points[0].positions[0];   //Get angles from Moveit
  angle[1] = leg.goal.trajectory.points[0].positions[0];
  angle[2] = leg.goal.trajectory.points[0].positions[0];
  angle[3] = leg.goal.trajectory.points[0].positions[0];
 }

ros::Subscriber<control_msgs::FollowJointTrajectoryActionGoal> sub("robotleg/robotleg_controller/follow_joint_trajectory/goal",1000, legCb);

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
