#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal 
from control_msgs.msg import FollowJointTrajectoryActionGoal

from std_msgs.msg import Float32MultiArray

setpoint = [0, 0, 1.57, 0]
pub = rospy.Publisher('/setpoint2arduino', Float32MultiArray, queue_size=1000)

def subscriber():
    sub = rospy.Subscriber('/robotleg/robotleg_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, callback_function)
    rospy.spin()

def setpointToArduino():
    pub = rospy.Publisher('/setpoint2arduino', Float32MultiArray, queue_size=1000)
    rospy.loginfo(setpoint)
    pub.publish(setpoint)

def callback_function(message):
    global setpoint
    global pub
    for i in range(4):
        setpoint[i] = message.goal.trajectory.points[-1].positions[i]
        rospy.loginfo("I received : %f", message.goal.trajectory.points[-1].positions[i])
    pub.publish(setpoint)
    setpointToArduino()


if __name__ == "__main__":
    rospy.init_node("simple_subscriber")
    subscriber()
    rospy.spin()
