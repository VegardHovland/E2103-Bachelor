#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal 
from control_msgs.msg import FollowJointTrajectoryActionGoal

from std_msgs.msg import Float32MultiArray

def subscriber():
    sub = rospy.Subscriber('/robotleg/robotleg_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, callback_function)
    rospy.spin()

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def callback_function(message):
    rospy.loginfo("I received : %f", message.goal.trajectory.points[-1].positions[2])

if __name__ == "__main__":
    rospy.init_node("simple_subscriber")
    subscriber()
