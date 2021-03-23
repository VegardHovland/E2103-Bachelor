#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Float32MultiArray
import time

class pubsub(object):
    def __init__(self):
        rospy.init_node('Serial_helper')
        self.value = Float32MultiArray()
        self.i=0                                                                                   # This should have a better name
        self.value.data = [0, 0, 1.57, 0]
        self.jointStates = [0, 0, 1.57, 0]
        self.pub = rospy.Publisher('/setpoint2arduino', Float32MultiArray, queue_size=1000)
        rospy.Subscriber('/robotleg/robotleg_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, self.update_value)      #define subscriber for trajectory points
        rospy.Subscriber('/joint_states', JointState, self.get_joints)                              # define joint_states subscriber
        rospy.spin()

    def update_value(self, msg):                                                                    # calback function for 
        while self.i < len(msg.goal.trajectory.points):
            for j in range(4):                                                                      # loop over 4 joints
                self.value.data[j] = msg.goal.trajectory.points[self.i].positions[j]                # Update current setpoints     
            rospy.spin                                                                              # Read new value from sensors
            time.sleep(0.01)
            rospy.sleep                                                                             # Sleep for a short duration and continue executing
            self.pub.publish(self.value)                                                            # Publish current setpoints
            if self.tol():                                                                          # check if close enough to setpoint
                self.i = self.i + 1                                                                 # loop to next setpoints in path          
        self.i=0                                                                                    # resent points index when execution is complete

    def get_joints(self, msg):
        self.jointStates = msg.position                                                             # Get joint states
    
    def tol(self):
        tolerance = True
        for indeks in range(4):
         if abs(self.value.data[indeks] - self.jointStates[indeks]) > 0.035:                          # Check if within tolerance curently 2 degrees
            tolerance = False
        return tolerance

if __name__ == '__main__':
    pubsub()
