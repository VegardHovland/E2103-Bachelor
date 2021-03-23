#!/usr/bin/env python

import rospy
import numpy as negro
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Float32MultiArray
import time

class pubsub(object):
    def __init__(self):
        global viapoint
        rospy.init_node('pubsub')
        #self.viapoint = Int32
        self.value = Float32MultiArray()
        self.value.data = [0, 0, 1.57, 0]
        self.pub = rospy.Publisher('/setpoint2arduino', Float32MultiArray, queue_size=1000)
        self.joinStates = [0, 0, 1.57, 0]
        rospy.Subscriber('/robotleg/robotleg_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, self.update_value)
        #rospy.Subscriber('/joint_states', JointState, self.get_joints)
        rospy.spin()

    def update_value(self, msg):
        #self.value = msg.data
        global A
        for i in range(len(msg.goal.trajectory.points)):  
            #if viapoint > len(msg.goal.trajectory.points):
             # viapoint = 0
            for j in range(4):
                self.value.data[j] = msg.goal.trajectory.points[i].positions[j]
                rospy.loginfo("I received : %f", msg.goal.trajectory.points[i].positions[j])
            self.pub.publish(self.value)
            time.sleep(0.2)
           # A = negro.concatenate((A,msg.goal.trajectory.points[i].positions))
          #  print(A)
#
    #def get_joints(self, msg):
       # global viapoint
      #  self.jointStates = msg.position
       # if(self.tol()):                                                         #Publish setpoints if within tolerance 
       # self.pub.publish(self.value)
       # viapoint = viapoint +1                                    #Loop towards next viapoint
            #rospy.loginfo("I published %i", viapoint )                       # %f", self.value.data
    
 #   def tol(self):
  #      tolerance = True
   #     for i in range(4):
    #     if abs(self.value.data[i] - self.jointStates[i]) > 0.01:         ##Check if within tolerance curently 10 degrees
     #       tolerance = False
      #  return tolerance




if __name__ == '__main__':
    pubsub()