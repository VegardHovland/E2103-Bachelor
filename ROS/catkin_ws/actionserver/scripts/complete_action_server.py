#! /usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Float32MultiArray
import time

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryGoal
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint
)

class JointTrajectoryActionServer(object):

    def __init__(self, controller_name):
        rospy.init_node('robotleg_interface')
        self._action_ns = controller_name + '/follow_joint_trajectory'
        self._as = actionlib.SimpleActionServer(self._action_ns,FollowJointTrajectoryAction,execute_cb=self.execute_cb,auto_start = False)
        self._as.register_goal_callback(self.goalCB)
        self._action_name = rospy.get_name()
        self._as.start()

        self._feedback = FollowJointTrajectoryFeedback
        self._result = FollowJointTrajectoryResult
        self.value = Float32MultiArray()
        self.i=0                                                                                   # This should have a better name
        self.value.data = [0, 0, 1.57, 0]
        self.jointStates = [0, 0, 1.57, 0]
        rospy.Subscriber('/joint_states', JointState, self.get_joints)                              # define joint_states subscriber
        self.value.data = self.jointStates
        self.pub = rospy.Publisher('/setpoint2arduino', Float32MultiArray, queue_size=1000)
        rospy.loginfo('Successful init')
        rospy.spin()

    def execute_cb(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points 
        self.viapoints = trajectory_points  
        self._goal = self._as.set_succeeded()                                                       # Sucessfully got viapoints 
        while self.i < len(self.viapoints):
            for j in range(4):                                                                      # loop over 4 joints
                self.value.data[j] = self.viapoints[self.i].positions[j]                            # Update current setpoints  
            rospy.spin                                                                              # Read new value from sensors
            time.sleep(0.02)                                                                        # sleep long enough to get new joint_states
            rospy.sleep                                                                             # continue program
            self.pub.publish(self.value)                                                            # Publish current setpoints
            if self.tol():                                                                          # check if close enough to setpoint
                self.i = self.i + 1                                                                 # loop to next setpoints in path                                                         # Set succeeded when done
        self.i=0  

    def goalCB(self):
        self._goal = self._as.accept_new_goal()                                                     

    def get_joints(self, msg):
        self.jointStates = msg.position                                                             # Get joint states
    
    def tol(self):
        tolerance = True
        for indeks in range(4):
         if abs(self.value.data[indeks] - self.jointStates[indeks]) > 0.035:                          # Check if within tolerance curently 2 degrees
            tolerance = False
        return tolerance
        
if __name__ == '__main__':
    JointTrajectoryActionServer('robotleg/robotleg_controller')