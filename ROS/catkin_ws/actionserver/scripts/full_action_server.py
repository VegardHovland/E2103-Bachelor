#! /usr/bin/env python

import rospy

import actionlib

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
        self._action_ns = controller_name + '/follow_joint_trajectory'
        self._as = actionlib.SimpleActionServer(self._action_ns,FollowJointTrajectoryAction,execute_cb=self.execute_cb,auto_start = False)
        self._as.register_goal_callback(self.goalCB)
        self._action_name = rospy.get_name()
        self._as.start()
        self._feedback = FollowJointTrajectoryFeedback
        self._result = FollowJointTrajectoryResult
        rospy.loginfo('Successful init')

    def execute_cb(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points      
        rospy.loginfo(trajectory_points)
        self._goal = self._as.set_succeeded()
    def goalCB(self):
        self._goal = self._as.accept_new_goal()
if __name__ == '__main__':
    rospy.init_node('robotleg_interface')
    server = JointTrajectoryActionServer('robotleg/robotleg_controller')
    rospy.spin()