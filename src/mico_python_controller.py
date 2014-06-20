#! /usr/bin/env python

import roslib; roslib.load_manifest('mico_controller_python')
import rospy
import actionlib
from pprint import pprint
import pickle
import bisect
import sys
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from sensor_msgs.msg import (
    JointState
)

from geometry_msgs.msg import (
    PoseStamped
)

class FollowJointTrajectoryActionServer:
    def __init__(self):
       

        self.server = actionlib.SimpleActionServer('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, self.execute, False)
        self.server.start()
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()
        self._control_rate = 10.0
        self.jointStatePublisher = rospy.Publisher('trajectory_joint_state', JointState, queue_size=10)
    
    def execute(self, goal):
        #pickle.dump(goal, open("trajecoryGoalMsg.data", "wb"))
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        num_points = len(trajectory_points)
        rospy.loginfo("Received %i trajectory points" % num_points)
        pprint(goal.trajectory)

        end_time = trajectory_points[-1].time_from_start.to_sec()
        control_rate = rospy.Rate(self._control_rate)
        for trajectory_point in trajectory_points:
            js = JointState()
            js.name = joint_names
            js.position = trajectory_point.positions
            self.jointStatePublisher.publish(js)
            control_rate.sleep()
            
        start_time = rospy.get_time()
        now_from_start = rospy.get_time() - start_time
        
        last_idx = -1
        self._result.error_code = self._result.SUCCESSFUL
        self.server.set_succeeded(self._result)



    def executeFromFile(self, filename):
        goal = pickle.load( open( filename, "rb" ) )
        self.execute(goal)
            


if __name__ == '__main__':
   
    rospy.init_node('mico_controller_server')
    server = FollowJointTrajectoryActionServer()
    #server.executeFromFile("trajecoryGoalMsg.data")
    rospy.spin()
