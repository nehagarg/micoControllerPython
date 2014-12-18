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
    JointTrajectory
)

from sensor_msgs.msg import (
    JointState
)

from geometry_msgs.msg import (
    PoseStamped
)
from std_msgs.msg       import Float64
import copy
class FollowJointTrajectoryActionServer:
    def __init__(self):
       

        self.server = actionlib.SimpleActionServer('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, self.execute, False)
        self.server.start()
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()
        self._control_rate = 10.0
        self.jointStatePublisher = rospy.Publisher('trajectory_joint_state', JointState, queue_size=10)
        self.jointStatePublisherForVrep = []
        for i in range(1,7):
            self.jointStatePublisherForVrep.append(rospy.Publisher('trajectory_points_joint' + repr(i), Float64, queue_size=10) )
        self.vrep_joint_names = {"Shoulder_Joint" : "Mico_joint1", "Arm_Joint": "Mico_joint2", "Forearm_Joint" : "Mico_joint3", "Wrist_1_Joint" : "Mico_joint4","Wrist_2_Joint":"Mico_joint5","Hand_Joint":"Mico_joint6"}
        self.vrep_position_offset =  [3.14, -1.57, 1.57, 3.14, 0,3.14]
        #self.jointTrajectoryPublisher=rospy.Publisher('trajectory_joint_velocity',JointTrajectory,queue_size=10)
    
    def execute(self, goal):
        #pickle.dump(goal, open("trajecoryGoalMsg3.data", "wb"))
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        num_points = len(trajectory_points)
        rospy.loginfo("Received %i trajectory points" % num_points)
        pprint(goal.trajectory)

        end_time = trajectory_points[-1].time_from_start.to_sec()
        control_rate = rospy.Rate(self._control_rate)
        i=num_points
        rospy.loginfo("the num of point is %i" % i )
        #self.jointTrajectoryPublisher.publish(goal.trajectory)

        for trajectory_point in trajectory_points:
            #Publish for vrepp
            jsVrep = JointState()
            jsVrep.name = copy.deepcopy(joint_names)
            jsVrep.position = trajectory_point.positions
            for j in range(len(joint_names)):
                jsVrep.name[j] = self.vrep_joint_names[jsVrep.name[j]]
                vrep_joint_index = int(jsVrep.name[j][-1]) - 1
                self.jointStatePublisherForVrep[vrep_joint_index].publish(jsVrep.position[j] + self.vrep_position_offset[vrep_joint_index])

            #Publisher for real robot
            js = JointState()
            #rospy.loginfo("the length of js.name is %i" %len(js.name))
            js.name=copy.deepcopy(joint_names)
            for j in range(len(joint_names)):
                #rospy.loginfo("j is %i" % j)
                js.name[j] = js.name[j]+'_'+str(i)
            i=i-1
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
    server.executeFromFile("../trajecoryGoalMsg.data")
    rospy.spin()
