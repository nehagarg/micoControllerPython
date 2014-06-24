#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from trajectory_msgs.msg import JointTrajectoryPoint
import tf
import copy

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    group = MoveGroupCommander("arm")
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Robot Links for arm:"
    print robot.get_link_names("arm")
    print "============ Robot Links for gripper:"
    print robot.get_link_names("gripper")
    print group.get_end_effector_link()
    print group.get_pose_reference_frame()
    print "============ Printing robot state"
    #print robot.get_current_state()
    print "============"
    tl = tf.TransformListener()
    
    rospy.sleep(1)

    waypoints = []
    
    # start with the current pose
    waypoints.append(group.get_current_pose().pose)
    print waypoints[0]
    currentPose = PoseStamped()
    currentPose.header.frame_id = group.get_pose_reference_frame()
    currentPose.pose = waypoints[0]
    
    currentPoseHandLink = tl.transformPose("Hand_Link", currentPose)
    currentPoseHandLink.pose.position.z= currentPoseHandLink.pose.position.z - 0.1
    
    # first orient gripper and move forward (+x)
    wpose = tl.transformPose(group.get_pose_reference_frame(), currentPoseHandLink).pose
    #wpose = Pose()
    #wpose.orientation.w = 1.0
    #wpose.position.x = waypoints[0].position.x + 0.1
    #wpose.position.y = waypoints[0].position.y
    #wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))
    print waypoints
    # second move down
    #wpose.position.z -= 0.10
    #waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    #wpose.position.y += 0.05
    #waypoints.append(copy.deepcopy(wpose))
    
    (plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0, False)         # jump_threshold

    print "============ Waiting while RVIZ displays plan3..."
    print fraction
    print plan3
    rospy.sleep(5)
    #print "Sending execution command"
    #group.execute(plan3)
    #group.go()
    #rospy.wait_for_service('compute_ik')
    #getJointPosition =  rospy.ServiceProxy('compute_ik', GetPositionIK )
    #service_request = GetPositionIKRequest()
    #service_request.ik_request.group_name = "arm";
    
    #service_request.ik_request.pose_stamped = currentPoseHandLink
    #service_request.ik_request.avoid_collisions = False 
    #serviceResponse = getJointPosition(service_request)
    #print serviceResponse
    
    rospy.spin()
    roscpp_shutdown()
