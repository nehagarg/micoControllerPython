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
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
import tf

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    right_arm = MoveGroupCommander("arm")
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Robot Links for arm:"
    print robot.get_link_names("arm")
    print "============ Robot Links for gripper:"
    print robot.get_link_names("gripper")
    print right_arm.get_end_effector_link()
    print "============ Printing robot state"
    #print robot.get_current_state()
    print "============"

    
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("pole")
    scene.remove_world_object("table")
    #scene.remove_world_object("part")
    scene.remove_world_object("grasp_marker")
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    print p.header.frame_id
    p.pose.position.x = 0.4
    p.pose.position.y = -0.4
    p.pose.position.z = 0.85
    p.pose.orientation.w = 1.0
    #scene.add_box("pole", p, (0.3, 0.1, 1.0))

    p.pose.position.y = -0.2
    p.pose.position.z = 0.175
    #scene.add_box("table", p, (0.5, 1.5, 0.35))

    p.pose.position.x = 0.4
    p.pose.position.y = 0.0
    p.pose.position.z = 0.5
    #scene.add_box("part", p, (0.15, 0.1, 0.3))

    rospy.sleep(10)
    grasps = []
   
    g = Grasp()
    g.id = "test"
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "base_link"

    grasp_pose.pose.position.x = -0.312728
    grasp_pose.pose.position.y = -0.3754694
    grasp_pose.pose.position.z =  0.0149573836412
    grasp_pose.pose.orientation.x = 0.76326547256
    grasp_pose.pose.orientation.y = -0.518249174456
    grasp_pose.pose.orientation.z = -0.313073506604
    grasp_pose.pose.orientation.w = -0.225451970573
    
    #object_pose = grasp_pose
    #object_pose.pose.position.x = -0.26
    #scene.add_box("part", object_pose, (0.05, 0.05, 0.1))
#    grasp_pose.pose.orientation.w = 1
    #right_arm.set_end_effector_link("Hand_Link")
    #right_arm.set_pose_target(grasp_pose)
    #scene.remove_world_object("grasp_marker")
    #scene.add_box("grasp_marker",grasp_pose, (0.05, 0.05, 0.05)) 
    #right_arm.go()
    pub = rospy.Publisher("pick_and_place_app/grasp_pose_0", PoseStamped, queue_size=10)
    
    #pub1 = rospy.Publisher("transformed/grasp_pose_0", PoseStamped, queue_size=10)

    
    rospy.sleep(2)
    #data = grasp_pose
    #(r, p, y) = tf.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    #p = p + 1.57
    #q = tf.transformations.quaternion_from_euler(r, p, y)
    #data.pose.orientation.x = q[0]
    #data.pose.orientation.y = q[1]
    #data.pose.orientation.z = q[2]
    #data.pose.orientation.w = q[3]
    
    pub.publish(grasp_pose)
    #pub1.publish(data)
    # pick an object
    #robot.arm.pick("part")

    rospy.spin()
    roscpp_shutdown()
