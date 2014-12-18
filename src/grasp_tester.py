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
import tf
import traceback

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

from std_msgs.msg import String
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from sensor_msgs.msg import (
    JointState
)

from geometry_msgs.msg import (
    PoseStamped
)

from pose_msgs.msg import *


import numpy
import copy

###import service type
from checkmotionfinished.srv import Ismotionfinished
            
def IsMotionFinished(enquiry):
    rospy.wait_for_service('IsMotionFinished')
    try:
        service_req=rospy.ServiceProxy('IsMotionFinished',Ismotionfinished)
        resp=service_req(enquiry)
        return resp.finished
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def poseCallback(Pose_Array):
    
    print type(Pose_Array)
    posearray=Pose_Array.posearray
    print (len(posearray))
    i=80
    for pose in posearray[80:]:
        i=i+1
        data=pose.pose
        #print (type(data))
        print (data)

        try:
            dataCopy1 = copy.deepcopy(data)
            (r, p, y) = tf.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])      
            tMatrix = tf.transformations.euler_matrix(r, p, y)
            x_offset=-0.10


            data.pose.position.x=data.pose.position.x+(tMatrix[0,0]*x_offset)
            data.pose.position.y=data.pose.position.y + (tMatrix[1,0]*x_offset)
            data.pose.position.z=data.pose.position.z + (tMatrix[2,0]*x_offset)
            
            newRotMatrix = tf.transformations.euler_matrix(0, 1.57, 0)
            transformedMatrix = numpy.dot(tMatrix, newRotMatrix)

            dataCopy1.pose.position.x=dataCopy1.pose.position.x - (transformedMatrix[0,2]*x_offset)
            dataCopy1.pose.position.y=dataCopy1.pose.position.y - (transformedMatrix[1,2]*x_offset)
            dataCopy1.pose.position.z=dataCopy1.pose.position.z - (transformedMatrix[2,2]*x_offset)
           
            dis = (dataCopy1.pose.position.x - data.pose.position.x) * (dataCopy1.pose.position.x - data.pose.position.x)
            dis = dis + (dataCopy1.pose.position.y - data.pose.position.y) * (dataCopy1.pose.position.y - data.pose.position.y)
            dis = dis + (dataCopy1.pose.position.z - data.pose.position.z) * (dataCopy1.pose.position.z - data.pose.position.z)
            if (dis < 0.008):
                pass
            else:
                newRotMatrix = tf.transformations.euler_matrix(0, -1.57, 0)
                transformedMatrix = numpy.dot(tMatrix, newRotMatrix)
        
            q = tf.transformations.quaternion_from_matrix(transformedMatrix)
            data.pose.orientation.x = q[0]
            data.pose.orientation.y = q[1]
            data.pose.orientation.z = q[2]
            data.pose.orientation.w = q[3]
            pub1.publish(data)
            #rospy.sleep(5)
            arm.set_joint_value_target(data, False)
            arm.set_planner_id("RRTConnectkConfigDefault")
            print "======= Waiting while setting joint value target"     
            
           
            
            #rospy.sleep(5)
            #modifed from here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # find who throw motion planer failures
            try:
                plan1 = arm.plan()
            except Exception, e:
                print "Exception thrown while path planning for grasp " + str(i)
                print traceback.format_exc()
                continue
            

            print "============ Waiting while RVIZ displays plan1..."
            #rospy.sleep(5)        
    
            a = raw_input("Shall i execute grasp" + str(i) + "? Say y/n ")
            
            if a == "y":
                print "executing"
                
                arm.execute(plan1)
                #########################
                ## Wait for Plan1 to finish
                #########################
                while(not IsMotionFinished(True)):
                    print "Plan1 is being executed!"
                print "Plan1 has finished!!"
                #b = raw_input("Shall i plan ik path" + str(i) + "? Say y/n ")
                #if b == 'y' :
                rospy.sleep(1)
                print "============ Computing cartesian path..."
                waypoints = []
                waypoints.append(data.pose)
                data.header.stamp = rospy.Time(0)
                dataHandLink = tl.transformPose("Hand_Link", data)
                dataHandLink.pose.position.z= dataHandLink.pose.position.z + x_offset
                dataHandLink.header.stamp = rospy.Time(0)
                wpose = tl.transformPose(arm.get_pose_reference_frame(), dataHandLink).pose
                waypoints.append(copy.deepcopy(wpose))
                print wpose
                (plan2, fraction) = arm.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.01,        # eef_step
                                 0.0, False)         # jump_threshold
                
                print fraction
                print "============ Visualizing plan1 and plan2"
                display_trajectory = DisplayTrajectory()
                display_trajectory.trajectory_start = robot.get_current_state()
                display_trajectory.trajectory.append(plan1)
                display_trajectory.trajectory.append(plan2)
            
                display_trajectory_publisher.publish(display_trajectory);
                    #c = raw_input("Shall i execute ik path" + str(i) + "? Say y/n ")
                    #if c == 'y' :
                        #print "executing"
                arm.execute(plan2)
                #########################
                ## Wait for Plan2 to finish
                #########################
                while(not IsMotionFinished(True)):
                    print "Plan2 is being executed!"
                print "Plan2 has finished!!"
                        #d=raw_input("Shall I close the gripper? Say y/n ")
                        #if d=='y':
                           # print "closing the gripper!"
                r = rospy.Rate(10) # 10hz
                while not rospy.is_shutdown():
                    strr = "close"
                    pub_gripper.publish(strr)
                    r.sleep()
                                
                    break
                   
            if a == "n":
                print "not executing"
                continue
        except Exception, err:
            print "NO IK solution for grasp  " + str(i)
            print traceback.format_exc()
            continue
        print "Oh Yeah! Grasp execution is finished!"
        break
     

# def next_pose(i):
#     print 'hello'
#     rospy.Subscriber("pick_and_place_app/grasp_pose_" + str(i), PoseStamped, poseCallback, i)

if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('grasp_tester')
    
    robot = RobotCommander()
    global arm
    global scene
    scene = PlanningSceneInterface()
    arm = MoveGroupCommander("arm")
    rospy.sleep(1)
    #scene.remove_world_object("part")
    global display_trajectory_publisher
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    DisplayTrajectory)
    print "hereeeeeeeeeeeeeeeeeeee"
    global tl
    tl = tf.TransformListener()
    global pub1
    pub1 = rospy.Publisher("transformed/grasp_pose", PoseStamped, queue_size=10)
    pub_gripper = rospy.Publisher('/gripper_state', String, queue_size=10,latch=False)
    i = 0
    #for i in range(0,200):
        #a = raw_input("Shall i subscribe to grasp" + str(i) + "? Say y/n ")
        #if a == "y":
    #rospy.Subscriber("pick_and_place_app/grasp_pose_" + str(i), PoseStamped, poseCallback, i)
    rospy.Subscriber("test_pose" , Marker_Pose_Array, poseCallback)
    #rospy.Subscriber("Finished" , char, finished)

    #server.executeFromFile("trajecoryGoalMsg.data")
    rospy.spin()
