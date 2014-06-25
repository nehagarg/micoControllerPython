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

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from sensor_msgs.msg import (
    JointState
)

from geometry_msgs.msg import (
    PoseStamped
)
import numpy
import copy
            
def poseCallback(data, subscriber_no):
    print str(subscriber_no)
    print data
    #arm.set_pose_target(data)
    try:
        dataCopy1 = copy.deepcopy(data)
        
        
        #object_pose = data
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
            #p=p - 1.57
            pass
        else:
            newRotMatrix = tf.transformations.euler_matrix(0, -1.57, 0)
            transformedMatrix = numpy.dot(tMatrix, newRotMatrix)
            #p = p  + 1.57
        # if y > 0:
        #     p = p + 1.57
        # else:
        #     p = p -1.57
        
        print dis
        
        q = tf.transformations.quaternion_from_matrix(transformedMatrix)
        #q = tf.transformations.quaternion_from_euler(r, p, y)
        data.pose.orientation.x = q[0]
        data.pose.orientation.y = q[1]
        data.pose.orientation.z = q[2]
        data.pose.orientation.w = q[3]
        pub1.publish(data)
        rospy.sleep(5)
        arm.set_joint_value_target(data, False)
        arm.set_planner_id("RRTConnectkConfigDefault")
        print "======= Waiting while setting joint value target"
        #pub1 = rospy.Publisher("transformed/grasp_pose", PoseStamped, queue_size=10)
        #pub1.publish(data)
        
        
        waypoints = []
        waypoints.append(data.pose)
        data.header.stamp = rospy.Time(0)
        #tl.waitForTransform(arm.get_pose_reference_frame(), "Hand_Link", data.header.stamp, rospy.Duration(3.0))
        dataHandLink = tl.transformPose("Hand_Link", data)
        dataHandLink.pose.position.z= dataHandLink.pose.position.z + x_offset
        dataHandLink.header.stamp = rospy.Time(0)
        #tl.waitForTransform("Hand_Link", arm.get_pose_reference_frame(), dataHandLink.header.stamp, rospy.Duration(3.0))
        wpose = tl.transformPose(arm.get_pose_reference_frame(), dataHandLink).pose
        waypoints.append(copy.deepcopy(wpose))
        
        rospy.sleep(5)
        plan1 = arm.plan()

        print "============ Waiting while RVIZ displays plan1..."
        rospy.sleep(5)
        

        

        #print "============ Waiting while plan1 is visualized (again)..."
        #rospy.sleep(5)
        a = raw_input("Shall i execute grasp" + str(subscriber_no) + "? Say y/n ")
        
        if a == "y":
            print "executing"
            #arm.go(wait=True)
            
            arm.execute(plan1)
            rospy.sleep(5)
            b = raw_input("Shall i plan ik path" + str(subscriber_no) + "? Say y/n ")
            if b == 'y' :
                print "============ Computing cartesian path..."
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
                c = raw_input("Shall i execute ik path" + str(subscriber_no) + "? Say y/n ")
                if c == 'y' :
                    print "executing"
                    arm.execute(plan2)
                    #rospy.sleep(5)
        if a == "n":
            print "not executing"
    except Exception, err:
        print "Exception thrown while path planning for grasp " + str(subscriber_no)
        print traceback.format_exc()
    
    i = subscriber_no + 1
    rospy.Subscriber("pick_and_place_app/grasp_pose_" + str(i), PoseStamped, poseCallback, i)

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
    i = 0
    #for i in range(0,200):
        #a = raw_input("Shall i subscribe to grasp" + str(i) + "? Say y/n ")
        #if a == "y":
    rospy.Subscriber("pick_and_place_app/grasp_pose_" + str(i), PoseStamped, poseCallback, i)


    #server.executeFromFile("trajecoryGoalMsg.data")
    rospy.spin()
