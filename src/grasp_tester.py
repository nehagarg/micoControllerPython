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

            
def poseCallback(data, subscriber_no):
    print str(subscriber_no)
    print data
    #arm.set_pose_target(data)
    try:
        #object_pose = data
        (r, p, y) = tf.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        p = p + 1.57
        q = tf.transformations.quaternion_from_euler(r, p, y)
        data.pose.orientation.x = q[0]
        data.pose.orientation.y = q[1]
        data.pose.orientation.z = q[2]
        data.pose.orientation.w = q[3]
        
        arm.set_joint_value_target(data, False)
        arm.set_planner_id("RRTConnectkConfigDefault")
        print "======= Waiting while setting joint value target"
        
        #hack
        #object_pose.pose.position.x = -0.26
        #scene.add_box("part", object_pose, (0.05, 0.05, 0.1))
        rospy.sleep(5)
        plan1 = arm.plan()

        print "============ Waiting while RVIZ displays plan1..."
        rospy.sleep(5)
    

        print "============ Visualizing plan1"
        display_trajectory = DisplayTrajectory()

        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        #display_trajectory_publisher.publish(display_trajectory);

        print "============ Waiting while plan1 is visualized (again)..."
        #rospy.sleep(5)
        a = raw_input("Shall i execute grasp" + str(subscriber_no) + "? Say y/n ")
        
        if a == "y":
            print "executing"
            arm.go(wait=True)
        if a == "n":
            print "not executing"
    except:
        print "Exception thrown while path planning for grasp " + str(subscriber_no)
    
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
    i = 0
    #for i in range(0,200):
        #a = raw_input("Shall i subscribe to grasp" + str(i) + "? Say y/n ")
        #if a == "y":
    rospy.Subscriber("pick_and_place_app/grasp_pose_" + str(i), PoseStamped, poseCallback, i)


    #server.executeFromFile("trajecoryGoalMsg.data")
    rospy.spin()
