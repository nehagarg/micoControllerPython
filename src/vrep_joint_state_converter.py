#! /usr/bin/env python

import roslib; roslib.load_manifest('mico_controller_python')
import rospy


from sensor_msgs.msg import (
    JointState
)

import numpy
import copy

def jointStateConverter(vrep_joint_state):
    output = JointState()
    output.name =["shoulder_joint", "arm_joint", "forearm_joint", "wrist_1_joint","wrist_2_joint","hand_joint"]
    output.position = vrep_joint_state.position[0:6]
    output.velocity = vrep_joint_state.velocity[0:6]
    output.effort = vrep_joint_state.effort[0:6]
    pub1.publish(output)

if __name__ == '__main__':
    rospy.init_node('vrep_joint_state_converter')
    global pub1
    pub1 = rospy.Publisher("/ada_joint_state", JointState, queue_size=10)
    rospy.Subscriber("/vrep/jointState" , JointState, jointStateConverter)
    
    rospy.spin()
