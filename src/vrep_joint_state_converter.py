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
    output.name =["Shoulder_Joint", "Arm_Joint", "Forearm_Joint", "Wrist_1_Joint","Wrist_2_Joint","Hand_Joint"]
    output.position = list(vrep_joint_state.position[0:6])
    position_offset = [3.14, -1.57, 1.57, 3.14, 0,3.14]
    for i in range(0,6):
        #output.position[i] = 1
        output.position[i] = output.position[i] - position_offset[i]
    output.velocity = vrep_joint_state.velocity[0:6]
    output.effort = vrep_joint_state.effort[0:6]
    pub1.publish(output)

if __name__ == '__main__':
    rospy.init_node('vrep_joint_state_converter')
    global pub1
    pub1 = rospy.Publisher("/ada_joint_state", JointState, queue_size=10)
    rospy.Subscriber("/vrep/jointState" , JointState, jointStateConverter)
    
    rospy.spin()
