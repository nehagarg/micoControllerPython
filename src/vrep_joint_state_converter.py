#! /usr/bin/env python

import roslib; roslib.load_manifest('mico_controller_python')
import rospy


from sensor_msgs.msg import (
    JointState
)

import numpy
import copy

def jointStateConverter(vrep_joint_state):
    vrep_joint_names = {"Shoulder_Joint" : "Mico_joint1", "Arm_Joint": "Mico_joint2", "Forearm_Joint" : "Mico_joint3", "Wrist_1_Joint" : "Mico_joint4","Wrist_2_Joint":"Mico_joint5","Hand_Joint":"Mico_joint6"}
    joint_names = list(vrep_joint_state.name)
    position_offset = [3.14, -1.57, 1.57, 3.14, 3.14,3.14]
    output = JointState()
    output.name =["Shoulder_Joint", "Arm_Joint", "Forearm_Joint", "Wrist_1_Joint","Wrist_2_Joint","Hand_Joint"]
    output.position = [0] * len(output.name)
    output.velocity = [0] * len(output.name)
    output.effort = [0] * len(output.name)

    for i in range(0,len(joint_names)):
        if joint_names[i] in vrep_joint_names.values():
            vrep_joint_index = int(joint_names[i][-1]) - 1
            output.position[vrep_joint_index] = vrep_joint_state.position[i] - position_offset[vrep_joint_index]
            output.velocity[vrep_joint_index] = vrep_joint_state.velocity[i]
            output.effort[vrep_joint_index] = vrep_joint_state.effort[i]
    #output.position = list(vrep_joint_state.position[0:6])
    output.position[4] = -1*output.position[4]
    #for i in range(0,6):
        #output.position[i] = 1
     #   output.position[i] = output.position[i] - position_offset[i]
    #output.velocity = vrep_joint_state.velocity[0:6]
    #output.effort = vrep_joint_state.effort[0:6]
    pub1.publish(output)

if __name__ == '__main__':
    rospy.init_node('vrep_joint_state_converter')
    global pub1
    pub1 = rospy.Publisher("/ada_joint_state", JointState, queue_size=10)
    rospy.Subscriber("/vrep/jointState" , JointState, jointStateConverter)
    
    rospy.spin()
