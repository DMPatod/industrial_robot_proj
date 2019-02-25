#!/usr/bin/env python

import math
import numpy
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

def Ztranslation(z_tranlation):
    matrix = [
        [1, 0,  0,  0],
        [0, 1,  0,  0],
        [0, 0,  1,   z_tranlation],
        [0, 0,  0,  1]
    ]
    return matrix


def rotX(theta, z_tranlation):
    rotX = [
        [1, 0,                  0,                  0],
        [0, math.cos(theta),    -math.sin(theta),   0],
        [0, math.sin(theta),    math.cos(theta),    z_tranlation],
        [0, 0,                  0,                  1]
    ]
    return rotX


def rotY(theta, z_tranlation):
    rotX = [
        [math.cos(theta),   0,  math.sin(theta),    0],
        [0,                 1,  0,                  0],
        [-math.sin(theta),  0,  math.cos(theta),    z_tranlation],
        [0,                 0,  0,                  1]
    ]
    return rotX


def rotZ(theta, z_tranlation):
    rotX = [
        [math.cos(theta),    -math.sin(theta),  0,  0],
        [math.sin(theta),    math.cos(theta),   0,  0],
        [0,                  0,                 1,  z_tranlation],
        [0,                  0,                 0,  1]
    ]
    return rotX

link1Height = 0.18
link2Height = 0.365
link3Height = 0.14769

def callback(result):
    rospy.loginfo(rospy.get_caller_id() + "Writing angles value" )
    joint1_pub = rospy.Publisher("/kuka/joint1_position_controller/command", Float64, queue_size=10)
    joint2_pub = rospy.Publisher("/kuka/joint2_position_controller/command", Float64, queue_size=10)
    joint3_pub = rospy.Publisher("/kuka/joint3_position_controller/command", Float64, queue_size=10)

    forward_matrix = numpy.matmul(rotZ(result.data[0], 0), rotY(result.data[1], link1Height))
    forward_matrix = numpy.matmul(forward_matrix, rotY(result.data[2], link2Height))
    forward_matrix = numpy.matmul(forward_matrix, Ztranslation(link3Height))

    if result.data[1] < -2 or result.data[1] > 2:
        result.data[1] = 0
        rospy.loginfo(rospy.get_caller_id() + "Invalid angle value for joint2")
    

    if result.data[2] < -2 or result.data[2] > 2:
        result.data[2] = 0
        rospy.loginfo(rospy.get_caller_id() + "Invalid angle value for joint3")
    
    joint1_pub.publish(result.data[0])
    rospy.loginfo("Move joint1 to " + str(result.data[0]) + "radians")
    joint2_pub.publish(result.data[1])
    rospy.loginfo("Move joint2 to " + str(result.data[1]) + "radians")
    joint3_pub.publish(result.data[2])
    rospy.loginfo("Move joint3 to " + str(result.data[2]) + "radians")

    rospy.loginfo(rospy.get_caller_id() + " final possition:")
    rospy.loginfo("x: " + str(forward_matrix[0][3]))
    rospy.loginfo("Y: " + str(forward_matrix[1][3]))
    rospy.loginfo("Z: " + str(forward_matrix[2][3]))
    

def listener():
    rospy.init_node('fw_Kinematics')

    rospy.loginfo("Forward Kinematics Begin")

    rospy.Subscriber("fw_Kinematics", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()