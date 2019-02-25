#!/usr/bin/env python

import math
import numpy
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

link1Height = 0.18
link2Height = 0.365
link3Height = 0.14769

def callback(result):
    rospy.loginfo(rospy.get_caller_id() + "Moving to possition")
    joint1_pub = rospy.Publisher("/kuka/joint1_position_controller/command", Float64, queue_size=10)
    joint2_pub = rospy.Publisher("/kuka/joint2_position_controller/command", Float64, queue_size=10)
    joint3_pub = rospy.Publisher("/kuka/joint3_position_controller/command", Float64, queue_size=10)

    x = result.data[0]
    y = result.data[1]
    z = result.data[2]

    r = math.sqrt(x**2 + y**2)
    a = math.sqrt((z - link1Height)**2 + r**2)
    try:
        temp1 = (a**2 - link2Height**2 - link3Height**2)/(-2*link2Height*link3Height)
    except ZeroDivisionError:
        temp1 = 0
    try:
        temp2 = (z - link1Height)/r
    except ZeroDivisionError:
        temp2 = 0
    phi1 = math.acos(temp1 - int(temp1))
    phi2 = math.atan(temp2 - int(temp2))

    try:
        theta1 = math.atan(y/x)
    except ZeroDivisionError:
        theta1 = 0
    theta2 = (math.pi/2) - phi2
    theta3 = math.pi - phi1

    joint1_pub.publish(theta1)
    rospy.loginfo("Move joint1 to " + str(theta1) + "radians")
    joint2_pub.publish(theta2)
    rospy.loginfo("Move joint1 to " + str(theta2) + "radians")
    joint3_pub.publish(theta3)
    rospy.loginfo("Move joint3 to " + str(theta3) + "radians")

def listener():
    rospy.init_node('iv_Kinematics')

    rospy.Subscriber("iv_Kinematics", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()