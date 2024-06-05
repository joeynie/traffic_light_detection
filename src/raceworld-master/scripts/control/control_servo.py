#! /usr/bin/env python
# -*- coding:UTF-8 -*-
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from math import *

def callback1(msg):
    pub = rospy.Publisher("/deepracer1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

    akm = AckermannDriveStamped()

    if msg.linear.x != 0:
        akm.drive.speed = msg.linear.x*1.80
        akm.drive.steering_angle = atan((msg.angular.z/msg.linear.x)*0.133)
    else:
        akm.drive.speed = 0
        akm.drive.steering_angle = 0

    pub.publish(akm)
    pass

def callback2(msg):
    pub = rospy.Publisher("/deepracer2/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

    akm = AckermannDriveStamped()

    if msg.linear.x != 0:
        akm.drive.speed = msg.linear.x*1.80
        akm.drive.steering_angle = atan((msg.angular.z/msg.linear.x)*0.133)
    else:
        akm.drive.speed = 0
        akm.drive.steering_angle = 0

    pub.publish(akm)
    pass

def callback3(msg):
    pub = rospy.Publisher("/deepracer3/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

    akm = AckermannDriveStamped()

    if msg.linear.x != 0:
        akm.drive.speed = msg.linear.x*1.80
        akm.drive.steering_angle = atan((msg.angular.z/msg.linear.x)*0.133)
    else:
        akm.drive.speed = 0
        akm.drive.steering_angle = 0

    pub.publish(akm)
    pass

def callback4(msg):
    pub = rospy.Publisher("/deepracer4/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

    akm = AckermannDriveStamped()

    if msg.linear.x != 0:
        akm.drive.speed = msg.linear.x*1.80
        akm.drive.steering_angle = atan((msg.angular.z/msg.linear.x)*0.133)
    else:
        akm.drive.speed = 0
        akm.drive.steering_angle = 0

    pub.publish(akm)
    pass

def cmd_to_akm():
    rospy.init_node('control_servo', anonymous=True)
    rospy.Subscriber("cmd_akm1", Twist, callback1)
    rospy.Subscriber("cmd_akm2", Twist, callback2)
    rospy.Subscriber("cmd_akm3", Twist, callback3)
    rospy.Subscriber("cmd_akm4", Twist, callback4)
    rospy.spin()
    pass

if __name__ =='__main__':
    cmd_to_akm()
    pass
