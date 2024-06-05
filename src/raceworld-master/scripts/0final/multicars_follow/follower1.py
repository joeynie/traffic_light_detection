#! /usr/bin/env python
# -*- coding:UTF-8 -*-
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
import math

# 内参
k = numpy.array([[504.78, 0.0, 320.5], 
              [0.0, 504.78, 240.5],
              [0.0, 0.0, 1.0]])
# 内参矩阵求逆
k_ = numpy.matrix(k).I
Zc = 0.103/2

def stop():
    cmd_vel_pub = rospy.Publisher("cmd_akm2", Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    for i in range(20):
        cmd_vel_pub.publish(twist)
    time.sleep(2)
    rospy.signal_shutdown("~")
    pass

def set_roi_forward(h, w, mask):
    mask[0:215, 0:w] = 0
    mask[215:400, 0:300] = 0
    # mask[400:h, 0:w] = 0
    return mask
    pass

def set_roi_forward2(h, w, mask):
    mask[0:200, 0:w] = 0
    mask[415:h, 0:w] = 0
    mask[200:415, 100:w] = 0
    return mask
    pass

def follow_line(image, color):
    global Zc
    cmd_vel_pub = rospy.Publisher("cmd_akm2", Twist, queue_size=10)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_gray = numpy.array([0, 0, 10])
    upper_gray = numpy.array([0, 0, 100])
    mask = cv2.inRange(hsv, lower_gray, upper_gray)

    h, w = mask.shape
    mask = set_roi_forward(h, w, mask)
    kernel = numpy.ones((29, 29), numpy.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    M1 = cv2.moments(mask)

    lower_gray1 = numpy.array([0, 0, 120])
    upper_gray2 = numpy.array([0, 0, 200])
    mask2 = cv2.inRange(hsv, lower_gray1, upper_gray2)
    kernel = numpy.ones((9,9), numpy.uint8)
    for i in range(1):
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel)
    mask2 = set_roi_forward2(h, w, mask2)
    M2 = cv2.moments(mask2)

    if M1['m00'] > 0:
        cx1 = int(M1['m10'] / M1['m00'])
        cy = int(M1['m01'] / M1['m00'])
        cx2 = 0
        if M2['m00'] > 0:
            cx2 = int(M2['m10'] / M2['m00'])

        b = numpy.array([[cx1], [cy], [1]])
        pose = k_*b
        z_c = Zc/float(pose[1][0])
        print("The distance 1-0 is: ", z_c)

        # cx = cx1 + int(cx2/6)
        cx = cx1
        cv2.circle(image, (cx1, cy), 5, (0, 0, 255), -1)
        # BEGIN CONTROL
        err = cx - w / 2 - 90
        twist = Twist()
        # twist.linear.x = math.sqrt(math.sqrt(z_c))*0.630
        # twist.linear.x = (z_c**0.25)*0.639
        twist.linear.x = (z_c**0.25)*0.399
        # twist.linear.x = (z_c**0.125)*0.615
        if (z_c > 6.0) | (z_c < 0.3):
            twist.linear.x = 0.2
        if(z_c < 0.25):
            stop()
        twist.angular.z = -float(err / 0.83) / 50
        cmd_vel_pub.publish(twist)
    # cv2.imshow("window1", mask)
    # cv2.waitKey(1)
    pass

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    follow_line(frame, "yellow")
    pass

if __name__ == '__main__':
    rospy.init_node("follower1")
    # time.sleep(0.1)
    rospy.Subscriber("/deepracer2/camera/zed_left/image_rect_color_left", Image, image_callback)
    rospy.spin()
    pass
