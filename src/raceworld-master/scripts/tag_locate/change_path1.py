#! /usr/bin/env python
# -*- coding:UTF-8 -*-
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import pupil_apriltags as apriltag


'''
可以调的参数
1.两侧的检测范围
2.hsv取色的范围
3.cx的大小以及计算时取的比例
4.角速度

可用参数搭配：
0.6 0.88
'''

path = 1
id = 20
pre_id = 20

def change_path():
    global path
    if path == 0:
        path = 1
    else:
        path = 0

def get_tag(frame):
    global id
    detector = apriltag.Detector()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    if tags == []:
        return False
    else:
        for tag in tags:
            if(tag.tag_id == 2):
                return False
            if(tag.tag_id == 0):
                return False
            id = tag.tag_id
        return True
    pass

def set_roi_forward1(h, w, mask):
    search_top = int(1 * h / 2)
    search_bot = search_top + 180
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask
    pass

def set_roi_forward2(h, w, mask):
    mask[0:200, 0:w] = 0
    mask[415:h, 0:w] = 0
    mask[200:415, 100:w] = 0
    return mask
    pass

def follow_line_in(image):
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)
    kernel = numpy.ones((7,7), numpy.uint8)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([26, 43, 46])
    lower_yellow = numpy.array([26, 43, 0])
    upper_yellow = numpy.array([34, 255, 255])
    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w = mask1.shape
    mask1 = cv2.dilate(mask1, kernel, 2)
    mask1 = set_roi_forward1(h, w, mask1)

    lower_gray = numpy.array([0, 0, 120])
    upper_gray = numpy.array([0, 0, 200])
    mask2 = cv2.inRange(hsv, lower_gray, upper_gray)
    mask2 = cv2.dilate(mask2, kernel, 2)
    mask2 = set_roi_forward2(h, w, mask2)

    # print(h,w)
 
    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
    if M1['m00'] == 0:
        cx1 = 630
        if M2['m00'] > 0:
            cx2 = int(M2['m10'] / M2['m00'])
        elif M2['m00'] == 0:
            # cx2 = 0
            cx2 = 0
        cy = 0
        pass

    elif M1['m00'] > 0:
        cx1 = int(M1['m10'] / M1['m00'])
        if M2['m00'] > 0:
            cx2 = int(M2['m10'] / M2['m00'])
        elif M2['m00'] == 0:
            # cx2 = 0
            cx2 = 0
        cy = int(M1['m01'] / M1['m00'])

    cx = int((cx1-cx2)*47/64+cx2)

    cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
    err = cx - w / 2 - 60
    twist = Twist()
    twist.linear.x = 0.6
    twist.angular.z = -float(err / 0.9) / 50
    cmd_vel_pub.publish(twist)
    # cv2.imshow("window1", mask2+mask1)
    cv2.imshow("window1", image)
    cv2.waitKey(1)
    pass

def set_roi_forward3(h, w, mask):
    mask[0:200, 0:w] = 0
    mask[200:360, 0:540] = 0
    mask[360:480, 0:w] = 0
    return mask
    pass
def follow_line_out(image):
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)
    kernel = numpy.ones((7, 7), numpy.uint8)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([26, 43, 0])
    upper_yellow = numpy.array([34, 255, 255])
    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w = mask1.shape
    mask1 = set_roi_forward1(h, w, mask1)
    mask1 = cv2.dilate(mask1, kernel, iterations=2)
    

    lower_gray = numpy.array([0, 0, 120])
    upper_gray = numpy.array([0, 0, 200])
    mask2 = cv2.inRange(hsv, lower_gray, upper_gray)
    kernel = numpy.ones((9,9), numpy.uint8)
    mask2 = set_roi_forward3(h, w, mask2)
    mask2 = cv2.dilate(mask2, kernel, iterations=2)

    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
    if M1['m00'] == 0:
        cx1 = 30
        if M2['m00'] > 0:
            cx2 = int(M2['m10'] / M2['m00'])
        elif M2['m00'] == 0:
            # cx2 = 0
            cx2 = 630
        cy = 0
        pass

    elif M1['m00'] > 0:
        cx1 = int(M1['m10'] / M1['m00'])
        if M2['m00'] > 0:
            cx2 = int(M2['m10'] / M2['m00'])
        elif M2['m00'] == 0:
            # cx2 = 0
            cx2 = 630
        cy = int(M1['m01'] / M1['m00'])

    cx = int((cx2-cx1)*32/64+cx1)

    cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
    err = cx - w / 2 - 57
    twist = Twist()
    twist.linear.x = 0.6
    twist.angular.z = -float(err / 1.4) / 50
    cmd_vel_pub.publish(twist)
    cv2.imshow("window1", mask2+mask1)
    cv2.waitKey(1)
    # cv2.imshow("window1", image)
    # cv2.waitKey(1)
    pass

def follow_line(frame):
    global path
    if(path == 0):
        follow_line_in(frame)
    else:
        follow_line_out(frame)
    pass

def image_callback(msg):
    global path
    global id
    global pre_id
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    if get_tag(frame):
        if id != pre_id:
            pre_id = id
            change_path()
        else:
            pass
    follow_line(frame)
    pass

if __name__ == '__main__':
    rospy.init_node("follower")
    rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    rospy.spin()
    pass
