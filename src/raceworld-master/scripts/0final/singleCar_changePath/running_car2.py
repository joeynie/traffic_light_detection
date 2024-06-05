#! /usr/bin/env python
# -*- coding:UTF-8 -*-
from geometry_msgs.msg import Twist

import os
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import time
import imghdr
from sys import exc_info
import apriltag


tag_flag = 0
global_flag = 1
path = 1
id = 20
pre_id = 20
delay = 50
delay_time = 20



'''
功能：根据全局变量path，按车道进行巡线

可以调的参数
1.两侧的检测范围
2.hsv取色的范围
3.cx的大小以及计算时取的比例
4.角速度

高性能PC可用参数搭配：
0.6 0.88
阿里云服务器：
0.05 5.0
'''
#换道
def change_path():
    global path
    path = 1-path;
    # if path == 0:
    #     path = 1
    # else:
    #     path = 0
#读取二维码
def get_tag(frame):
    global id
    global tag_flag 
    global delay
    detector = apriltag.Detector()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    if tags == []:
        tag_flag=0
        return False
    else:
        delay = 0
        tag_flag = 1
        for tag in tags:
            print("Tag detected with id: ", tag.tag_id)
            id = tag.tag_id
        return True
    pass


def set_roi_forward1(h, w, mask):
    search_top = int(1 * h / 2)
    search_bot = search_top + 180
    #感兴趣区域外全部改为黑色
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
    print("in")
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)
    kernel = np.ones((7,7), np.uint8)
    #转换为HSV图像
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([26, 43, 46])
    lower_yellow = np.array([26, 43, 0])
    upper_yellow = np.array([34, 255, 255])
    #对车道线图像二值化
    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w = mask1.shape
    #膨胀操作
    mask1 = cv2.dilate(mask1, kernel, 2)
    #设置感兴趣区域ROI
    mask1 = set_roi_forward1(h, w, mask1)

    lower_gray = np.array([0, 0, 120])
    upper_gray = np.array([0, 0, 200])
    #对车道边界二值化
    mask2 = cv2.inRange(hsv, lower_gray, upper_gray)
    #膨胀操作
    mask2 = cv2.dilate(mask2, kernel, 2)
    #设置感兴趣区域ROI
    mask2 = set_roi_forward2(h, w, mask2)
    #cv2.namedWindow("mask1",0)
    #cv2.imshow("mask1", mask1)
    #cv2.namedWindow("mask2",0)
    #cv2.imshow("mask2", mask2)
    # print(h,w)
    #获得图像矩
    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
    if M1['m00'] == 0:
        cx1 = 630
        if M2['m00'] > 0:
            cx2 = int(M2['m10'] / M2['m00'])
        elif M2['m00'] == 0:
            cx2 = 0
        cy = 0
        pass

    elif M1['m00'] > 0:
        cx1 = int(M1['m10'] / M1['m00'])
        if M2['m00'] > 0:
            cx2 = int(M2['m10'] / M2['m00'])
        elif M2['m00'] == 0:
            cx2 = 0
        cy = int(M1['m01'] / M1['m00'])

    cx = int((cx1-cx2)*50/64+cx2)
    #在质心画圆
    cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
    err = cx - w / 2 - 60
    twist = Twist()
    twist.linear.x = linerSpeed
    #根据质心与图像中线的偏移设置转角
    twist.angular.z = -float(err / 5.0) / 50
    cmd_vel_pub.publish(twist)
    cv2.namedWindow("Camera",0)
    cv2.imshow("Camera", image)
    cv2.waitKey(1)
    pass

def set_roi_forward3(h, w, mask):
    mask[0:200, 0:w] = 0
    mask[200:360, 0:540] = 0
    mask[360:480, 0:w] = 0
    return mask
    pass


def follow_line_out(image):
    print("out")
    """
    外道巡线函数
    """
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)
    kernel = np.ones((7, 7), np.uint8)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([26, 43, 0])
    upper_yellow = np.array([34, 255, 255])
    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w = mask1.shape
    mask1 = set_roi_forward1(h, w, mask1)
    mask1 = cv2.dilate(mask1, kernel, iterations=2)
    

    lower_gray = np.array([0, 0, 120])
    upper_gray = np.array([0, 0, 200])
    mask2 = cv2.inRange(hsv, lower_gray, upper_gray)
    kernel = np.ones((9,9), np.uint8)
    mask2 = set_roi_forward3(h, w, mask2)
    mask2 = cv2.dilate(mask2, kernel, iterations=2)
    #cv2.namedWindow("mask1",0)
    #cv2.imshow("mask1", mask1)
    #cv2.namedWindow("mask2",0)
    #cv2.imshow("mask2", mask2)
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

    cx = int((cx2-cx1)*33/64+cx1)

    cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
    err = cx - w / 2 - 57
    twist = Twist()
    twist.linear.x = linerSpeed
    twist.angular.z = -float(err / 5.0) / 50
    cmd_vel_pub.publish(twist)
    # cv2.waitKey(1)
    cv2.namedWindow("Camera",0)
    cv2.imshow("Camera", image)
    cv2.waitKey(1)
    pass

def follow_line(frame):
    global path
    if(path == 0):
        follow_line_in(frame)
    else:
        follow_line_out(frame)
    pass


def stop(id):
    """
    停车动作函数
    """
    index = id
    cmd_vel_pub = rospy.Publisher("cmd_akm"+str(index), Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    for i in range(20):
        cmd_vel_pub.publish(twist)
    time.sleep(1)

def do_match(frame):
    max = 0.0
    #读取本地图片
    template = cv2.imread("/home/Workspace/aliyun_demo/src/raceworld-master/scripts/0final/singleCar_changePath/01.png")
    cv2.imshow("template",template)
    imageGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    templateGray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    #摄像头图像与本地图片进行模板匹配，返回一个矩阵，每个像素值代表匹配程度
    res = cv2.matchTemplate(imageGray, templateGray, cv2.TM_CCOEFF_NORMED)
    #获得矩阵中最大值及位置和最小值及位置
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    if max_val>max:
        max = max_val
    return max
    pass

def image_callback(msg):
    """
    回调函数
    """
    # 获取图像并开始寻线
    global global_flag
    global path
    global id
    global pre_id
    global tag_flag
    global delay
    global delay_time
    global light #0:no 1:green 2:yellow 3:red
    global linerSpeed;
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    light = rospy.get_param("LightOn",0);
    # print("light:%d",light);
    if light==3:
        stop(1)
        print("stop")
    else:
        if get_tag(img):
            delay=0
            if id != pre_id:
                pre_id = id
                change_path()
            else:
                pass
        if light==2: linerSpeed = 0.05
        else: linerSpeed = 0.15
        follow_line(img)
        print("linerSpeed:%f",linerSpeed);
    
    # match = do_match(img)
    match=0
    # print(match)
    # if match > 0.7 or light==3:
    #     stop(1)
    #     print("stop now!")
        # time.sleep(3)
        # rospy.signal_shutdown("~~~")
   

def get_camera():
    rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    pass

if __name__ == '__main__':
    rospy.init_node("get_camera")
    get_camera()
    rospy.spin()
    pass
