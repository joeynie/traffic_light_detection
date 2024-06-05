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

global global_flag
global_flag = 1

def range_detection(img):
    """
    探测文字所在的范围(可以用来写OCR)
    """
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 利用Sobel边缘检测生成二值图
    sobel = cv2.Sobel(gray, cv2.CV_8U, 1, 0, ksize=3)
    # 二值化
    ret, binary = cv2.threshold(sobel, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY)

    # 膨胀、腐蚀
    element1 = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 9))
    element2 = cv2.getStructuringElement(cv2.MORPH_RECT, (24, 6))

    # 膨胀一次，让轮廓突出
    dilation = cv2.dilate(binary, element2, iterations=1)

    # 腐蚀一次，去掉细节
    erosion = cv2.erode(dilation, element1, iterations=1)

    # 再次膨胀，让轮廓明显一些
    dilation2 = cv2.dilate(erosion, element2, iterations=2)

    #  查找轮廓和筛选文字区域
    region = []
    contours, hierarchy = cv2.findContours(dilation2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):
        cnt = contours[i]
        # 计算轮廓面积，并筛选掉面积小的
        area = cv2.contourArea(cnt)
        if (area < 1000): continue 
        # 找到最小的矩形
        rect = cv2.minAreaRect(cnt) 
        # print ("rect is: ") 
        # print (rect) 
        # box是四个点的坐标 
        box = cv2.boxPoints(rect) 
        box = np.int0(box) 
        # 计算高和宽 
        height = abs(box[0][1] - box[2][1]) 
        width = abs(box[0][0] - box[2][0]) 
        # 根据文字特征，筛选那些太细的矩形，留下扁的
        if (height > width * 1.3):continue
        region.append(box)

    return region


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
    time.sleep(2)

def set_roi_forward(h, w, mask):
    search_top = int(3 * h / 4)
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask
    pass

def follow_line(image, color):
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([26, 43, 46])
    upper_yellow = np.array([34, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    h, w = mask.shape
    # print(h,w)
    mask = set_roi_forward(h, w, mask)
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        # print(cx, cy)
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        # BEGIN CONTROL
        err = cx - w / 2 - 50
        twist = Twist()
        twist.linear.x = 0.1*5
        twist.angular.z = -float(err / 1.5) / 50
        cmd_vel_pub.publish(twist)
    cv2.imshow("window1", image)
    cv2.waitKey(1)
    pass

def image_callback(msg):
    """
    回调函数
    """
    # 获取图像并开始寻线
    global global_flag
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    region = range_detection(img)
    follow_line(img,"yellow")
    # 绘制轮廓
    id = 0
    flag = 0 # 是否看到标志的flag
    for box in region:
        cv2.drawContours(img, [box], 0, (0, 255, 0), 2)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 颜色统计
        temp = hsv[box[2][1]:box[0][1],box[0][0]:box[2][0]]
        num = 0
        # print(box)
        for i in range(box[0][1]-box[2][1]-1):
            for j in range(box[2][0]-box[0][0]-1):
                if (temp[i][j][1]>= 0 and temp[i][j][1]<= 30 and temp[i][j][2]>=220 and temp[i][j][2]<=255) or (((temp[i][j][0]>= 0 and temp[i][j][0]<= 10) or (temp[i][j][0]>=155 and temp[i][j][0]<=180)) and temp[i][j][1]>= 43 and temp[i][j][1]<= 255 and temp[i][j][2]>=46 and temp[i][j][2]<=255) : 
                    num += 1

        print("id = ",id)
        id+=1
        print("num = ",num,";")
        print('\n')
        if(num >1400): # 核心指标(线上设置的大一点，线下设置的小一点——惯性)
            print("stop sign is here.")
            print('\n')
            flag = 1
    if(flag): 
        global_flag = 0
    cv2.imshow("ouput",img)
    cv2.waitKey(1)
    if(global_flag==0):
        index = 1;
        stop(index)
        print("stop now!")
        time.sleep(3)
        rospy.signal_shutdown("~~~")
    rospy.sleep(0.01)
   


    

def get_camera():
    rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    pass

if __name__ == '__main__':
    rospy.init_node("get_camera")
    get_camera()
    rospy.spin()
    pass
