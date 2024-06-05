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
import pupil_apriltags as apriltag


tag_flag = 0
global_flag = 1
path = 1
id = 20
pre_id = 20
delay = 50
delay_time = 5 # 延时时间



'''
功能：根据全局变量path，按车道进行巡线

可以调的参数
1.两侧的检测范围
2.hsv取色的范围
3.cx的大小以及计算时取的比例
4.角速度

可用参数搭配：
0.6 0.88
'''

def change_path():
    global path
    if path == 0:
        path = 1
    else:
        path = 0

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
            if(tag.tag_id == 3):
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
    kernel = np.ones((7,7), np.uint8)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([26, 43, 46])
    lower_yellow = np.array([26, 43, 0])
    upper_yellow = np.array([34, 255, 255])
    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w = mask1.shape
    mask1 = cv2.dilate(mask1, kernel, 2)
    mask1 = set_roi_forward1(h, w, mask1)

    lower_gray = np.array([0, 0, 120])
    upper_gray = np.array([0, 0, 200])
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
    twist.linear.x = 0.7
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
    twist.linear.x = 0.7
    twist.angular.z = -float(err / 1.4) / 50
    cmd_vel_pub.publish(twist)
    # cv2.imshow("window1", mask2+mask1)
    # cv2.waitKey(1)
    cv2.imshow("window1", image)
    cv2.waitKey(1)
    pass

def follow_line(frame):
    global path
    if(path == 0):
        follow_line_in(frame)
    else:
        follow_line_out(frame)
    pass


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
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    if get_tag(img):
        delay = 0
        if id != pre_id:
            pre_id = id
            change_path()
        else:
            pass

    follow_line(img)
    
    region = range_detection(img)
    length = len(region)
    if (length<=2 and not tag_flag and delay>delay_time):
        # 绘制轮廓
        id = 0
        flag = 0 # 是否看到标志的flag
        for box in region:
            left_point = box[1][0]
            if left_point<300:
                # 控制视频扫描的区域，防止庙门的干扰
                # left_point是最左侧，300大致在居中位置
                # print(left_point)
                continue
            cv2.drawContours(img, [box], 0, (0, 255, 0), 2)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # 颜色统计
            temp = hsv[box[2][1]:box[0][1],box[0][0]:box[2][0]]
            num = 0
            white = 0
            print(box)
            for i in range(box[0][1]-box[2][1]-1):
                for j in range(box[2][0]-box[0][0]-1):
                    if (temp[i][j][1]>= 0 and temp[i][j][1]<= 30 and temp[i][j][2]>=220 and temp[i][j][2]<=255) or (((temp[i][j][0]>= 0 and temp[i][j][0]<= 10) or (temp[i][j][0]>=155 and temp[i][j][0]<=180)) and temp[i][j][1]>= 43 and temp[i][j][1]<= 255 and temp[i][j][2]>=46 and temp[i][j][2]<=255) : 
                        num += 1
                    if(temp[i][j][1]>= 0 and temp[i][j][1]<= 30 and temp[i][j][2]>=220 and temp[i][j][2]<=255):
                        white +=1
            red = 0
            red = num-white
            print("id = ",id)
            id+=1
            print("num = ",num,";")
            print("white = ",white,";")
            print("red = ",red,";")
            print('\n')
            if(num >1300 and red>1200): # 核心指标(线上设置的大一点，线下设置的小一点——惯性)
                print("stop sign is here.")
                print('\n')
                flag = 1
        if(flag): 
            global_flag = 0
        if(global_flag==0):
            index = 1;
            stop(index)
            print("stop now!")
            time.sleep(3)
            rospy.signal_shutdown("~~~")
    else:
        # print("delay=",delay)
        delay+=1
        for box in region:
            cv2.drawContours(img, [box], 0, (0, 255, 0), 2)
    rospy.sleep(0.01)
    cv2.imshow("ouput",img)
    cv2.waitKey(1)
   

def get_camera():
    rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    pass

if __name__ == '__main__':
    rospy.init_node("get_camera")
    get_camera()
    rospy.spin()
    pass
