#!/usr/bin/env python
# -*- coding:UTF-8 -*-
# -*- coding: utf-8 -*-
# from PIL import Image
# import pytesseract
import os
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import time

global_flag = 0;

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
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    region = range_detection(img)

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
        for i in range(box[0][1]-box[2][1]):
            for j in range(box[2][0]-box[0][0]):
                if (temp[i][j][1]>= 0 and temp[i][j][1]<= 30 and temp[i][j][2]>=220 and temp[i][j][2]<=255) or (((temp[i][j][0]>= 0 and temp[i][j][0]<= 10) or (temp[i][j][0]>=155 and temp[i][j][0]<=180)) and temp[i][j][1]>= 43 and temp[i][j][1]<= 255 and temp[i][j][2]>=46 and temp[i][j][2]<=255) : 
                    num += 1

        print("id = ",id)
        id+=1
        print("num = ",num,";")
        print('\n')
        if(num >1150): # 核心指标
            print("stop sign is here.")
            print('\n')
            flag = 1
    cv2.imshow("ouput",img)
    cv2.waitKey(1)
    # 停车条件判断
    if(flag):global_flag=1
    else:global_flag=0
    index = 1;
    if(global_flag):stop(index)

def get_camera():
    rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    pass

if __name__ == '__main__':
    rospy.init_node("get_camera")
    get_camera()
    rospy.spin()
    pass

# # 读取图片
# imagePath = 'F:/figure/sign.png'
# img = cv2.imread(imagePath)




# # 切割图片
# for box in region:
#     temp = img.copy()
#     cv2.rectangle(temp,box[0],box[2],(255,0,0), 2)
#     cv2.imwrite('output',temp)
#     cv2.imshow('F:/figure/output.png',temp)
#     cv2.waitKey(0)
# cv2.imshow('img', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# 初始解决方案
# image = cv2.imread('F:/figure/sign.png')
# image_ = cv2.GaussianBlur(image, (5, 5), 0)
# grayscaled = cv2.cvtColor(image_,cv2.COLOR_BGR2GRAY)
# # ret,th1 = cv2.threshold(grayscaled,127,255,cv2.THRESH_BINARY)
# # # th2 = cv2.adaptiveThreshold(grayscaled,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
# # #             cv2.THRESH_BINARY,11,2)
# th3 = cv2.adaptiveThreshold(grayscaled,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
#             cv2.THRESH_BINARY,11,2)
# cv2.imwrite("F:/figure/scan.png",th3)
# cv2.imshow("input",image)
# cv2.imshow("output",th3)
# cv2.waitKey(0)  # 等待时间，毫秒级
