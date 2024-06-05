#! /usr/bin/env python3
# -*- coding:UTF-8 -*-
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import time
import apriltag


tag_flag = 0
global_flag = 1
id = 20
delay = 50
delay_time = 20

def set_roi_forward(h, w, mask):
    search_top = int(3 * h / 4)
    search_bot = search_top + 20
    #感兴趣区域外全部改为黑色
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask
    pass

def follow_line(image, color):
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)

    #转换为HSV图像
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #cv2.namedWindow("hsv image",0)
    #cv2.imshow("hsv image",hsv)
    lower_yellow = np.array([26, 43, 46])
    upper_yellow = np.array([34, 255, 255])
    #转换为二值图，范围内的颜色为白色，其他范围为黑色
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #cv2.namedWindow("binary image",0)
    #cv2.imshow("binary image",mask)

    h, w = mask.shape
    #print(mask.shape)
    
    #设置感兴趣区域ROI
    mask = set_roi_forward(h, w, mask)
    #cv2.namedWindow("region of interest",0)
    #cv2.imshow("region of interest",mask)
    #获得图像矩
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])-235
        cy = int(M['m01'] / M['m00'])
        # print(cx, cy)
        #在质心画圆
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        err = cx - w / 2 - 50
        twist = Twist()
        twist.linear.x = 0.1
        #根据质心与图像中线的偏移设置转角
        twist.angular.z = -float(err / 2.0) / 50
        print("Linear:",twist.linear.x)
        print("Angular:",twist.angular.z)
        print("Message From follow.py Published\n")
        cmd_vel_pub.publish(twist)
    cv2.namedWindow("camera",0)
    cv2.imshow("camera", image)
    cv2.waitKey(1)
    pass

#停止行驶
def stop(id):
    index = id
    cmd_vel_pub = rospy.Publisher("cmd_akm"+str(index), Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    for i in range(20):
        cmd_vel_pub.publish(twist)
    time.sleep(2)

def do_match(frame):
    max = 0.0
    #读取本地图片
    template = cv2.imread(r"/home/chouu/aliyun_demo/src/raceworld/scripts/01.png")
    #摄像头图像与本地图片进行模板匹配，返回一个矩阵，每个像素值代表匹配程度
    res = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
    #获得矩阵中最大值及位置和最小值及位置
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    if max_val>max:
        max = max_val
    return max
    pass

def image_callback(msg):
    global global_flag
    global id
    global tag_flag
    global delay
    global delay_time
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    follow_line(img, "yellow")
    match = do_match(img)
    #print(match)
    #由于行驶在内道，距离路牌较远，所以匹配值要求降低
    if match > 0.57:
        stop(1)
        print("stop now!")
        time.sleep(3)
        rospy.signal_shutdown("~~~")
   

def get_camera():
    rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    pass

if __name__ == '__main__':
    rospy.init_node("get_camera")
    get_camera()
    rospy.spin()
    pass
