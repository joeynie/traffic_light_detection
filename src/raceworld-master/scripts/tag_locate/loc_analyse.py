#! /usr/bin/env python
# -*- coding:UTF-8 -*-
from multiprocessing.connection import wait
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
# 上面用来摄像机图像获取
import  pupil_apriltags as apriltag
import math

cameraParams_Intrinsic = [504.78216005824515, 504.78216005824515, 320.5, 240.5]  # camera_fx, camera_fy, camera_cx, camera_cy——内参
camera_matrix = np.array(([504.78216005824515, 0.0, 320.5],
                         [0.0, 504.78216005824515, 240.5],
                         [0, 0, 1.0]), dtype=np.double) # 内参矩阵

class Detecte:
    def __init__(self):
        self.tags = None # tag数据群——便于互通
        self.bridge = cv_bridge.CvBridge() # 发布
        self.car1_sub = rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, self.callback) # 从第一辆车里调
        self.frame = None
    
    # 回调函数
    def callback(self,msg):
        # detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))
    
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("catch", self.frame)
        # 1. 上面从ros中获取图像,并展示图像

        # # 检测apriltag
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)    # The image must be a grayscale image of type numpy.uint8 灰度化

        tag_detector = apriltag.Detector()  # Build a detector for apriltag
        self.tags = tag_detector.detect( gray, 
                            estimate_tag_pose=True,
                            camera_params=cameraParams_Intrinsic, # cameraParams_Intrinsic为相机内参
                            tag_size=13)  # Perform apriltag detection to get a list of detected apriltag
        if(not(self.tags == [])):
            self.detect_tags(self.frame)
            # 2. 上面完成solve前的数据准备工作
            # 3. 下面准备一些计算所用的矩阵
            object_3d_points = np.array(([0, 0, 0],
                                    [0, 200, 0],
                                    [150, 0, 0],
                                    [150, 200, 0]),
                                    dtype=np.double)    # apriltag coordinates in the World coordinate system——apriltag世界坐标系坐标

            object_2d_point = np.array((self.tags[0].corners[0].astype(int),
                                    self.tags[0].corners[1].astype(int),
                                    self.tags[0].corners[2].astype(int),
                                    self.tags[0].corners[3].astype(int)),
                                    dtype=np.double)    # apriltag coordinates in the Image pixel system——apriltag图像坐标系坐标
            dist_coefs = np.array([0,0,0,0,0], dtype=np.double)    # Distortion coefficient: k1, k2, p1, p2, k3(不考虑畸变时)
            # The function solvepnp receives a set of corresponding 3D and 2D coordinates
            # and calculates the geometric transformation corresponding to the two sets of coordinates (rotation matrix rvec, translation matrix tvec).
            # 函数solvepnp用来计算几何变换的坐标（旋转矩阵与平移矩阵）
            found, rvec, tvec = cv2.solvePnP(object_3d_points, object_2d_point, camera_matrix, dist_coefs)
            
            rotM = cv2.Rodrigues(rvec)[0]
            camera_postion = -np.matrix(rotM).T * np.matrix(tvec)
            # print(camera_postion.T)
            thetaZ = math.atan2(rotM[1, 0], rotM[0, 0])*180.0/math.pi
            thetaY = math.atan2(-1.0*rotM[2, 0], math.sqrt(rotM[2, 1]**2 + rotM[2, 2]**2))*180.0/math.pi
            thetaX = math.atan2(rotM[2, 1], rotM[2, 2])*180.0/math.pi
            # camera coordinates——相机坐标计算
            x = tvec[0]
            y = tvec[1]
            z = tvec[2]
            (x, y) = self.RotateByZ(x, y, -1.0*thetaZ)
            (x, z) = self.RotateByY(x, z, -1.0*thetaY)
            (y, z) = self.RotateByX(y, z, -1.0*thetaX)
            Cx = x*-1
            Cy = y*-1
            Cz = z*-1
            
            print("camera position:",Cx, Cy, Cz)       
            print("camera rotation:", thetaX, thetaY, thetaZ)
            print('\n')

            # Extra points for debug the accuracy
            '''
            Out_matrix = np.concatenate((rotM, tvec), axis=1)
            pixel = np.dot(camera_matrix, Out_matrix)
            pixel1 = np.dot(pixel, np.array([0, 100, 105, 1], dtype=np.double))
            pixel2 = pixel1/pixel1[2]
            print("test point coordinate:", pixel2) 
            '''
    
    
    
    # 计算函数——用来solve
    def RotateByZ(self,Cx, Cy, thetaZ):
        rz = thetaZ*math.pi/180.0
        outX = math.cos(rz)*Cx - math.sin(rz)*Cy
        outY = math.sin(rz)*Cx + math.cos(rz)*Cy
        return outX, outY
    def RotateByY(self,Cx, Cz, thetaY):
        ry = thetaY*math.pi/180.0
        outZ = math.cos(ry)*Cz - math.sin(ry)*Cx
        outX = math.sin(ry)*Cz + math.cos(ry)*Cx
        return outX, outZ
    def RotateByX(self,Cy, Cz, thetaX):
        rx = thetaX*math.pi/180.0
        outY = math.cos(rx)*Cy - math.sin(rx)*Cz
        outZ = math.sin(rx)*Cy + math.cos(rx)*Cz
        return outY, outZ
    # tag信息
    def detect_tags(self,img):
        for tag in self.tags:
            # 标注
            cv2.circle(img, tuple(tag.corners[0].astype(int)), 4,(0,0,255), 2) # right-bottom
            cv2.circle(img,tuple(tag.corners[1].astype(int)), 4,(0,0,255), 2) # left-top
            cv2.circle(img, tuple(tag.corners[2].astype(int)), 4,(0,0,255), 2) # right-top
            cv2.circle(img, tuple(tag.corners[3].astype(int)), 4,(0,0,255), 2) # left-bottom
            # tags info
            print("family:",tag.tag_family)
            print("id:", tag.tag_id)
            print("conners:", tag.corners)
            print("homography:", tag.homography)    
            print("pose_R:%s\npose_T:%s\npose_err:%s" %(tag.pose_R, tag.pose_t,tag.pose_err))
        



if __name__=="__main__":
    rospy.init_node("get_pose")
    D1 = Detecte()
    rospy.spin()
