#!/usr/bin/env python
# coding: utf-8
import os
import rospy
import rospkg
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from hg_bringup.cfg import ColorHSVConfig
import numpy as np
import cv2 
from cv_bridge import CvBridge, CvBridgeError

class color_show:
    def __init__(self):
        print('start')
        rospy.init_node('color')
        rospy.on_shutdown(self.cancel)

        Server(ColorHSVConfig, self.dynamic_reconfigure_callback)

        self.sub_img = rospy.Subscriber("/usb_cam/image_raw", Image, self.Image_callback)
        
        self.bridge = CvBridge()



    def Image_callback(self, data):
        if not isinstance(data, Image): 
            print(1123)
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)
        rgb_img = cv2.resize(frame, (640, 480))
        action = cv2.waitKey(10) & 0xFF
        binary= self.object_follow(rgb_img, self.hsv_range)
        cv2.imshow('原图', frame)
        cv2.imshow('二值图', binary)  
        if action == ord('q') or action == ord('Q'): 
            self.cancel()
            




    def dynamic_reconfigure_callback(self, config, level):
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        return config

    def cancel(self):
            self.sub_img.unregister()
            cv2.destroyAllWindows()
    def object_follow(self, img, hsv_msg):
        src = img.copy()
        # 由颜色范围创建NumPy数组
        # Create NumPy array from color range
        src = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        # 根据特定颜色范围创建mask
        # Create a mask based on a specific color range
        mask = cv2.inRange(src, lower, upper)
        color_mask = cv2.bitwise_and(src, src, mask=mask)
        # 将图像转为灰度图
        # Convert the image to grayscale
        gray_img = cv2.cvtColor(color_mask, cv2.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
        # Get structure elements of different shapes
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # 形态学闭操作
        # Morphological closed operation
        gray_img = cv2.morphologyEx(gray_img, cv2.MORPH_CLOSE, kernel)
        # 图像二值化操作
        # Image binarization operation
        ret, binary = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
        return binary
if __name__=="__main__":
    color=color_show()
    rospy.spin()
    cv2.destroyAllWindows()
