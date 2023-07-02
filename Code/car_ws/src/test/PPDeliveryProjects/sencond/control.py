#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float32, Int32, Bool 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose
import sys, select, termios, tty
import cv2
import numpy as np
from PIL import Image as PImage
import time
import os
from Rosrobot import rosrobot
import math
from tf.transformations import quaternion_from_euler
from tf_conversions import transformations    
class Car_control:
    def __init__(self):
        rospy.init_node('Yolo', anonymous=False)
        self.type_pub = rospy.Publisher("/control_type",Int32,queue_size = 1000)
        self.angle = 0
        self.control_type=1
        self.work_flag = 1
        self.Todeg = 180/math.pi
        self.vel = Twist()

    def get_position(self,data):
        self.position_X=data.pose.pose.position.x
        self.position_Y=data.pose.pose.position.y
    def get_angle(self): 
        Angle =  self.car.get_imu_attitude_data()[0] * self.Todeg
        if Angle <0 :Angle = Angle+360
        if Angle ==180 or Angle == -180 :Angle =180
        self.angle=Angle 

    def send_VelOrder(self,v_x,v_y,v_w):      
            self.vel.linear.x = v_x
            self.vel.linear.y = v_y
            self.vel.linear.z = 0
            self.vel.angular.x = 0
            self.vel.angular.y = 0
            self.vel.angular.z = v_w
            self.vel_pub.publish(self.vel)


    def coordinate(self,x_data_in,y_data_in,z_data_in):
          cur_dis = 0
          if x_data_in>0 or y_data_in>0 or z_data_in>0 :move_type=1
          if x_data_in<0 or y_data_in<0 or z_data_in<0 :move_type=-1
          x_data_in =abs(x_data_in)
          y_data_in = abs(y_data_in)
          flag = True
          self.cur_pose_x = self.position_X
          self.cur_pose_y = self.position_Y
          cur_pose_theta = self.angle
          time.sleep(1)
          while 1:
            try:
              now_x = self.position_X
              now_y = self.position_Y
              now_pose_theta = self.angle

              #计算当前的距离差
              delta_x = (now_x - self.cur_pose_x)
              delta_y = (now_y - self.cur_pose_y)
              delta_w = (z_data_in + cur_pose_theta)
              cur_dis = math.sqrt(delta_x*delta_x + delta_y*delta_y)
              if x_data_in != 0 and y_data_in == 0 and z_data_in == 0:
                if abs(cur_dis - x_data_in)>0.05:
                  print(abs(cur_dis - x_data_in))
                  self.send_VelOrder(0.1*move_type,0,0)

                else:
                  self.send_VelOrder(0,0,0)
                  flag = False
                  break
              elif(x_data_in == 0 and y_data_in != 0 and z_data_in == 0):
                if abs(cur_dis - y_data_in)>0.05:
                  print(abs(cur_dis - y_data_in))
                  self.send_VelOrder(0,0.1*move_type,0)
                else:
                    self.send_VelOrder(0,0,0)
                    rospy.sleep(0.01)
                    flag = False
                    break
              
              elif(x_data_in == 0 and y_data_in == 0 and z_data_in != 0):
                if delta_w >360 :delta_w = delta_w-360
                if delta_w <0   :delta_w = delta_w+360
                
                if abs(now_pose_theta-delta_w)>5:
                  print('angle =',now_pose_theta,'delta_w=',delta_w ,'dether',abs(now_pose_theta-delta_w))
                  self.send_VelOrder(0,0,0.8*move_type)
                else:
                  self.send_VelOrder(0,0,0)
                  flag = False
                  break
            except Exception as e:
              rospy.logerr(e)


    def rotation_angle(self,Z,W):
            euler = transformations.euler_from_quaternion([0,0,Z,W])
            angle_R = euler[0]*self.Todeg
            angle_P = euler[1]*self.Todeg
            angle_Y = euler[2]*self.Todeg
            #if angle_Y<0: angle_Y =angle_Y+360
            return angle_Y


    def control_type_callback(self,data):
        self.control_type=data.data

    def main(self):
     rospy.init_node('Car_control')
     time.sleep(1)
     rospy.Subscriber('/control_type', Int32, self.control_type_callback)
     rospy.Subscriber('/odom', Odometry, self.get_position)
     self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1000)


    while not rospy.is_shutdown():
      if self.work_flag == 1:

        if  self.control_type==0:
               pass
        if self.control_type==2:
                time.sleep(1)
                self.coordinate(0,0,90)   #左转90度
                self.coordinate(0.64,0,0)
                self.coordinate(-0.64,0,0)
                self.coordinate(0,0,-90)
    



if __name__=="__main__":

    control = Car_control()
    
    control.main()
