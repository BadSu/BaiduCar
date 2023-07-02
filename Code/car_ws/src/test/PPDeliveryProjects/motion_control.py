#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from nav_msgs.msg import Odometry
from std_msgs.msg import  Float32
from pid import *
import time
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Twist

class Parameter(object):
    Period = 0.04

class motion_control():
    def __init__(self):
        self.odom_speed = [0.000,0.000,0.000] #odom读取回来的vx，vy，vth
        self.dt = Parameter.Period #周期40ms
        self.YAW_angle = 0.000 #陀螺仪当前角度
        rospy.Subscriber('/odom', Odometry, self.get_odom)
        rospy.Subscriber('/Angle', Float32, self.get_yaw,queue_size=1)  # 该话题是获取陀螺仪数据的
        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5) #该话题是控制底盘前后左右旋转的
        self.orientationx=None
#单步坐标
    def coordinate(self,x_data_in,y_data_in,z_data_in,total_time=10,error = [0.01,0.01,0.1]):
        while not isinstance(self.orientationx, float):     pass
        flag_i = 0
        flag_loop = True
        c_x_PID = PID(1.5, 0.0, 0.002, 0.3, 0.05, self.dt)
        c_y_PID = PID(1.5, 0.0, 0.002, 0.3, 0.05, self.dt)
        c_z_PID = PID(0.11, 0.0, 0.002, 2, -2, self.dt)
        cur_coordinate_xyz = [0.0,0.0,0.0]
        set_point = [x_data_in,y_data_in,z_data_in]   #
        start_cur_coordinate_x = self.orientationx    #里程计起始位置
        start_cur_coordinate_y = self.orientationy
        time.sleep(0.5)
        #陀螺仪相关变量
        degree_f = 0
        degree_b = self.YAW_angle
        vth = 0.0
        vth_acc = 0.0 #总弧度累加值

        v_acc_xyth = [0.0,0.0,0.0] #一个循环的X，Y和弧度值

        pid_world_coordinates_xy = [0.0,0.0] #PID运算后的世界速度坐标

        pid_frame_coordinates = [0.0,0.0,0.0] #PID运算后小车速度

        rate = rospy.Rate(1/self.dt)        # Hz
        self.send_xyz_motion(0,0,0)         #下发速度
        rate.sleep()
        while_time = 0

        while not rospy.is_shutdown() and (flag_loop == True) and (while_time <= total_time):

            # print(self.vx,self.vy)
            #累加当前陀螺仪弧度
            degree_f = degree_b
            degree_b = self.YAW_angle
            vth = degree_b - degree_f
            if(vth>180.0):
                vth = degree_b - 360.0 - degree_f
            if(vth <-180.0):
                vth = degree_b + 360.0 - degree_f
            v_acc_xyth[2] = radians(vth)
            if -0.0001 <= v_acc_xyth[2] and v_acc_xyth[2] <= 0.0001:  #陀螺仪过滤
                vth_acc += 0.0
            else:
                vth_acc += v_acc_xyth[2]
            #获取当前移动距离
            cur_coordinate_xyz[0] = self.orientationx-start_cur_coordinate_x    
            cur_coordinate_xyz[1] = self.orientationy-start_cur_coordinate_y
            cur_coordinate_xyz[2] = vth_acc
            distance = sqrt (cur_coordinate_xyz[0]*cur_coordinate_xyz[0]+cur_coordinate_xyz[1]*cur_coordinate_xyz[1])

            if set_point[0]!=0 :
                if set_point[0]>0:  x_type = 1
                if set_point[0]<0 : x_type = -1 
                if (abs(set_point[0])-distance) <= error[0]:flag_i +=1
                else:   flag_i = 0
                c_x_PID.setpoint = abs(set_point[0])
                pid_frame_coordinates[0] =x_type * c_x_PID.update(distance)
                #print(distance)

            if set_point[1]!=0 :
                if set_point[1]>0 : y_type = 1 
                if set_point[1]<0 :y_type = -1 
                if (abs(set_point[1]) - distance) <= error[1]:flag_i +=1
                else:   flag_i = 0
                c_y_PID.setpoint = abs(set_point[1])
                pid_frame_coordinates[1] = y_type * c_y_PID.update(distance)
                #print(distance)

            if set_point[2]!=0 :
                total_time = 4
                if abs(degrees(cur_coordinate_xyz[2]) - set_point[2]) <=error[2]:flag_i +=1
                else:   flag_i = 0
                c_z_PID.setpoint = set_point[2]
                pid_frame_coordinates[2] = c_z_PID.update(degrees(cur_coordinate_xyz[2]))
                #print(degrees(cur_coordinate_xyz[2]))
            self.send_xyz_motion(pid_frame_coordinates[0],pid_frame_coordinates[1], pid_frame_coordinates[2] )
            if flag_i <3: flag_loop = True
            else:
                flag_loop = False
            rate.sleep()
            while_time += self.dt
        print(distance)
        print(degrees(cur_coordinate_xyz[2]))
        self.send_xyz_motion(0,0,0)
        return degrees(cur_coordinate_xyz[2])

    def send_xyz_motion(self,x,y,z):
        twist = Twist() 
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.speed_pub.publish(twist)
   
    def get_yaw(self,data):
        self.YAW_angle = data.data
        return self.YAW_angle

    def get_odom(self,msg):  #获取里程计的当前的位姿（7元素）
        self.orientationx = msg.pose.pose.position.x
        self.orientationy = msg.pose.pose.position.y

#if __name__ == '__main__':
  # 初始化节点，注册节点名
  #  rospy.init_node('try')
  #  coor = motion_control()
    #coor.coordinate(0.2, 0, 0)
  #  coor.coordinate(0, 0, -90)

