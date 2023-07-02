#!/usr/bin/env python
# encoding: utf-8
import sys
import math
import rospy
import random
import threading
from math import pi
from time import sleep
from sensor_msgs.msg import Imu, MagneticField, JointState
from Rosrobot import rosrobot
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
from dynamic_reconfigure.server import Server
from hg_bringup.cfg import PIDparamConfig
import time

class hg_driver:
    def __init__(self):
        #rospy.on_shutdown(self.cancel)
        # 弧度转角度
        # Radians turn angle
        self.car = rosrobot()
        self.car.set_car_type(1)
        self.imu_link = rospy.get_param("~imu_link", "imu_link")
        self.Prefix = rospy.get_param("~prefix", "")
        self.car_type = rospy.get_param("~car_type", "S1")  #D1 差速  O1 全向轮 M1 麦克纳姆轮  T1履带轮  S1 三轮
        print(self.car_type)
        self.wheel_diameter = float(rospy.get_param("~wheel_diameter", 0.1)) # unit: m
        self.encoder_per_loop = int(rospy.get_param("~encoder_per_loop", 1550)) # 轮子转一圈编码输出多少脉冲
        self.encoder_to_meter = (self.wheel_diameter * pi) / self.encoder_per_loop
        self.meter_to_encoder = float(self.encoder_per_loop) / (self.wheel_diameter * pi)
        
        #三轮相关参数
        self.length_a = float(rospy.get_param("~length_a",0.105))    #a轮子之间的距离
        self.length_b = float(rospy.get_param("~length_b",0.105))    #b轮子之间的距离
        self.length_c = float(rospy.get_param("~length_c",0.105))    #c轮子之间的距离
        #两轮、履带相关参数
        self.Todeg = 180/math.pi
        self.length_D1 = float(rospy.get_param("~length_D1",0.15))    #a轮子之间的距离
        self.cos_30 =math.sqrt(3)/2    
        self.length = 0.1    

        self.xlinear_limit = rospy.get_param('~xlinear_speed_limit', 1.0)
        self.ylinear_limit = rospy.get_param('~ylinear_speed_limit', 1.0)
        self.angular_limit = rospy.get_param('~angular_speed_limit', 5.0)
        self.now = time.time()
        self.last = self.now
        self.first=0
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.velPublisher = rospy.Publisher("/pub_vel", Twist, queue_size=100)
        self.angPublisher = rospy.Publisher('/Angle', Float32, queue_size=100)
        self.dyn_server = Server(PIDparamConfig, self.dynamic_reconfigure_callback)
        self.car.create_receive_threading()

    def cancel(self):
        self.velPublisher.unregister()
        self.sub_cmd_vel.unregister()
        # Always stop the robot when shutting down the node
        rospy.loginfo("Close the robot...")
        rospy.sleep(1)

    def pub_data(self):
        # 发布小车运动速度、陀螺仪数据、电池电压
        ## Publish the speed of the car, gyroscope data, and battery voltage
        while not rospy.is_shutdown():
            sleep(0.05)
            twist = Twist()
            d_encoder, encoder_speed, meter_speed = self.Get_encoder_speed()
            vx, vy, angular = self.Get_car_speed(d_encoder, encoder_speed, meter_speed)
            Angle =  self.car.get_imu_attitude_data()[0] * self.Todeg
            # 将小车当前的线速度和角速度发布出去
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = angular
            self.velPublisher.publish(twist)
            self.angPublisher.publish(Angle)
            
    def Get_encoder_speed(self):
        d_encoder =self.car.get_motor_encoder()
        encoder_speed = [0, 0, 0, 0]
        meter_speed = [0, 0, 0, 0]
        
        for i in range(4):
               encoder_speed[i] = -float(d_encoder[i])*100    #编码差 / 时间 = 速度（ticks/s)
        for index in range(len(encoder_speed)):
                  meter_speed[index] = float(encoder_speed[index]) * self.encoder_to_meter
        return [d_encoder, encoder_speed, meter_speed]

    def Get_car_speed(self, d_encoder, encoder_speed, meter_speed):

        self.vA, self.vB, self.vC, self.vD = meter_speed
        vx, vy, vth = self.get_S1_car_speed(self.vA, self.vB, self.vC)

        v_raw = [vx, vy, vth]

        return [vx, vy, vth]
    def get_S1_car_speed(self, speed_a, speed_b, speed_c):
            D  =  self.cos_30*(self.length_a + self.length_b + self.length_c)
            a1 =  (self.length_b + 0.5*self.length_c)/D
            a2 = -(self.length_a + 0.5*self.length_c)/D
            a3 = -(0.5*self.length_a -0.5*self.length_b)/D
            b1 = -(self.cos_30*self.length_c)/D
            b2 = -(self.cos_30*self.length_c)/D
            b3 =  (self.cos_30*self.length_b + self.cos_30*self.length_a)/D
            c1 = -(self.cos_30)/D
            c2 = -(self.cos_30)/D
            c3 = -(self.cos_30)/D
            v_x  = a1 * speed_a + a2 * speed_b + a3 * speed_c
            v_y  = b1 * speed_a + b2 * speed_b + b3 * speed_c
            v_th = c1 * speed_a + c2 * speed_b + c3 * speed_c

            v = [v_x, v_y, v_th]
            return [v_x, v_y, v_th]

    
    # 计算下发到电机的速度
    def send_car_speed(self,speed_x,speed_y,speed_th):
        speed_x = speed_x/1.4
        speed_y = speed_y/1.4
        speed_th = speed_th

        speed_1 =  self.cos_30 * speed_x - 0.55 * speed_y - self.length * speed_th
        speed_2 = -self.cos_30 * speed_x - 0.55 * speed_y - self.length * speed_th
        speed_3 =  speed_y - self.length * speed_th    
        speed_4 = 0


        if  speed_1>1:  speed_1 = 1 
        if  speed_1<-1 : speed_1 =-1

        if  speed_2>1:  speed_2 = 1 
        if  speed_2<-1 : speed_2 =-1
        else :speed_2 =speed_2

        if  speed_3>1:  speed_3 = 1 
        if  speed_3<-1 : speed_3 =-1
        else :speed_3 =speed_3

        if  speed_4>1:  speed_4 = 1 
        if  speed_4<-1 : speed_4 =-1
        else :speed_4 =speed_4


        return [100*speed_1, 100*speed_2, 100*speed_3,100*speed_4]

    def RGBLightcallback(self, msg):
        # 流水灯控制，服务端回调函数 RGBLight control
        '''
        effect=[0, 6]，0：停止灯效，1：流水灯，2：跑马灯，3：呼吸灯，4：渐变灯，5：星光点点，6：电量显示
        speed=[1, 10]，数值越小速度变化越快。
        '''
        if not isinstance(msg, Int32): return
        # print ("RGBLight: ", msg.data)
        for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzercallback(self, msg):
        # 蜂鸣器控制  Buzzer control
        if not isinstance(msg, Bool): return
        # print ("Buzzer: ", msg.data)
        if msg.data:
            for i in range(3): self.car.set_beep(1)
        else:
            for i in range(3): self.car.set_beep(0)

    def cmd_vel_callback(self, msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
        if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
        vx = msg.linear.x
        vy = msg.linear.y
        angular = msg.angular.z
        # 小车运动控制,vel: ±1, angular: ±5
        # Trolley motion control,vel=[-1, 1], angular=[-5, 5]
        # rospy.loginfo("cmd_velx: {}, cmd_vely: {}, cmd_ang: {}".format(vx, vy, angular))
        m1,m2,m3,m4=self.send_car_speed(vx,vy,angular)
        self.car.set_motor(m1,0.98*m2,m3,m4)
    def dynamic_reconfigure_callback(self, config, level):
        self.car.set_pid_param(config['Kp'], config['Ki'], config['Kd'])
        # print("PID: ", config['Kp'], config['Ki'], config['Kd'])
        self.linear_max = config['linear_max']
        self.linear_min = config['linear_min']
        self.angular_max = config['angular_max']
        self.angular_min = config['angular_min']
        return config

if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = hg_driver()
        driver.pub_data()
        rospy.spin()
    except:
        rospy.loginfo("Final!!!")
