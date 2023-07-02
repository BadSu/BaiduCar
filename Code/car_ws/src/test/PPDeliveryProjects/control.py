#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
from std_msgs.msg import  Float32, Int32
import time
from Rosrobot import rosrobot
from math import *
from motion_control import *
from geometry_msgs.msg import  Twist

from Rosrobot import rosrobot
class Car_control:
    def __init__(self):
        rospy.init_node('Control')
        self.car=rosrobot()
        time.sleep(0.5)
        self.car.set_pwm_servo_all(90,90,0,0)         #控制夹爪、旗子复位
        self.control_type=0

        self.guideboard = 1
        self.coor =  motion_control()
        rospy.Subscriber('/control_type', Int32, self.control_type_callback)   # 获取任务信息
        self.control_type_pub=rospy.Publisher("/control_type",Int32, queue_size = 10)  #发布任务结果
        self.car.set_colorful_lamps(0XFF,0,0,0)
    def control_type_callback(self,data):
        if data.data == 100 : data.data = 0
        self.control_type=data.data

    def main(self):
        andle = None
        while not rospy.is_shutdown():

            if  self.control_type==0:
                pass
                

            if self.control_type==3:      #识别
                print('旋转识别标牌')
                self.coor.coordinate(0,0,-90)   #右转90度
                self.control_type_pub.publish(0)  
                self.control_type = 0

            if self.control_type==2:      #识别
                print('旋转识别标牌')
                self.coor.coordinate(0,0,90)   #zuo转90度
                self.control_type_pub.publish(0)  
                self.control_type = 0
            #激流勇进 击倒   4
            if self.control_type==4:      
                print('执行击倒标牌')
                if self.guideboard == 0:        self.coor.coordinate(-0.1,0,0)
                if self.guideboard == 2:        self.coor.coordinate(0.1,0,0)
                time.sleep(0.1)        
                self.coor.coordinate(0,0,-90)     #右转90度
                time.sleep(0.3)
                self.car.set_pwm_servo(1,125)     #夹爪（1号舵机）闭合
                time.sleep(0.2)
                self.car.set_pwm_servo(2,150)     #夹爪抬起（2号舵机）
                time.sleep(0.5)
                self.car.set_pwm_servo(3,180)     #机械臂伸出（3号舵机）
                time.sleep(1)
                self.car.set_pwm_servo(3,0)
                time.sleep(1)
                self.car.set_pwm_servo(2,90)
                time.sleep(0.2)
                self.car.set_pwm_servo(1,90)
                self.coor.coordinate(0,0,90)
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('任务完成')

            #滕王阁 举旗   5
            if self.control_type==5:      
                print('举旗')
                self.car.set_pwm_servo(4,58)            #举旗（4号舵机）
                time.sleep(1)

                self.car.set_colorful_lamps(0XFF,0,255,0)       #控制RGB灯带灯光颜色 第一个参数为控制对象， 0XFF表示控制所有灯 后三个参数控制颜色
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)

                self.car.set_pwm_servo(4,0)
                time.sleep(0.5)
                self.coor.coordinate(0,0,90)
                #self.coor.coordinate(-0.04,0,0)
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('任务完成')

            #腾蛟起风  斜坡   6
            if self.control_type==6:      
                print('斜坡直行')
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('任务完成')


            #渔舟唱晚 侧方停车   7

            if self.control_type==7:      
                time.sleep(1)
                print('执行侧方停车')

                self.coor.coordinate(0,0,90)
                time.sleep(0.1)
                self.coor.coordinate(0.49,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,255,0,0)
                self.car.set_beep(100)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,255,0,0)
                self.car.set_beep(100)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,255,0,0)
                self.car.set_beep(100)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.coor.coordinate(-0.49,0,0)
                time.sleep(0.5)
                self.coor.coordinate(0,0,-90)
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('任务完成')

            #物转星移 夹取、放置   8
            if self.control_type==14 or self.control_type==15 or self.control_type==16:      
                print('执行夹取并放置')
                #self.coor.coordinate(0,0,180)
                if self.control_type == 14:        
                    angle =self.coor.coordinate(0,-0.15,0)
                    self.coor.coordinate(0,0,-angle)
                if self.control_type == 16:   
                    angle=self.coor.coordinate(0,0.15,0) 
                    self.coor.coordinate(0,0,-angle)
                time.sleep(0.2)        
                self.car.set_pwm_servo_all(90,90,0,0)
                time.sleep(0.2)
                self.car.set_pwm_servo(1,70)
                time.sleep(1)
                self.car.set_pwm_servo(3,170)
                self.car.set_pwm_servo(3,170)
                time.sleep(1.5)
                self.car.set_pwm_servo(1,120)
                time.sleep(1)
                self.car.set_pwm_servo(3,0)
                time.sleep(1.5)
                if self.control_type == 14: 
                    angle =self.coor.coordinate(0,0.15,0)
                    self.coor.coordinate(0,0,-angle)
                if self.control_type == 16:  
                    angle =self.coor.coordinate(0,-0.15,0)
                    self.coor.coordinate(0,0,-angle)
                time.sleep(0.5) 
                self.coor.coordinate(0,0,180)

                time.sleep(0.5)
                self.car.set_pwm_servo(3,180)
                self.car.set_pwm_servo(3,180)
                time.sleep(2)
                self.car.set_pwm_servo(1,90)
                time.sleep(1)
                self.car.set_pwm_servo_all(90,90,0,0)
                time.sleep(1.5)
                self.coor.coordinate(0,0,90)
                time.sleep(0.5)
                self.coor.coordinate(-0.05,0,0)
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('任务完成')


            #黄鹤楼 举旗   9
            if self.control_type==9:      
                print('举旗')
                self.car.set_pwm_servo(4,116)
                time.sleep(1)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)

                self.car.set_pwm_servo(4,0)
                time.sleep(0.5)
                self.coor.coordinate(0,0,90)
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('任务完成')

            #岳阳楼 举旗   10
            if self.control_type==10:      
                print('举旗')
                self.car.set_pwm_servo(4,172)

                time.sleep(1)

                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_pwm_servo(4,0)
                time.sleep(0.5)

                self.coor.coordinate(0,0,90)
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('任务完成')

            #漩涡  直行  11
            if self.control_type==11:      
                print('左转')
                self.coor.coordinate(0,0,18)
                time.sleep(0.1)
                self.control_type_pub.publish(99)   #任务完成
                self.control_type=0
                print('任务完成')
  
            if self.control_type==12:      
                print('右转')
                self.coor.coordinate(0,0,-96)
                time.sleep(0.1)
                self.control_type_pub.publish(98)   #任务完成
                self.control_type=0


            if self.control_type==13:      
                print('右转')
                self.coor.coordinate(0,0,-96)
                time.sleep(0.1)
                self.control_type_pub.publish(97)   #任务完成
                self.control_type=0
                print('任务完成')
                
            if self.control_type==20:     
                time.sleep(1) 
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('未执行任务')
            if self.control_type==21:      
                self.coor.coordinate(0,0,90)
                self.control_type_pub.publish(100)   #任务完成
                self.control_type=0
                print('未执行任务')

if __name__=="__main__":

        control = Car_control()
        
        control.main()
