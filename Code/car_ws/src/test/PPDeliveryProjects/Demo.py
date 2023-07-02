#!/usr/bin/env python
#-*- coding:utf-8 -*-

from Rosrobot import rosrobot
import time
import cv2
class Test():

    def __init__(self):

        self.car =rosrobot()
        self.car.set_car_type(1)
        self.car.create_receive_threading()
        self.car.set_pwm_servo_all(90,90,0,0)

    def motor_encoder_test(self):

        self.car.set_motor(20,20,20,0)
        self.car.set_motor(20,20,20,0)
        time.sleep(1)
        num =0
        while num <100:
            d_encoder =self.car.get_motor_encoder()
            print(d_encoder)
            num+=0.5
        self.car.set_motor(0,0,0,0)

    def servo4_test(self):
        self.car.set_pwm_servo(4,58)
        time.sleep(5)
        self.car.set_pwm_servo(4,116)
        time.sleep(5)
        self.car.set_pwm_servo(4,172)
        time.sleep(5)
        self.car.set_pwm_servo(4,0)

    def servo3_test(self): 
        self.car.set_pwm_servo(3,180)
        time.sleep(5)
        self.car.set_pwm_servo(3,0)

    def servo2_test(self):
        time.sleep(5)
        self.car.set_pwm_servo(2,180)
        time.sleep(5)
        self.car.set_pwm_servo(2,90)

    def servo1_test(self):        #爪子先张开 再紧闭 最后张开
        self.car.set_pwm_servo(2,180)
        time.sleep(1)
        self.car.set_pwm_servo(1,75)
        time.sleep(5)
        self.car.set_pwm_servo(1,125)
        time.sleep(5)
        self.car.set_pwm_servo(1,90)

    def beep_test(self):
        self.car.set_beep(100)
        time.sleep(1)
        self.car.set_beep(100)
        time.sleep(1)
        self.car.set_beep(100)
        time.sleep(1)

    def tube_test(self):
                self.car.set_colorful_lamps(0XFF,255,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,255,255,255)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,255,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,255,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,255,255,255)
                time.sleep(0.5)
                self.car.set_colorful_lamps(0XFF,0,0,0)
    
    def start(self,data):
        if data ==1:    self.motor_encoder_test()
        if data ==5:    self.servo1_test()
        if data ==4:    self.servo2_test()
        if data ==3:    self.servo3_test()
        if data ==2:    self.servo4_test()
        if data ==6:    self.beep_test()
        if data ==7:    self.tube_test()
        if data ==8:    self.camera_test()
    def  camera_test(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        cap.set(3, 640) # width
        cap.set(4, 480) # height
        num= 0
        while cap.isOpened():
            retval, frame = cap.read()
            cv2.imshow('Live', frame)
            num+=0.1
            if cv2.waitKey(27) >= 0:break
            if num >20: break
        cap.release() 
        cv2.destroyAllWindows()
if __name__=="__main__":
    test = Test()
    test.start(7)

    print("done")
