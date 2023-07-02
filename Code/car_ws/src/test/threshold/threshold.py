#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import sys, os
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QGraphicsScene, QMessageBox, QTableWidgetItem
from PyQt5.QtGui import QPixmap, QImage

from UI import GUI, color, line
import numpy as np
import cv2
from time import sleep


class MainWindow(QMainWindow, GUI.Ui_Mainwindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

class ThresholdWindow(QMainWindow,color.Ui_Colorwindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

class LineWindow(QMainWindow,line.Ui_Linewindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

class UI_System():
    def __init__(self):
        """ UI系统初始化 """
        self.mainview = MainWindow()                  # 主窗口
        self.thresholdview = ThresholdWindow()        # 阈值调节窗口
        self.lineview = LineWindow()                  # 车道线窗口
        self.mainview.RGBMSAN.clicked.connect(self.RGBMSAN_button)
        self.mainview.HSVMSAN.clicked.connect(self.HSVMSAN_button)
        self.mainview.CDXSBAN.clicked.connect(self.CDXSBAN_button)

        self.thresholdview.WCSCAN.clicked.connect(self.WCSCAN_pass_button)
        self.lineview.WCCXAN.clicked.connect(self.WCCXAN_pass_button)
        self.mainview.show()

        # 定时器：40ms捕获一帧，即25fps
        self._timer = QtCore.QTimer(self.mainview)
        self._timer.timeout.connect(self.main_loop)
        self._timer.setInterval(40)     # 25fps,即40ms
        self._timer.start()

        # 以下为视觉参数初始化
        self.low_x = 0
        self.low_y = 0
        self.low_z = 0
        self.high_x = 0
        self.high_y = 0
        self.high_z = 0
        self.mode = 0
        self.status = 0

        # camera_device = "/dev/cam_lane" #获取图像
        # cmd = 'ls -l '+str(camera_device)
        # val = os.popen(cmd).readlines()
        # segements = val[0].split('video')
        # camera_id = int((segements[-1])[0])
        # self.cap = cv2.VideoCapture(camera_id)
        self.cap = cv2.VideoCapture('训练师车道线视频.mp4')
        self.status, frame = self.cap.read()

    def RGBMSAN_button(self):
        """ RGB模式按钮功能 """
        self.mode = 1
        print("open RGB moled")
        self.cap.set(3, 640) # width
        self.cap.set(4, 480) # height
        self.thresholdview.show()
        main_size = self.mainview.geometry()
        self.thresholdview.setGeometry(main_size.x(),main_size.y()-28,main_size.width(),main_size.height())
        sleep(1)
        self.mainview.close()

    def HSVMSAN_button(self):
        """ HSV模式按钮功能 """
        self.mode = 2           
        print("open HSV moled")
        self.cap.set(3, 640) # width
        self.cap.set(4, 480) # height
        self.thresholdview.show()
#        self.thresholdview.setFixedSize(800, 600)
        main_size = self.mainview.geometry()
        self.thresholdview.setGeometry(main_size.x(),main_size.y()-28,main_size.width(),main_size.height())
        sleep(1)
        self.mainview.close()

    def CDXSBAN_button(self):
        """ 车道线识别模式按钮功能 """
        self.mode = 3           
        print("打开车道线角度调节模式")
        self.cap.set(3, 640) # width
        self.cap.set(4, 480) # height
        self.lineview.show()
        main_size = self.mainview.geometry()
        self.lineview.setGeometry(main_size.x(),main_size.y()-28,main_size.width(),main_size.height())
        print(main_size)
        sleep(1)
        self.mainview.close()

    def WCSCAN_pass_button(self):
        """ 阈值调节完成按钮功能 """
        print("cloes threshold")
#        self._timer.stop()
        sleep(0.1)
        self.mainview.show()
        main_size = self.thresholdview.geometry()
        self.mainview.setGeometry(main_size.x(),main_size.y()-28,main_size.width(),main_size.height())
        self.mainview.show()
        sleep(1)
        self.thresholdview.close()

    def WCCXAN_pass_button(self):
        """ 车道线摄像头角度调节完成按钮功能 """
        print("完成车道线摄像头角度调节")
#        self._timer.stop()
        sleep(0.1)

        main_size = self.lineview.geometry()
        self.mainview.setGeometry(main_size.x(),main_size.y()-28,main_size.width(),main_size.height())
        self.mainview.show()
        sleep(1)
        self.lineview.close()

    def main_loop(self):
        """ 通过界面计时器执行的循环 """
        try:   
            if self.status : 
                if (self.mode == 2 or self.mode == 1) :
                    """ 阈值调节模式 """
                    status_s, frame = self.cap.read()                                                

                    self.low_x = self.thresholdview.LHRH.value() #获取设置参数
                    self.high_x = self.thresholdview.HHRH.value()
                    self.low_y = self.thresholdview.LSGH.value()
                    self.high_y = self.thresholdview.HSGH.value()
                    self.low_z = self.thresholdview.LVBH.value()
                    self.high_z = self.thresholdview.HVBH.value()

                    low_x ="min:"+str(self.low_x)
                    high_x ="max:"+str(self.high_x)
                    low_y ="min:"+str(self.low_y)
                    high_y ="max:"+str(self.high_y)
                    low_z ="min:"+str(self.low_z)
                    high_z ="max:"+str(self.high_z)

                    self.thresholdview.LHRB.setText(low_x)
                    self.thresholdview.HHRB.setText(high_x)
                    self.thresholdview.LSGB.setText(low_y)
                    self.thresholdview.HSGB.setText(high_y)
                    self.thresholdview.LVBB.setText(low_z)
                    self.thresholdview.HVBB.setText(high_z)
                   
                    lower_array=np.array([self.low_x,self.low_y,self.low_z]) #给到阈值划分
                    upper_array=np.array([self.high_x,self.high_y,self.high_z])

                    if self.mode == 2: #模式选择 2 为HSV模式                     
                        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #图像格式转换hsv
                        mask=cv2.inRange(hsv,lower_array,upper_array) #设定取值范围
                    else: #不为2 为RGB模式
                        mask=cv2.inRange(frame,lower_array,upper_array) #设定取值范围

                    img1 = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)        # 使用其他显示器需转换为RGB模式
                    QImg = QImage(img1, img1.shape[1], img1.shape[0], QImage.Format_RGB888)
                    self.thresholdview.YT.setPixmap(QPixmap.fromImage(QImg))        # 显示原图
                    self.thresholdview.YT.setScaledContents (True)      # 让图片自适应label大小

                    img2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)        # 使用其他显示器需转换为RGB模式
                    QImg = QImage(img2, img2.shape[1], img2.shape[0], QImage.Format_RGB888)
                    self.thresholdview.SC.setPixmap(QPixmap.fromImage(QImg))        # 显示阈值图
                    self.thresholdview.SC.setScaledContents (True)      # 让图片自适应label大小 

                if self.mode == 3:
                    """ 车道线校准模式 """              
                    H = 480
                    W = 640
                    status, apic = self.cap.read()
            
                    cv2.line(apic,(38,480),(160,200),(0,255,242),2)#left_outside
                    cv2.line(apic,(88,480),(194,200),(0,255,242),2)#left_inside
                    cv2.line(apic,(598,480),(482,200),(0,255,242),2)#right_outside
                    cv2.line(apic,(546,480),(448,200),(0,255,242),2)#right_inside
                    cv2.line(apic,(0,int(0.99*H)),(640,int(0.99*H)),(0,255,0),1)#_up
                    cv2.line(apic,(0,int(0.7*H)),(640,int(0.7*H)),(0,255,0),1)#_up_crossroad
                    cv2.line(apic,(0,int(0.85*H)),(640,int(0.85*H)),(0,255,0),1)#low
                    cv2.line(apic,(320,0),(320,480),(0,0,255),3)#middele                   

                    img = cv2.cvtColor(apic, cv2.COLOR_BGR2RGB)        # 使用其他显示器需转换为RGB模式
                    QImg = QImage(img, img.shape[1], img.shape[0], QImage.Format_RGB888)
                    self.lineview.CX.setPixmap(QPixmap.fromImage(QImg))        # 显示阈值图
                    self.lineview.CX.setScaledContents (True)      # 让图片自适应label大小

        except:
            s=sys.exc_info()
            print("Error %s happened on line %d" %(s[1],s[2].tb_lineno))   

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = UI_System()
    sys.exit(app.exec_())

