#!/usr/bin/env python3
#-*- coding:utf-8 -*-import Yolov3CarAPI
import time
import cv2
import rospy
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage, Image
import Yolov3CarAPI
from image_ros_msgs.msg import BoundingBox, BoundingBoxes
import numpy as np
from collections import Counter
from motion_control import *
from math import *
class Yolo:
    def __init__(self):

        rospy.init_node('Yolo', anonymous=False)
        self.sub_img = rospy.Subscriber("/usb_cam/image_raw", Image, self.Image_callback)
        rospy.Subscriber('/control_type', Int32, self.control_type_callback)
        self.control_type_pub = rospy.Publisher("/control_type",Int32,queue_size = 1000)
        self.yolo_type_pub = rospy.Publisher("/yolo_type",Int32,queue_size = 1000)
        self.follow_type_pub = rospy.Publisher("/follow_type",Int32,queue_size = 1000)
        #rospy.Subscriber('/Angle', Float32, self.get_yaw,queue_size=1)  # 该话题是获取陀螺仪数据的

        self.coor = motion_control()
        self.follow_type =3
        self.start_flag=False
        self.control_type =0 
        self.yolo_type = 0
        self.task_1 = True
        self.task_2 = False
        self.task_3 = False
        self.task_4 = False
        self.yolo_num = 0
        self.list_1=[]
        self.list_2=[]
        self.result_1 = None
        self.vortex_num =0
        self.get_res_result_point_x=200
        self.get_res_result_point_y=0
        self.get_res_result_result=None
        self.pick ='konjac'     #  konjac , citrus,  swordfish
        self.pick_num = None
        self.pick_y_save = [0,0,0]
    def control_type_callback(self,data):

        self.control_type =data.data                 

        if data.data == 99:                       #第一次识别漩涡
            print('继续巡线')
            self.task_1 = True
            time.sleep(1)
            self.follow_type_pub.publish(5)            
            self.control_type =0

        if data.data == 98:                       #第二次识别漩涡
            print('继续巡线')
            self.task_1 = True
            time.sleep(1)
            self.follow_type_pub.publish(6)            
            self.control_type =0

        if data.data == 97:                       #第三次识别漩涡
            print('继续巡线')
            self.task_1 = True
            time.sleep(1)
            self.follow_type_pub.publish(7)            #
            self.control_type =0
            
        if data.data == 100:   
            print('继续巡线')
            self.task_1 = True
            time.sleep(1)
            self.follow_type_pub.publish(4)            #以较快速度巡线
            self.control_type =0

    def control_pub(self,ctr_type):               #control.py结束动作后再发送
       flag =True
       while flag :
           if self.control_type ==0 :
             self.control_type_pub.publish(ctr_type)
             flag= False
           else : pass

            
    def Get_result(self,result):
        self.yolo_type =20                     #识别出现问题
        if self.result_1 == 'tower': 
           self.yolo_type =21                   #识别出现问题

        if self.result_1 == 'tornado' :         #击倒牌子
            self.yolo_type =4
            print('识别到 激流勇进')

        if self.result_1 == 'tower' and result =='twg':  #举旗子
            self.yolo_type =5
            print('识别到 滕王阁')

        if self.result_1 == 'dam':      #继续走
            self.yolo_type =6
            print('识别到 腾蛟起风')

        if self.result_1 == 'barge' :         #侧方停车
            self.yolo_type =7
            print('识别到 渔舟唱晚')

        if self.result_1 == 'trade' and self.pick_num == 0:         #夹取放置
            self.yolo_type =14
            print('识别到 物转星移')

        if self.result_1 == 'trade' and self.pick_num == 1:         #夹取放置
            self.yolo_type =15
            print('识别到 物转星移')

        if self.result_1 == 'trade' and self.pick_num == 2:         #夹取放置
            self.yolo_type =16
            print('识别到 物转星移')

        if self.result_1 == 'tower' and result =='hhl':  #举旗子
            self.yolo_type =9
            print('识别到 黄鹤楼')

        if self.result_1 == 'tower' and result =='yyl':  #举旗子
            self.yolo_type =10
            print('识别到 岳阳楼')

        if self.result_1 == 'vortex' and self.vortex_num ==0 :  #Ö±×ß
            print('第一次识别到 回船转舵')
            self.vortex_num = 1
            self.yolo_type =11
            return        

        if self.result_1 == 'vortex' and self.vortex_num ==1 :  #Ö±×ß
            print('第二次识别到 回船转舵')
            self.vortex_num = 2
            self.yolo_type =12
            return

        if self.result_1 == 'vortex' and self.vortex_num ==2 :  #Ö±×ß
            print('第三次识别到 回船转舵')
            self.vortex_num = 0
            self.yolo_type =13
            return
        if self.yolo_type ==20: print('未识别到相应内容')
        if self.yolo_type ==21: print('未识别到楼阁标牌')

    def Image_callback(self, image):           #读取图像并进行处理
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        self.Frame_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        self.start_flag = True

    def pix_to_coord(self,point):              #由像素坐标获取世界坐标
        pts1 = np.float32([[85, 60], [86, -70], [215, -70], [216, 60]])     # 原图像,单位为机械臂mm
        pts2 = np.float32([[273, 292], [378, 296], [369, 219], [284, 217]])      # 标定图像,单位为像素值
        M, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)
        if M is not None:
            tt = np.float32(point).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(tt, M)
            coord = [dst[0][0][0], dst[0][0][1]]
            return coord
        else:
            return [-1, -1]
    
    def get_res_result(self,get_res_result_res):         
        #识别到多个物体
        get_res_result_line_x = 0
        get_res_result_line_y = 0
        if len(get_res_result_res)>1:
            print('识别到多个物体')
            for i in range(0,len(get_res_result_res)):
                save=','.join('%s'%id for id in(get_res_result_res[i])) 
                load=save.split(',')
                line__x = int(load[3])-int(load[1])
                line__y = int(load[4])-int(load[2])
                if get_res_result_line_x > line__x and get_res_result_line_y > line__y:
                    get_res_result_line_x = line__x
                    get_res_result_line_y = line__y
                    self.get_res_result_point_x = (int(load[3])+int(load[1]))/2
                    self.get_res_result_point_y =(int(load[4])+int(load[2]))/2
                    self.get_res_result_result=load[0]
        #识别到单个物体
        if len(get_res_result_res) == 1:
          
            print('识别到单个物体')
            save=','.join('%s'%id for id in(get_res_result_res[0])) 
            load=save.split(',')
            self.get_res_result_point_x = (int(load[3])+int(load[1]))/2
            self.get_res_result_point_y = (int(load[4])+int(load[2]))/2
            self.get_res_result_result=load[0]

    def get_pick(self,pick_res):
        point_y =None
        pick_y = None 
        for i in range(0,len(pick_res)):
            save=','.join('%s'%id for id in(pick_res[i])) 
            load=save.split(',')
            point_y = (int(load[4])+int(load[2]))/2
            if load[0]=='konjac':  #konjac , citrus,  swordfish
               self.pick_y_save[0]= point_y
            if load[0]=='citrus':  
               self.pick_y_save[1]= point_y
            if load[0]=='swordfish': 
               self.pick_y_save[2]= point_y
            if load[0]==self.pick: pick_y = point_y
        if pick_y == max(self.pick_y_save): self.pick_num=2
        if pick_y == min(self.pick_y_save): self.pick_num=0
        if pick_y>min(self.pick_y_save) and pick_y<max(self.pick_y_save):self.pick_num=1
    def main(self): 
        classes = ["tower","hhl","yyl","twg","barge","trade","konjac","citrus","swordfish","tornado","spray","dam","vortex"]
        yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car0413/config.json", 
                                        classes=classes)
        yolov3api.parse_config()
        num = 0
        task_3_flag = True
        Task_1_flag = False
        res = None
        time.sleep(0.1)
        while not rospy.is_shutdown():
          #判定是否进行识别
          if self.task_1 or self.task_2 or self.task_3 or self.task_4:   
            task =True
          else:
            task = False
          if self.start_flag and task:

          #判定识别是否带边框
            if self.task_1 or self.task_2  :             #步骤一和步骤二进行地标的识别 增加边框过滤周围识别物的影响  
              frame, res = yolov3api.run(self.Frame_image,draw=True)
            if self.task_3  :                            #步骤三对路牌进行识别 取消边框增大识别区域
               frame, res = yolov3api.run(self.Frame_image,draw=False)
            #取消注释可查看识别画面
            #cv2.imshow('Live', frame)
            #if cv2.waitKey(27) >= 0:
            #    break  

            #步骤1:移动到标签前某个位置停止
            if res!= [] and self.task_1:             #循迹时检测到地标
                self.get_res_result(res)
                dis = self.pix_to_coord([self.get_res_result_point_x,self.get_res_result_point_y])  #根据识别的像素中心获取标签与世界坐标系的相对距离 单位为MM
                print('检测到',self.get_res_result_result ,'x距离为： ',dis[0],'y距离为：',dis[1])
                if self.get_res_result_result != 'vortex':                
                    self.follow_type_pub.publish(self.follow_type)            #降低巡线速度 具体速度在follow.py中修改
                
                if dis[0] <=70 and Task_1_flag ==False:
                    self.follow_type = 0      
                    print('停止循迹')
                    self.follow_type_pub.publish(0)                            #停止循迹
                    time.sleep(2)
                    self.task_2 = True
                    self.task_1 = False
                    Task_1_flag = False
                    self.follow_type = 3
                    print('********进入步骤2*****************')
                    continue                                              #进入步骤三前重新获取一次图像


            #步骤2 识别地标内容并移动到地标上方         
            if  self.task_2 :  
                self.get_res_result(res)
                res_result=self.get_res_result_result
                dis = self.pix_to_coord([self.get_res_result_point_x,self.get_res_result_point_y])  #根据识别的像素中心获取标签与世界坐标系的相对距离
                self.list_1.append(res_result)             #将识别结果加入列表 获取次数最多的结果
                dis_result = dis                                          
                self.yolo_num += 1
                if self.yolo_num >5 :
                    number = Counter(self.list_1)
                    result = number.most_common()
                    print('地标识别结果为：',result[0][0])
                    self.result_1 = result[0][0]
                    self.list_1.clear()
                    print('地标与小车的距离为: ',(dis[0]+300)/1000)
                    #Angle = self.coor.coordinate(0 ,dis[1]/1000, 0)
                    #self.coor.coordinate(0 ,0, -Angle)
                    self.coor.coordinate((dis[0]+300)/1000 ,0, 0)

                    time.sleep(0.1)
                    #识别到地标twoer,需二次识别的

                    if self.result_1 == 'tower' or self.result_1 =='trade':
                            self.task_3 = True 
                            self.task_2 = False
                            print('********进入步骤3*****************')
                            continue
                    else: 
                            result_2= None                       
                            self.task_4 = True
                            self.task_2 = False
                            print('********进入步骤4*****************')
                            continue

            if self.task_3:
                if task_3_flag and self.result_1=='tower':  
                       print('旋转进行识别') 
                       self.control_pub(3)          #旋转90度进行二次识别
                       task_3_flag=False
                       time.sleep(1)
                       continue
                if task_3_flag and self.result_1=='trade':  
                       print('旋转进行识别') 
                       self.control_pub(2)          #旋转90度进行二次识别
                       result_2= None                       
                       task_3_flag=False
                       time.sleep(1)
                       continue
                if res!= [] and self.control_type == 0 and self.result_1=='trade':
                    self.get_pick(res)
                    print(self.pick_y_save)
                    print('目标方块在%d号点位'  %self.pick_num)
                    task_3_flag = True
                    self.task_4 = True
                    self.task_3 = False
                #旋转后能识别到楼阁牌子
                if res!= [] and self.control_type == 0 and self.result_1=='tower':
                    self.get_res_result(res)
                    self.list_2.append(self.get_res_result_result) 
                    self.yolo_num += 1
                    if self.yolo_num >3:
                        number = Counter(self.list_2)
                        result = number.most_common()
                        result_2 =result[0][0]
                        print('牌子识别结果为： ',result[0][0])
                        self.yolo_num=0
                        self.list_2.clear()
                        task_3_flag = True
                        self.task_4 = True
                        self.task_3 = False
                        print('********进入任务4*****************')
                #未识别到路牌
                else : 
                    self.yolo_num+=0.1   
                    if self.yolo_num >20: 
                        print('未识别到物体')
                        self.task_4 = True 
                        self.task_3 = False
                        print('********进入任务4*****************')

            if self.task_4 :
                        print('开始发布任务')
                        self.Get_result(result_2)

                        self.control_pub(self.yolo_type)
                        self.task_4 = False
                        self.start_flag = False         #结束识别 直到任务完成且有图像输入

if __name__=="__main__":
    yolo = Yolo()
    yolo.main()
    print("done")



