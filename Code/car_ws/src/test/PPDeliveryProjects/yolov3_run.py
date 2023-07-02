#!/usr/bin/env python3
#-*- coding:utf-8 -*-
import Yolov3CarAPI
import time
import cv2
import rospy
from motion_control import *
import rospy
import numpy as np

coor =  motion_control()
rospy.init_node('test')

def pix_to_coord(point):
        pts1 = np.float32([[85, 60], [86, -70], [215, -70], [216, 60]])     # Ô­Í¼Ïñ,µ¥Î»Îª»úÐµ±Û×ø±êÖµ
        pts2 = np.float32([[273, 292], [378, 296], [369, 219], [284, 217]])      # ±ê¶¨Í¼Ïñ,µ¥Î»ÎªÏñËØÖµ
        M, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)
        if M is not None:
            tt = np.float32(point).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(tt, M)
            coord = [dst[0][0][0], dst[0][0][1]]
            return coord
        else:
            return [-1, -1]
#from std_msgs.msg import String, Float32, Int32, Bool
classes=["tower","hhl","yyl","twg","barge",
         "trade","konjac","citrus","swordfish","tornado",
         "spray","dam","vortex"]

yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car0403/config.json", 
                                classes=classes)


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(3, 640) # width
cap.set(4, 480) # height

"""

fps=20         
size=(int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
      int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out=cv2.VideoWriter()
out.open("output.mp4", fourcc, fps, size)

#frame = cv2.flip(frame,1)


"""

yolov3api.parse_config()
max_x = 0
max_y = 0
res_result=None
while cap.isOpened():
    retval, frame = cap.read()
    frame, res = yolov3api.run(frame,draw=False)
    cv2.imshow('Live', frame)
    if cv2.waitKey(27) >= 0:
        break
    if res!=[]:
        for i in range(0,len(res)):
            str_2=','.join('%s'%id for id in(res[i])) 
            load=str_2.split(',')
            line_x = int(load[3])-int(load[1])
            line_y = int(load[4])-int(load[2])
            point_x = (int(load[3])+int(load[1]))/2
            point_y =(int(load[4])+int(load[2]))/2
            time.sleep(2)
            if load[0]=='konjac' or  load[0]==  'citrus' or  load[0]=='swordfish':
                dis = pix_to_coord([point_x,point_y])
                print('result: ',load[0]  , '  x :' ,dis[0],  '  y: ',dis[1])


cap.release() 
#out.release() 
cv2.destroyAllWindows()
'''
# image
#cap = cv2.VideoCapture("output.mp4")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(3, 640) # width
cap.set(4, 480) # height

"""

fps=20         
size=(int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
      int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out=cv2.VideoWriter()
out.open("output.mp4", fourcc, fps, size)

#frame = cv2.flip(frame,1)


"""

yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car0311/config.json", 
                                   classes=classes,
                                   draw=True)
yolov3api.parse_config()
rospy.init_node('yolo')
type_pub = rospy.Publisher("/control_type",Int32,queue_size = 1000)
#str=' '
while cap.isOpened():
    retval, frame = cap.read()
    frame, res = yolov3api.run(frame)
    #print("res_list:", res)
    #out.write(frame)
    if res!= []:
      str=','.join('%s'%id for id in(res[0])) 
      num=str.split(',')
      image_x = abs(int(num[3])-int(num[1]))
      image_y = abs(int(num[4])-int(num[2]))
      image_mid_x =int(num[1])+image_x/2
      image_mid_y =int(num[2])+image_y/2
      real_X = (100/image_x)*(image_mid_x - 320)
      real_Y = (100/image_y)*(image_mid_y - 240)
      type_pub.publish(0)
    cv2.imshow('Live', frame)
    if cv2.waitKey(5) >= 0:
        break
cap.release() 
#out.release() 
cv2.destroyAllWindows()
'''


# img_path (str)
"""
yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car/config.json", 
                                   classes=classes,
                                   img_path="car_images/")
yolov3api.parse_config()
yolov3api.run()

"""



"""
# video_path (str)

yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car/config.json", 
                                   classes=classes,
                                   video_path="./baidu_car_images/fire.mp4",
                                   draw=True)
yolov3api.parse_config()
yolov3api.run()
"""

