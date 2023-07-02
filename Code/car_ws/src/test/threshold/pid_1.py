#!/usr/bin/env python3
#coding=utf-8
import sys, os, time
import cv2
import numpy as np
from PIL import Image as PImage
from Rosmaster_Lib import Rosmaster


def Get_Hist(img):
    """
        计算直方图数组
    """
    img = np.sum(img, axis=0)
    img.reshape(img.size, 1)

    hist = np.int32(np.around(Normalize(img, 255)))
    # print(hist.shape)
    img = np.zeros((hist.size, hist.size, 3))  # 创建用于绘制直方图的全0图像

    bins = np.arange(hist.size).reshape(hist.size, 1)  # 直方图中各bin的顶点位置

    pts = np.column_stack((bins, hist))
    cv2.polylines(img, [pts], False, (0, 255, 0))
    img = np.flipud(img)
    return img, hist

def Normalize(data, scale):
    """
    归一化-->[0.0, 1.0]
    :param data:
    :param scale:
    :return: [-0.5, 0.5]*scale
    """
    mx = max(data)
    mn = min(data)
    m = (mx - mn)/2 + mn

    return [((float(i) - m) / (mx - mn)+0.5)*scale for i in data]

def Range_Convert(data, src, dest):
    data = min(src) if data <= min(src) else data
    data = max(src) if data >= max(src) else data

    data *= (min(dest)-max(dest))/(min(src)-max(src))   # 放大
    data += max(dest)-min(dest)/2+min(dest)
    return data

def get_yellow_lane_bin_img(frame, low_rh, high_rh,low_gs, high_gs,low_bv, high_bv):    # 提取黄色车道线
    # 输入原图,返回缩放后的原图与车道线二值图
    try:
        lower_array = np.array([low_rh,low_gs,low_bv]) # 第二个参数调节亮度
        upper_array = np.array([high_rh,high_gs, high_bv])

        selection = 1
        if selection == 1:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)#使用HSV进行车道线识别
            mask = cv2.inRange(hsv, lowerb=lower_array, upperb=upper_array)
        if selection == 2:
            mask = cv2.inRange(frame, lowerb=lower_array, upperb=upper_array)#使用RGB进行车道线识别

        __img = PImage.fromarray(mask)  
        _img = np.array(__img)

        # img = __img.resize((120, 120), PImage.ANTIALIAS)   # 缩小
        # img = np.zeros((120, 120))  # 创建全0图像
        # cv2.resize(_img, (120, 120), img, cv2.INTER_AREA)
        small_img = np.zeros((120, 120))  # 创建全0图像
        small_img = cv2.resize(_img, (120, 120), small_img, cv2.INTER_AREA)
        small_img = small_img.astype(np.uint8)
        retval, bin_img = cv2.threshold(small_img, 125, 255, cv2.THRESH_BINARY)

        small_img = small_img.astype(np.float32)
        img_3chanel = cv2.cvtColor(small_img, cv2.COLOR_GRAY2BGR)
        # origin_img = frame.resize((120, 120), PImage.ANTIALIAS)
        # origin_img = np.zeros((120, 120, 3))  # 创建全0图像
        # cv2.resize(frame, (120, 120), origin_img,  cv2.INTER_AREA)
        origin_img = np.zeros((120, 120, 3))  # 创建全0图像
        origin_img = cv2.resize(frame, (120, 120),  origin_img, cv2.INTER_AREA)
        cv2.imshow("bin_img", mask)
        cv2.waitKey(10)
        return origin_img, bin_img
    except:
        s = sys.exc_info()
        print("Error '%s' happened on line %d" % (s[1], s[2].tb_lineno))

class CONTROLER:
    def __init__(self):
        self.imgInd = 0
        self.sign_speed = 30
        self.target_point = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        
        self.last_control_time = time.time() * 1000
        self.current_time = self.last_control_time
        self.last_error = 0.0

        

        self.pid_angle_value = 0.0
        # self.pid_cmd_vel = Twist()
        self.pid_speed = 1
        self.pid_out_rate = 1.0
        self.pid_out_power = 1.0
        self.mn_width = 5.0
        self.mn_height = 60.0
        self.mn_area = 1300.0
        self.mn_distance = 65.0
        self.windup_guard = 1.0
        self.P = 1.0
        self.I = 1.0
        self.D = 0.0

        self.RC_buff_length = 5
        self.history_left_lane = [0] * self.RC_buff_length  # 惯性滤波缓存区
        self.history_right_lane = [0] * self.RC_buff_length  # 惯性滤波缓存区
        self.history_angle = [float(0.0)] * self.RC_buff_length  # 惯性滤波缓存区
        


        self.camera_correction_done = False     # 摄像头校准标志
        self.crossroads = False                 # 十字路口标志
        self.last_direction = 0                 # 上次转弯的方向,debug用
        
        self.road_width = 0        # 车道宽度
        self.last_bais = 0
        self.low_x = 0   # HSV的H阈值
        self.high_x = 34    # HSV的H阈值
        self.low_y = 35   # HSV的S阈值
        self.high_y = 255     # HSV的S阈值
        self.low_z = 46   # HSV的V阈值
        self.high_z = 255     # HSV的V阈值

    def fliter_left_right_bias(self, hist_array):
            """
                滤波器函数，过滤直方图中的干扰
                :param hist_array:        直方图数组
                :return:脉冲数量，最左边的脉冲峰顶的索引位置，最右边脉冲峰顶的索引位置，过滤后的图像波形
            """
            try:
                left_distance = right_distance = 0
                shape = list(hist_array)
                # 计算脉冲个数
                pulse_flag = 0  # 波形连续标志
                pulse_height = []
                pulse_index = []
                # 1 对直方图进行波形切割，记录了波峰的高度和索引
                for i in range(len(shape)):
                    if pulse_flag and (shape[i] > 50 and i < len(shape) - 1):
                        _pulse_height.append(shape[i])
                        _pulse_index.append(i)
                    elif pulse_flag and ((shape[i] <= 50) or (i == (len(shape) - 1))):
                        pulse_flag = 0  # 波形分割
                        pulse_height.append(_pulse_height)
                        pulse_index.append(_pulse_index)
                    elif shape[i] > 0:
                        pulse_flag = 1
                        _pulse_height = []
                        _pulse_index = []
                        _pulse_height.append(shape[i])
                        _pulse_index.append(i)

                pulse_point_preprocess = []
                pulse_height_preprocess = []
                # 2 根据脉冲高度和面积，过滤掉干扰脉冲
                for i, points in enumerate(pulse_height):
                    height = max(points)
                    area = sum(points)
                    width = len(points)
                    if (width >= self.mn_width and (area >= self.mn_area or height >= self.mn_height)):
                        pulse_height_preprocess.append(points)
                        pulse_point_preprocess.append(pulse_index[i])

                # 对过滤后的脉冲波形进行绘图
                __array = np.array([0] * len(shape))
                for points in pulse_point_preprocess:
                    for ind in points:
                        __array[ind] = shape[ind]
                __array.reshape((1, len(shape)))
                img = np.zeros((len(shape), len(shape), 3))  # 创建用于绘制直方图的全0图像
                bins = np.arange(len(shape)).reshape(len(shape), 1)  # 作为索引，0~len()
                pts = np.column_stack((bins, __array))
                cv2.polylines(img, [pts], False, (0, 255, 0))
                hist_img = np.flipud(img)   # 生成滤波后的波形图

                # 3 对于过滤后的脉冲进行左右双峰的定位
                if len(pulse_point_preprocess) == 1:  # 单峰
                    points = pulse_height_preprocess[0]
                    mx_h = max(points)
                    mx_h_ind = points.index(mx_h)
                    left_distance = right_distance = pulse_point_preprocess[0][mx_h_ind]

                elif len(pulse_point_preprocess) == 2:  # 双峰
                    # 方法1：取峰顶最高点
                    points = pulse_height_preprocess[0]
                
                    mx_h = max(points)
                    mx_h_ind = points.index(mx_h)
                    left_distance = pulse_point_preprocess[0][mx_h_ind]
                    
                    points = pulse_height_preprocess[-1]
                    mx_h = max(points)
                    mx_h_ind = points.index(mx_h)
                    right_distance = pulse_point_preprocess[-1][mx_h_ind]
                    # 方法2：取峰顶中心点
                    
                elif len(pulse_point_preprocess) > 2:  # 多峰，从左向右，从右向左取最高的2个
                    left_ind = 0
                    right_ind = len(pulse_point_preprocess) - 1
                    left_mx = []
                    left_mx_ind = []
                    right_mx = []
                    right_mx_ind = []
                    while left_ind < right_ind:
                        points = pulse_height_preprocess[left_ind]
                        if sum(points) > sum(left_mx):
                            left_mx = points
                            left_mx_ind = pulse_point_preprocess[left_ind]
                        points = pulse_height_preprocess[right_ind]
                        if sum(points) > sum(right_mx):
                            right_mx = points
                            right_mx_ind = pulse_point_preprocess[right_ind]
                        left_ind += 1
                        right_ind -= 1
                        pass
                    if left_ind == right_ind:
                        points = pulse_height_preprocess[right_ind]
                        if sum(points) > sum(right_mx):
                            right_mx = points
                            right_mx_ind = pulse_point_preprocess[right_ind]

                    # 多峰变双峰
                    # 方法1 求左侧最高峰的最高点，右侧最高峰的最高点
                    # mx_h = max(left_mx)
                    # mx_h_ind = left_mx.index(mx_h)
                    # left_distance = left_mx_ind[mx_h_ind]
                    #
                    # mx_h = max(right_mx)
                    # mx_h_ind = right_mx.index(mx_h)
                    # right_distance = right_mx_ind[mx_h_ind]

                    # 多峰变双峰
                    # 方法2 求左侧最高峰的中心点，右侧最高峰的中心点
                    left_distance = int(sum(left_mx_ind) / len(left_mx_ind))
                    right_distance = int(sum(right_mx_ind) / len(right_mx_ind))

                return len(pulse_point_preprocess), left_distance, right_distance, hist_img

            except:
                s = sys.exc_info()
                print("Error '%s' happened on line %d" % (s[1], s[2].tb_lineno))

    def get_servo_angle(self):
        try:
            """
            :return:舵机数值
            """
            self.output = self.output ** int(self.pid_out_power)
            angle = self.pid_out_rate * self.output
            
            #print("pid_power_out=", self.output, "angle=", angle)
            return angle
        except:
            s = sys.exc_info()
            print("Error '%s' happened on line %d" % (s[1], s[2].tb_lineno))

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        """

        error = self.target_point - feedback_value
        error = error
        self.current_time = time.time() * 1000  # ms
        delta_time = self.current_time - self.last_control_time
        delta_error = error - self.last_error

        self.PTerm = self.P * error
        self.ITerm += error * delta_time

        # 超调震荡窗口
        if (self.ITerm < -self.windup_guard):
            self.ITerm = -self.windup_guard
        elif (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard

        self.DTerm = 0.0
        if delta_time > 0:
            self.DTerm = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time = self.current_time
        self.last_error = error

        self.output = self.PTerm + (self.I * self.ITerm) + (self.D * self.DTerm)

    def put_left_lane(self, data):
        for i in range(self.RC_buff_length - 1):
            self.history_left_lane[i] = self.history_left_lane[i + 1]
        self.history_left_lane[self.RC_buff_length - 1] = data

    def put_right_lane(self, data):
        for i in range(self.RC_buff_length - 1):
            self.history_right_lane[i] = self.history_right_lane[i + 1]
        self.history_right_lane[self.RC_buff_length - 1] = data

    def put_angle(self, data):
        for i in range(len(self.history_angle)-1):
            self.history_angle[i] = self.history_angle[i + 1]
        self.history_angle[-1] = data

    def get_history_turn(self):
    # 舵机角度<0为左转
        angle = sum(self.history_angle) / len(self.history_angle)
        if angle < 0:
            return 1
        elif angle > 0:
            return 2
        else:
            return 0
      
    def get_history_angle(self):
        return sum(self.history_angle) / len(self.history_angle)

    def get_left_lane_history(self):
        return sum(self.history_left_lane[:self.RC_buff_length])

    def get_right_lane_history(self):
        return sum(self.history_right_lane[:self.RC_buff_length])

    def call_back_30hz(self, cv_image):
            try:
                print("*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*")
                # print(time.localtime( time.time() ))
                #cv_image = self.bridge.imgmsg_to_cv2(image_raw, "bgr8")
                line_low = 0.99
                if self.crossroads:
                    #print("THIS IS CROSSROADS")
                    # 在十字路口扩大窗口
                    line_up = 0.7
                else:
                    line_up = 0.85
                if not self.camera_correction_done:
                    # 相机校准过程中，图像不缩放，画出中线以供校准
                    H = cv_image.shape[0]
                    W = cv_image.shape[1]
                    cv2.line(cv_image, pt1=(0, int(H * line_low)), pt2=(W, int(H * line_low)), color=(0, 255, 0), thickness=2)
                    cv2.line(cv_image, pt1=(0, int(H * line_up)), pt2=(W, int(H * line_up)), color=(0, 255, 0), thickness=2)
                    cv2.line(cv_image, pt1=(W // 2, 0), pt2=(W // 2, H), color=(0, 255, 0), thickness=2)
                    cv2.imshow("lanes", cv_image)

                    print("camera_correction...")
                    if  cv2.waitKey(1) == 27:
                        self.camera_correction_done = True
                        cv2.destroyAllWindows()
                else:
                    # print("start run!")
                    if self.sign_speed == 0:
                        print("speed_x, speed_z: 0, 0")
                        return
                    # img_raw = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                    # 正常行驶时，缩放，显示滤波图像
                    origin_img, bin_img = get_yellow_lane_bin_img(cv_image, self.low_x, self.high_x,self.low_y, self.high_y,self.low_z, self.high_z)      # 获取缩放后的原图与二值图
                    H = origin_img.shape[0]
                    W = origin_img.shape[1]
                    cv2.line(origin_img, pt1=(0, int(H * line_low)), pt2=(W, int(H * line_low)), color=(0, 255, 0), thickness=2)
                    cv2.line(origin_img, pt1=(0, int(H * line_up)), pt2=(W, int(H * line_up)), color=(0, 255, 0), thickness=2)
                    cv2.line(origin_img, pt1=(W // 2, 0), pt2=(W // 2, H), color=(0, 255, 0), thickness=2)
                    bin_img_rectangle_ROI = bin_img[int(H * line_up):int(H * line_low), :]     # 获取二值图的ROI区域
                    hist_bin_line_img, hist_array = Get_Hist(bin_img_rectangle_ROI[:, :])    # 计算直方矩阵，以及脉冲波形图
                    line_count, left, right, hist_bin_line_img_filtered = self.fliter_left_right_bias(hist_array)    # 对直方图进行滤波
                    #print("after fliter_left_right_bias ")
                    # 统一格式，显示在窗口
                    #origin_img = origin_img.astype(np.uint8)
                    # print(type(origin_img), origin_img.shape)
                    #hist_bin_line_img = hist_bin_line_img.astype(np.uint8)
                    # print(type(hist_bin_line_img), hist_bin_line_img.shape)
                    #hist_bin_line_img_filtered = hist_bin_line_img_filtered.astype(np.uint8)
                    # print(type(hist_bin_line_img_filtered), hist_bin_line_img_filtered.shape)
                    #stackedimage = np.hstack((origin_img, hist_bin_line_img, hist_bin_line_img_filtered))
                    #cv2.imshow("lanes", stackedimage)
                    #if  cv2.waitKey(1) == 27:
                        
                    #    self.pid_cmd_vel.angular.x = 0
                    #    self.pid_cmd_vel.angular.z = 0
                    #    self.pub.publish(self.pid_cmd_vel)
                    #    rospy.signal_shutdown("closed!")
                    #print("line_count=", line_count, "left=", left, "right=", right)
                    #print("last_direction=", self.last_direction)    # 上次转弯的方向,debug用
                    mid_line = int(len(hist_array)/2)  # 视野中线
                    # 根据线的数量决定PID的输入量
                    if line_count >= 2:
                        if ((right - left) >= self.mn_distance):
                            bais = mid_line - (right + left) // 2
                            #rospy.loginfo("检测到多线")
                            #rospy.loginfo("bais>0时应左转，bais<0时应右转, bais=%s", bais)
                            self.update(bais)
                            angle_value = self.get_servo_angle()
                            self.put_angle(angle_value)

                            self.put_left_lane(1)
                            self.put_right_lane(1)
                        else:
                            #rospy.loginfo("检测到多线，但左右线距过窄，延续之前的角度")
                            angle_value = self.get_history_angle()  # 延续之前的角度
                            self.put_angle(angle_value)
                        if line_count >= 3:  # 十字路口
                            self.crossroads = True
                        else:
                            self.crossroads = False

                    if line_count < 2:
                        # 左右车道线惯性滤波
                        l = self.get_left_lane_history()
                        r = self.get_right_lane_history()
                        if l and r:
                            angle_value = self.get_history_angle()  # 延续之前的角度
                            self.put_angle(angle_value)
                            if left < mid_line:
                                self.put_left_lane(1)
                                self.put_right_lane(0)

                            if left > mid_line:
                                self.put_left_lane(0)
                                self.put_right_lane(1)
                        elif l or r:
                            # 单线情况，跟随之前时刻的转向
                            if self.get_history_turn() == 1:  # 前一个时刻是左转
                                #rospy.loginfo("前一个时刻是左转")
                                bais =0.5 * (len(hist_array) - right) 
                                self.update(bais)
                                angle_value = self.get_servo_angle()
                                self.put_angle(angle_value)
                            elif self.get_history_turn() == 2:
                                bais = 0.5 * (0 - left)
                                self.update(bais)
                                angle_value = self.get_servo_angle()
                                self.put_angle(angle_value)

                            else:
                                # 之前没有转向，说明只是开机时出现的无效单线
                                #rospy.loginfo("之前没有转向，说明只是开机时出现的无效单线")
                                angle_value = 0.0
                                self.put_angle(angle_value)
                                # print("line_count=", line_count, "left=", left, "right=", right)

                            # 维持以前的状态
                            if l:
                                self.put_left_lane(1)
                                self.put_right_lane(0)
                            else:
                                self.put_left_lane(0)
                                self.put_right_lane(1)

                        else:
                            # 惯性滤波结果是0
                            self.put_left_lane(0)
                            self.put_right_lane(0)
                            angle_value = 0.0
                            self.put_angle(angle_value)

                    angle_value = self.get_history_angle()  # 取惯性滤波后的角度值
                    print(str(time.time())+":angel == "+str(angle_value))
                    
                    # 正数右转，负数左转
                    self.pid_angle_value = angle_value
                    self.last_direction = pid.get_history_turn()
                    if self.pid_speed > 0:
                        #print("self.pid_cmd_vel.linear.x=", self.pid_speed)
                        #print("self.pid_cmd_vel.angular.z=", self.pid_angle_value)
                        # self.pid_cmd_vel.linear.x = self.pid_speed
                        print("1、speed_x, speed_z: %f, %f"%(self.pid_speed, self.pid_angle_value))
                    else:
                        # self.pid_cmd_vel.linear.x = self.sign_speed
                    
                        # self.pid_cmd_vel.angular.z = self.pid_angle_value
                        print("2、speed_x, speed_z: %f, %f"%(self.pid_speed, self.pid_angle_value))
                        
                    # self.pub.publish(self.pid_cmd_vel)
            except:
                s = sys.exc_info()
                print("Error '%s' happened on line %d" % (s[1], s[2].tb_lineno))

if __name__ == '__main__':
    pid = CONTROLER()
    
    cap = cv2.VideoCapture('lane_video.mp4')
    ret, frame = cap.read()
    # print(frame.shape())

    ret = cap.set(3, 480) # width
    ret = cap.set(4, 360) # height

    while(cap.isOpened()):
        ret, frame = cap.read()
        frame = cv2.resize(frame, (480, 360))

        pid.call_back_30hz(frame)
        # cv2.imshow('Live', frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    print("done")



# ******************************************************************88
# # 创建Rosmaster对象 bot  Create the Rosmaster object bot
# bot = Rosmaster()
# # print(bot.help)

# # 启动接收数据，只能启动一次，所有读取数据的功能都是基于此方法
# # Start to receive data, can only start once, all read data function is based on this method
# bot.create_receive_threading()

# # 控制电机运动 Control motor movement
# def run_motor(M1, M2, M3, M4):
#     bot.set_motor(M1, M2, M3, M4)
#     return M1, M2, M3, M4


# bot.set_motor(80, 80, 80, 0)
# time.sleep(1)
# bot.set_motor(0, 0, 0, 0)

# print(bot.get_motor_encoder())
# time.sleep(2)
# print(bot.get_motor_encoder())
# time.sleep(2)

# # 程序结束后请删除对象，避免在其他程序中使用Rosmaster库造成冲突
# # After the program is complete, delete the object to avoid conflicts caused by using the library in other programs
# del bot
