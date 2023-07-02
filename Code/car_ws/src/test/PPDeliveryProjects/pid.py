#!/usr/bin/env python
# -*- coding:utf-8 -*-
from math import *
from numpy import sign

class PID:
    '''位置PID'''
    def __init__(self, P, I, D, outhigh, outlow, dt):
        self.Kp = P                         #PID增益比例
        self.Ki = I                         #PID增益积分
        self.Kd = D                         #PID增益微分
        self.out_high = outhigh             #输出上限
        self.out_low = outlow               #输出下限
        self.cycle = dt                     #PID周期

        self.first_call = 0                 #首次调用值

        self.setpoint = 0.0                 #PID设定值
        self.output = 0.0                   #输出

        self.PTerm = 0.0                    #P计算结果
        self.ITerm = 0.0                    #I计算结果
        self.DTerm = 0.0                    #D计算结果

        self.process_variable_last = 0.0    #过程变量上一次值
        self.pid_error_lasr = 0.0           #误差上一次值
        self.integral_error_last = 0.0      #积分误差上一次值
        
    def PID_reset(self):
        self.first_call = 0                 #首次调用值

        self.setpoint = 0.0                 #PID设定值
        self.output = 0.0                   #输出

        self.PTerm = 0.0                    #P计算结果
        self.ITerm = 0.0                    #I计算结果
        self.DTerm = 0.0                    #D计算结果

        self.process_variable_last = 0.0    #过程变量上一次值
        self.pid_error_lasr = 0.0           #误差上一次值
        self.integral_error_last = 0.0      #积分误差上一次值

    def update(self, process_variable):
        '''判断首次使用'''
        flag = bool(0)
        if self.first_call == 0:
            flag = bool(1)
            self.first_call = 1
        else:
            flag = bool(0)
        '''计算误差'''
        PIDerror = self.setpoint - process_variable
        '''计算P'''
        self.PTerm = PIDerror * self.Kp
        '''计算I'''
        if self.Ki != 0.0:
            if flag:
                self.ITerm = 0.0
            else:
                self.ITerm = (((PIDerror + self.pid_error_lasr) * 0.5) * (self.Kp / self.Ki) * (0.0167 * self.cycle)) + self.integral_error_last
        '''计算D'''
        if self.Kd != 0.0:
            if flag:
                self.DTerm = 0.0
            else:
                self.DTerm = ((self.Kp * self.Kd) * ((process_variable - self.process_variable_last)*-1.0)) / (0.0167 * self.cycle)
        '''保存上一次值'''
        self.process_variable_last = process_variable
        self.pid_error_lasr = PIDerror
        pisum = self.PTerm + self.ITerm
        if pisum < self.out_high or pisum > self.out_low:
            self.integral_error_last = self.ITerm
        else:
            if pisum > self.out_high:
                self.integral_error_last = self.out_high - self.PTerm
            elif pisum < self.out_low:
                self.integral_error_last = self.out_low - self.PTerm
        '''输出结果'''
        self.output = self.PTerm + self.ITerm + self.DTerm
        if self.output >= self.out_high:
            self.output = self.out_high
        elif self.output <= self.out_low:
            self.output = self.out_low
        return self.output

class PID_LeadLag:
    '''PID超前滞后'''
    def __init__(self, P, lag, lead, outhigh, outlow, dt):
        self.Kp = P                         #PID增益比例
        self.Klag = lag                     #PID滞后时间
        self.Klead = lead                   #PID超前时间
        self.out_high = outhigh             #输出上限
        self.out_low = outlow               #输出下限
        self.cycle = dt                     #PID周期

        self.first_call = 0                 #首次调用值

        self.output = 0.0

        self.input_last = 0.0               #输入上一次值
        self.output_lasr = 0.0              #输出上一次值

    def update(self, pidinput):
        '''判断首次使用'''
        flag = bool(0)
        if self.first_call == 0:
            flag = bool(1)
            self.first_call = 1
        else:
            flag = bool(0)
        '''计算'''
        if flag:
            self.output = pidinput
        else:
            self.output = ((self.cycle * pidinput * self.Kp) + ((pidinput - self.input_last) * self.Kp * self.Klead) + (self.Klag * self.output_lasr)) / (self.cycle + self.Klag)
        '''保存上一次值'''
        self.input_last = pidinput
        if flag:
            self.output_lasr = pidinput
        else:
            self.output_lasr = self.output
        '''输出结果'''
        if self.output >= self.out_high:
            self.output = self.out_high
        elif self.output <= self.out_low:
            self.output = self.out_low
        return self.output

class PID_Output_Rate_Limiter:
    '''PID输出速率限制'''
    def __init__(self, init_out, EGU, dt):
        self.output_rate = EGU              #输出速率
        self.initial_output = init_out      #初始输出
        self.cycle = dt                     #周期

        self.first_call = 0                 #首次调用值

        self.output = 0.0

        self.initial_output_result = 0.0    #初始输出结果
        self.rate_input_result = 0.0        #输入结果
        self.egu_resul = 0.0                #EGU计算结果
        self.initial_rate_input_result = 0.0#输入与初始输出 差
        
        self.output_lasr = 0.0              #输出上一次值
        
    def PID_reset(self):
        self.first_call = 0                 #首次调用值

        self.output = 0.0

        self.initial_output_result = 0.0    #初始输出结果
        self.rate_input_result = 0.0        #输入结果
        self.egu_resul = 0.0                #EGU计算结果
        self.initial_rate_input_result = 0.0#输入与初始输出 差
        
        self.output_lasr = 0.0              #输出上一次值
        
    def update(self, rate_input):
        '''判断首次使用'''
        flag = bool(0)
        if self.first_call == 0:
            flag = bool(1)
            self.first_call = 1
        else:
            flag = bool(0)

        if flag:
            self.initial_output_result = self.initial_output
        else:
            self.initial_output_result = self.output_lasr

        if flag:
            self.rate_input_result = self.initial_output_result
        else:
            self.rate_input_result = rate_input

        self.egu_resul = abs((self.output_rate / 60) * self.cycle)
        self.initial_rate_input_result = self.rate_input_result - self.initial_output_result

        if self.egu_resul >= abs(self.initial_rate_input_result):
            self.output = self.rate_input_result
        else:
            self.output = (sign(self.initial_rate_input_result) * self.egu_resul) + self.initial_output_result

        self.output_lasr = self.output

        return self.output

class PID_Control_Input_Filter:
    '''PID滤波'''
    def __init__(self):
        self.forward_coefficients = [0.144426, 0.235649, 0.239850, 0.235649, 0.144426]

        self.first_call = 0                 #首次调用值

        self.output = 0.0

        self.out_result = 0.0

        self.forward_coefficients_lasr = [0.0, 0.0, 0.0, 0.0, 0.0] 

    def update(self, Control_input):
        '''判断首次使用'''
        flag = bool(0)
        if self.first_call == 0:
            flag = bool(1)
            self.first_call = 1
        else:
            flag = bool(0)
        
        if flag:
            self.output = Control_input
            self.forward_coefficients_lasr = [Control_input] * 5
        else:
            lasr_read = self.forward_coefficients_lasr
            self.forward_coefficients_lasr = [Control_input, lasr_read[0], lasr_read[1], lasr_read[2], lasr_read[3]]
            
            list_product = [a*b for a,b in zip(self.forward_coefficients_lasr, self.forward_coefficients)]
            for i in range(0, len(list_product)):
                if i == 5:exit()
                self.out_result += list_product[i]

            self.output = self.out_result
        
        return self.output

class speed_up:
    '''
    缓起
    speed_up 缓起时间 ms
    dt 周期
    '''
    def __init__(self, speed_up_time, dt):
        self.up_time = speed_up_time / 1000
        self.cycle = dt
        self.cycle_accumulation = 0.0
        self.output = 0.0

    def PID_reset(self):
        self.cycle_accumulation = 0.0
        self.output = 0.0
    
    def update(self):
        '''out 0~1'''
        self.cycle_accumulation += self.cycle       #累加时间
        
        if self.cycle_accumulation < self.up_time:
            self.output = 1.0 - ((self.up_time - self.cycle_accumulation)/self.up_time)
        else:
            self.output = 1.0
        return self.output