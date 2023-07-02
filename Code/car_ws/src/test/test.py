#!/usr/bin/env python3
#coding=utf-8
import time
from Rosmaster_Lib import Rosmaster
# from ipywidgets import interact
# import ipywidgets as widgets

# 创建Rosmaster对象 bot  Create the Rosmaster object bot
bot = Rosmaster()
# print(bot.help)

# 启动接收数据，只能启动一次，所有读取数据的功能都是基于此方法
# Start to receive data, can only start once, all read data function is based on this method
bot.create_receive_threading()

# 控制电机运动 Control motor movement
def run_motor(M1, M2, M3, M4):
    bot.set_motor(M1, M2, M3, M4)
    return M1, M2, M3, M4


bot.set_motor(80, 80, 80, 0)
time.sleep(1)
bot.set_motor(0, 0, 0, 0)

print(bot.get_motor_encoder())
time.sleep(2)
print(bot.get_motor_encoder())
time.sleep(2)

# 程序结束后请删除对象，避免在其他程序中使用Rosmaster库造成冲突
# After the program is complete, delete the object to avoid conflicts caused by using the library in other programs
del bot