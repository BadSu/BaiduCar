#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File    :   sound_test.py
@Time    :   2022/11/29 17:31:54
@Author  :   Jerry Law 
@Version :   1.0
@Company :   Guangzhou High Genius Dynamics Co. Ltd.
@Describe:   喇叭测试程序。
'''

# here put the import lib
from pygame import mixer  # Load the popular external library
from time import sleep

mixer.init()
mixer.music.load('data/夏天的风-温岚.mp3')

mixer.music.play()
sleep(10)       # 播放10秒钟后停止
mixer.music.stop()
sleep(1)
