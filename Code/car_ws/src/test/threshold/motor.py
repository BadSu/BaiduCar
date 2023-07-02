import time
from Rosmaster_Lib import Rosmaster

# 创建Rosmaster对象 bot Create the Rosmaster object bot
bot = Rosmaster()

# 启动接收数据，只能启动一次，所有读取数据的功能都是基于此方法\n",
# Start to receive data, can only start once, all read data function is based on this method\n",
bot.create_receive_threading()

bot.set_motor(50, 0, 0, 0)

time.sleep(3)

bot.set_motor(0, 0, 0, 0)

del bot
