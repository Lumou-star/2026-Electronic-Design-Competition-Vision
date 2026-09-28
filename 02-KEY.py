from machine import Pin         # 导入pin模块
from machine import FPIOA       # 导入fpioa
import time                     # 导入整个时间包

fpioa = FPIOA()                        # 实例化fpioa
fpioa.set_function(53,FPIOA.GPIO33)    # 设置引脚功能

KEY = Pin(53,Pin.IN,Pin.PULL_DOWN)     # 实例化KEY引脚53,配置为输入，下拉

while True:
    if KEY.value() == 1:
        time.sleep(0.02)                # 检测到按键按下，延迟20ms
        if KEY.value() == 1:            # 再次确认按键按下
            print("按下了")
            while KEY.value() == 1:     # 等待按键释放
                pass
    else:
        print("没有按下")
    time.sleep(0.01)            # 主循环延时，降低CPU占用
