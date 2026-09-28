import time, os, sys

from media.sensor import *
from media.display import *
from media.media import *

from machine import FPIOA
from machine import TOUCH

sensor_id = 2           # 选择使用哪个摄像头，有0，1，2三个，默认是2
sensor = None


touch = TOUCH(0)        # 实例化TOUCH设备0

try:
    # 构造一个具有默认配置的摄像头对象
    sensor = Sensor(id=sensor_id,width=640,height=480)

    # 重置摄像头sensor
    sensor.reset()

    # 无需进行镜像翻转
    # 设置水平镜像
    sensor.set_hmirror(False)
    # 设置垂直翻转
    sensor.set_vflip(False)

    # 设置通道0的输出尺寸为320*320
    sensor.set_framesize(chn=CAM_CHN_ID_0,width=320,height=320)
    # 设置通道0的输出像素格式为RGB565
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # 使用IDE的帧缓冲区作为显示输出
    Display.init(Display.ST7701, width=800,height=480, to_ide=True)
    # 初始化媒体管理器
    MediaManager.init()
    # 启动传感器
    sensor.run()

    clock = time.clock()            # 创建一个对象
    clock.reset()                   # 先重置一遍计时器



    while True:
        clock.tick()                # 开始计时
        os.exitpoint()

        img = sensor.snapshot(chn=CAM_CHN_ID_0)         # 从通道0捕获一帧图像
        img_gray = img.to_grayscale()                   # 将捕获的img图像转化为灰度图像
        img_gray = img_gray.binary([(61, 9)])         # 对灰度图像做二值化处理


        a = touch.read(5)

        if a != ():
            print("触摸数据: ")
            for index,point in enumerate(a,start=1):
                print("触摸点:{},X={},Y= {}".format(index,point.x,point.y))

        time.sleep(0.1)

#        img = img.copy(roi=(232,104,232,240))       # 裁剪图像
#        img.compressed_for_ide()                    # 将图片压缩到帧缓冲区中心
        # 显示捕获的图像
        Display.show_image(img)   #图像在屏幕上的显示位置
#        print("fps:{}".format(clock.fps()))         # 打印帧率

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    # 停止传感器运行
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 反初始化显示模块
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # 释放媒体缓冲区
    MediaManager.deinit()

