import time, os, sys

from media.sensor import *
from media.display import *
from media.media import *

from machine import UART
from machine import FPIOA

sensor_id = 2           # 选择使用哪个摄像头，有0，1，2三个，默认是2
sensor = None

fpioa = FPIOA()
fpioa.set_function(11,FPIOA.UART2_TXD)
fpioa.set_function(12,FPIOA.UART2_RXD)

uart2 = UART(UART.UART2,115200)        # 定义一个变量名uart2，后续直接调用

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

    Tx_Data = [0xAA,0xAA,0x00,0x00,0x00,0x00,0xFF,0xFF]
    def Send_Proc(number,direction,steps):
        Tx_Data[2] = number
        Tx_Data[3] = direction
        Tx_Data[4] = steps
        Tx_Data[5] = sum(Tx_Data[2:6])  % 256
        uart2.write(bytes(Tx_Data))

#    变量初始化区域
    x_old,y_old=0,0
    c_x,c_y=100,100
    x,y=100,100
    while True:
        clock.tick()                # 开始计时
        os.exitpoint()

        img = sensor.snapshot(chn=CAM_CHN_ID_0)         # 从通道0捕获一帧图像
        img_gray = img.to_grayscale()                   # 将捕获的img图像转化为灰度图像
        img_gray = img_gray.binary([(71, 12)])         # 对灰度图像做二值化处理

        blobs = img.find_blobs([(79, 100, -53, 71, -3, 62)],False,(0,0,800,480),x_stride=2,y_stride=2,pixels_threshold=5,margin = True)
#        for blob in blobs:
#            img.draw_rectangle(blob.x(),blob.y(),blob.w(),blob.h(),color = (0,0,255),thickness = 2,fill = False)
        if blobs:
            blob = max(blobs, key= lambda b: b.pixels() )  # 取最大斑点
            x, y = blob.cx(), blob.cy()                    # 返回斑点中心点
            # 滤波抖动
            x = x_old if abs(x - x_old) < 5 else x
            y = y_old if abs(y - y_old) < 5 else y
            x_old, y_old = x, y
            img.draw_cross(x, y, color=(255,255,0),size=20,thickness=2)

        rects = img_gray.find_rects(threshold=8000)
        for i in rects:
            corner = i.corners()
            # 此函数返回嵌套列表[[x1,y1],[x2,y2],[x3,y3],[x4,y4]] 即矩形四个点坐标
            img.draw_line(corner[0][0],corner[0][1],corner[1][0],corner[1][1],color=(0,255,0),thickness=2)
            img.draw_line(corner[2][0],corner[2][1],corner[1][0],corner[1][1],color=(0,255,0),thickness=2)
            img.draw_line(corner[2][0],corner[2][1],corner[3][0],corner[3][1],color=(0,255,0),thickness=2)
            img.draw_line(corner[0][0],corner[0][1],corner[3][0],corner[3][1],color=(0,255,0),thickness=2)
            c_x = sum([corner[k][0] for k in range(4)])/4
            c_y = sum([corner[k][1] for k in range(4)])/4
        img.draw_cross(round(c_x), round(c_y), color=(255,0,0), size=20, thickness=2)   # 在矩形中心绘制十字
        img.draw_circle(round(c_x), round(c_y), 20, color=(255,0,0), thickness=2)       # 在矩形中心绘制圆

#        if x < c_x:                          # 激光点偏左
#            Send_Proc(0x00,0x00,0x01)        # 控制x方向的电机右转
#            time.sleep(0.01)
#        elif x > c_x:                        # 激光点偏右
#            Send_Proc(0x00,0x01,0x01)        # 控制x方向的电机左转
#            time.sleep(0.01)
#        if  y < c_y:                          # 激光笔偏上
#            Send_Proc(0x01,0x01,0x01)        # 控制y方向的电机下转
#            time.sleep(0.01)
#        elif y > c_y:                        # 激光笔偏下
#            Send_Proc(0x01,0x00,0x01)        # 控制y方向的电机上转
#            time.sleep(0.01)


        img = img.copy(roi=(96,24,224,224))       # 裁剪图像
        img.compressed_for_ide()                    # 将图片压缩到帧缓冲区中心
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

