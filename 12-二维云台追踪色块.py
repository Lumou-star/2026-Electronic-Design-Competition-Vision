import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA
import ustruct  # 添加ustruct用于打包数据

sensor_id = 2           # 选择使用哪个摄像头，有0，1，2三个，默认是2
sensor = None

# 颜色阈值
Threshold = [(23, 52, -47, -23, -128, 127)]


# 寻找最大色块函数
def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[4] > max_size:  # blob[4] 是像素数量
            max_blob = blob
            max_size = blob[4]
    return max_blob


# 定义画面的中心点坐标
img_center_x = 400
img_center_y = 240
# 定义色块的中心点坐标
blob_cx = 0
blob_cy = 0
# 定义物体和画面中心的偏差
delta_x = 0
delta_y = 0

# 定义x偏差的高8位和低8位
send_data1=0
send_data2=0
# 定义y偏差的高8位和低8位
send_data3=0
send_data4=0

try:
    # 构造一个具有默认配置的摄像头对象
    sensor = Sensor(id=sensor_id)
    # 重置摄像头sensor
    sensor.reset()

    # 镜像设置 - 根据实际情况调整
    sensor.set_hmirror(False)  # 水平镜像
    sensor.set_vflip(False)    # 垂直翻转

    # 设置通道0的输出尺寸为800 * 480
    sensor.set_framesize(chn=CAM_CHN_ID_0, width=800, height=480)
    # 设置通道0的输出像素格式为RGB565
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # 使用IDE的帧缓冲区作为显示输出
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    # 初始化媒体管理器
    MediaManager.init()
    # 启动传感器
    sensor.run()

    # 跳过一些帧让摄像头稳定
    for i in range(10):
        sensor.snapshot(chn=CAM_CHN_ID_0)
        time.sleep_ms(10)

    fpioa = FPIOA()            # 定义一个变量fpioa,后续直接使用
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)

    uart2 = UART(UART.UART2, 115200)        # 定义一个变量名uart2，后续直接调用

    clock = time.clock()            # 创建一个对象
    clock.reset()                   # 先重置一遍计时器

    while True:
        clock.tick()                # 开始计时
        os.exitpoint()

        # 捕获通道0的图像
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # 识别逻辑，数据处理逻辑
        blobs = img.find_blobs(Threshold,
                               inert=False,
                               roi=(0, 0, 800, 480),
                               x_stride=10,
                               y_stride=10,
                               area_threshold=100,
                               pixels_threshold=5000,
                               merge=True)
        # 识别到色块
        if blobs:
            max_blob = find_max(blobs)              # 选出最大色块
            if max_blob:
                # blob结构: (x, y, w, h, pixels, cx, cy)
                # 记录色块中心点坐标
                blob_cx = max_blob[5]          # 色块中心x坐标
                blob_cy = max_blob[6]          # 色块中心y坐标

                delta_x = blob_cx - img_center_x
                delta_y = blob_cy - img_center_y

                send_data1 = (delta_x >> 8) & 0xFF   # 高
                send_data2 = delta_x & 0xFF          # 低
                send_data3 = (delta_y >> 8) & 0xFF   # 高
                send_data4 = delta_y & 0xFF          # 低

                # 使用ustruct打包为6字节
                send_data = ustruct.pack("<BBBBBB",0xFF,int(send_data1),int(send_data2),
                                         int(send_data3),int(send_data4),0xFE)
                # 发送
                uart2.write(send_data)
                time.sleep_ms(10)
                # 在图像上绘制框和十字
                img.draw_rectangle(max_blob[0], max_blob[1],
                                 max_blob[2], max_blob[3],
                                 color=(0, 255, 0),thickness=5)
                img.draw_cross(blob_cx, blob_cy, color=(255, 255, 255),size=15,thickness=2)

                # 绘制中心点和方向标记
                img.draw_circle(img_center_x, img_center_y, 5, color=(255, 0, 0),thickness=10)

        # 显示捕获的图像
        Display.show_image(img)

#        # 打印帧率
#        print("fps: {:.2f}".format(clock.fps()))
except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    # 停止传感器运行
    if isinstance(sensor, Sensor):
        sensor.stop()
    uart2.deinit()
    # 反初始化显示模块
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # 释放媒体缓冲区
    MediaManager.deinit()
