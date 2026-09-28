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
Threshold = [(28, 59, 1, 26, -60, -33),  # 蓝色
             (36, 62, -56, -37, -128, 127),  # 绿色
             (25, 72, 22, 63, -7, 64)]     # 红色

# 定义识别哪种颜色 0-蓝色,1-绿色,2-红色
color_index = 0
color_names = ["Blue", "Green", "Red"]


# 寻找最大色块函数
def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[4] > max_size:  # blob[4] 是像素数量
            max_blob = blob
            max_size = blob[4]
    return max_blob

def map_value(value, in_min, in_max, out_min, out_max):
    """
    将value从输入范围[in_min, in_max]映射到输出范围[out_min, out_max]
    公式: out = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    """
    # 确保值在输入范围内
    value = max(in_min, min(in_max, value))

    # 线性映射
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# 画面的中心点坐标
img_center_x = 400
img_center_y = 240
# 定义色块的中心点坐标
blob_cx = 0
blob_cy = 0
# 色块位置和中心位置的差值
delta_x=0
delta_y=0

# 定义舵机角度
servo_angle_x = 0
servo_angle_y = 0

data = 0

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

    # 用于颜色切换的变量
    current_threshold = Threshold[color_index]

    while True:
        clock.tick()                # 开始计时
        os.exitpoint()

        # 捕获通道0的图像
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # 串口接收部分，更改识别颜色
        # 修正1: 正确的串口接收方式
        if uart2.any():          # 先检查是否有数据
            data = uart2.read()  # 读取数据

            if data:  # 确保有数据
               # 如果是单字节数据
                if len(data) == 1:
                    byte_value = data[0]

                    if byte_value == 0xC1:
                        color_index = 0
                        current_threshold = Threshold[0]
                        print("切换到蓝色")
                    elif byte_value == 0xC2:
                        color_index = 1
                        current_threshold = Threshold[1]
                        print("切换到绿色")
                    elif byte_value == 0xC3:
                        color_index = 2
                        current_threshold = Threshold[2]
                        print("切换到红色")
        # 识别逻辑，数据处理逻辑
        blobs = img.find_blobs([current_threshold],
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


                # x方向
                # 0 ------- 180
                delta_x = blob_cx - img_center_x             # 计算x的差值
                servo_angle_x = map_value(delta_x, -400, 400, 180, 0)

                # y方向
                delta_y = blob_cy - img_center_y             # 计算y的差值
                servo_angle_y = map_value(delta_y, -240, 240, 50, 130)

                # 使用ustruct打包为4字节
                send_data = ustruct.pack("<bbbb", 0xFF, int(servo_angle_x), int(servo_angle_y), 0xFE)
                # 发送
                uart2.write(send_data)
                print(servo_angle_x,servo_angle_y)


                # 在图像上绘制框和十字
                img.draw_rectangle(max_blob[0], max_blob[1],
                                 max_blob[2], max_blob[3],
                                 color=(0, 255, 0),thickness=5)
                img.draw_cross(blob_cx, blob_cy, color=(255, 255, 255),size=15,thickness=2)

                # 绘制中心点和方向标记
                img.draw_circle(img_center_x, img_center_y, 5, color=(255, 0, 0),thickness=10)



        img.draw_string_advanced(0,0,32,f"color:{color_names[color_index]}", color=(255, 0, 0))
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
