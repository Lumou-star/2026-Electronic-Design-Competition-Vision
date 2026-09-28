import time, os, sys

from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA
import ustruct  # 添加ustruct用于打包数据


sensor_id = 2           # 选择使用哪个摄像头，有0，1，2三个，默认是2
sensor = None

# 阈值
threshold = ([84, 26])

#定义一个名为roi1的列表，其中包含了2个元组。每个元组代表了一个矩形感兴趣区域在图像上的位置和大小。
roi1 =	 [
          (169,183, 160, 166),
          ( 374,163+50, 160, 166)
         ]

# 识别状态 0-不识别，1-识别
recognition_state = 1

try:
    # 构造一个具有默认配置的摄像头对象
    sensor = Sensor(id=sensor_id)
    # 重置摄像头sensor
    sensor.reset()

    # 无需进行镜像翻转
    # 设置水平镜像
    sensor.set_hmirror(False)
    # 设置垂直翻转
    sensor.set_vflip(False)

    # 设置通道0的输出尺寸为640*640
    sensor.set_framesize(chn=CAM_CHN_ID_0,width=800,height=480)
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

    uart = UART(UART.UART2, 115200)        # 实例化对象uart，后续直接调用

    clock = time.clock()            # 创建一个对象
    clock.reset()                   # 先重置一遍计时器
    while True:
        clock.tick()                # 开始计时
        os.exitpoint()
        # 捕获通道0的图像
        img = sensor.snapshot(chn=CAM_CHN_ID_0)


        # 串口接收逻辑
        if uart.any():          # 先检查是否有数据
            data = uart.read()  # 读取数据
            if data:  # 确保有数据
                if len(data) == 1:
                    byte_value = data[0]
                    if byte_value == 0xC2:      # 停止识别
                        recognition_state = 1
                        print("start")
                    elif byte_value == 0xC1:    # 开始识别
                        recognition_state = 0
                        print("end")
            # 调试：打印检测结果
            if blob1 and len(blob1) > 0:
                for i, b in enumerate(blob1):
                    img.draw_rectangle(b.rect(), color=(0, 255, 0), thickness=2)  # 绿色框
        if recognition_state:
            # 识别,通过串口发送逻辑
            flag = [0, 0]

            # 绘制ROI区域
            for i in roi1:
                img.draw_rectangle(i, color=(255, 0, 0), thickness=2)

            # 计算ROI面积
            roi_area1 = roi1[0][2] * roi1[0][3]  # 100 * 50 = 5000
            roi_area2 = roi1[1][2] * roi1[1][3]  # 100 * 50 = 5000

            # 检测
            blob1 = img.find_blobs([threshold], roi=roi1[0], merge=False, margin=0)
            blob2 = img.find_blobs([threshold], roi=roi1[1], merge=False, margin=0)



            # ROI1判断：只有检测到明显小于ROI的色块才认为是真实检测
            if blob1 and len(blob1) > 0:
                for b in blob1:
                    # 如果色块面积小于ROI面积的50%，认为是真实检测
                    if b.pixels() < roi_area1 * 0.5:
                        flag[0] = 1
                        img.draw_rectangle(b.rect(), color=(255, 255, 0), thickness=3)  # 黄色高亮真实检测
                        break

            # ROI2判断
            if blob2 and len(blob2) > 0:
                for b in blob2:
                    if b.pixels() < roi_area2 * 0.5:
                        flag[1] = 1
                        img.draw_rectangle(b.rect(), color=(255, 255, 0), thickness=3)
                        break

            send_data = ustruct.pack("<bbbb", 0xFF, flag[0], flag[1], 0xFE)
            uart.write(send_data)
#            print(f"Flags: {flag[0]} {flag[1]}")
        else:
            print("No")


        # 显示识别状态
        img.draw_string_advanced(20,64,32,f"识别：{recognition_state}",color=(0,0,255))
        # 显示两个位置是否有物体
        img.draw_string_advanced(20,32,32,f"{flag[0]},{flag[1]}",color=(0,255,0))
         # 打印帧率
        img.draw_string_advanced(20,0,32,f"fps:{int(clock.fps())}", color=(255, 0, 0))
        # 显示捕获的图像
        Display.show_image(img)

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
