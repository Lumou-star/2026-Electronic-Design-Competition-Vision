import time, os, sys

from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin         # 导入pin模块
from machine import FPIOA       # 导入fpioa

sensor_id = 2           # 选择使用哪个摄像头，有0，1，2三个，默认是2
sensor = None

# 拍照间隔（毫秒）和保存文件夹
CAPTURE_INTERVAL = 250
SAVE_FOLDER = "/sdcard/captured_images"

# 要保存的照片大小
PHOTO_W = 800
PHOTO_H = 480

try:
    # ====== 1. 按键初始化（必须放在sensor和display之前！） ======
    fpioa = FPIOA()                             # 实例化fpioa
    fpioa.set_function(53, FPIOA.GPIO33)        # 设置引脚功能
    KEY = Pin(53, Pin.IN, Pin.PULL_DOWN)        # 下拉模式：按下=1，松开=0

    # ====== 2. 摄像头初始化 ======
    # 构造一个具有默认配置的摄像头对象
    sensor = Sensor(id=sensor_id)
    # 重置摄像头sensor
    sensor.reset()

    # 无需进行镜像翻转
    # 设置水平镜像
    sensor.set_hmirror(False)
    # 设置垂直翻转
    sensor.set_vflip(False)

    # 设置通道0的输出尺寸为800*480（显示用）
    sensor.set_framesize(chn=CAM_CHN_ID_0, width=800, height=480)
    # 设置通道0的输出像素格式为RGB565
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # ====== 3. 显示初始化（会重新配置自己需要的引脚，不干扰按键） ======
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    # 初始化媒体管理器
    MediaManager.init()
    # 启动传感器
    sensor.run()

    # 创建保存图片的文件夹
    try:
        os.mkdir(SAVE_FOLDER)
        print(f"已创建文件夹: {SAVE_FOLDER}")
    except OSError:
        print(f"文件夹已存在: {SAVE_FOLDER}")

    # 计算320x320裁剪区域（从800x480画面正中间裁出正方形）
    crop_x = (800 - PHOTO_W) // 2   # 240
    crop_y = (480 - PHOTO_H) // 2   # 80

    # 采集状态变量
    capturing = False           # 是否正在采集
    last_capture_time = 0       # 上一次拍照的时间
    photo_count = 0             # 已拍照片数量
    key_released = True         # 按键是否已释放（防止重复触发）

    print("=" * 30)
    print(f"保存路径: {SAVE_FOLDER}")
    print(f"照片尺寸: {PHOTO_W}x{PHOTO_H}")
    print(f"拍照间隔: {CAPTURE_INTERVAL} ms")
    print("按一次按键开始采集，再按一次停止采集")
    print("=" * 30)

    clock = time.clock()            # 创建一个对象
    clock.reset()                   # 先重置一遍计时器
    while True:
        clock.tick()                # 开始计时
        os.exitpoint()

        # 捕获通道0的图像
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # 按键检测：按一次开始采集，再按一次停止（下拉模式：按下=1）
        if KEY.value() == 1:                        # 按键按下
            if key_released:                        # 一次按下只触发一次
                key_released = False
                time.sleep_ms(20)                   # 消抖延时
                if KEY.value() == 1:                # 再次确认
                    capturing = not capturing       # 切换采集状态
                    if capturing:
                        # 找到已有文件的最大编号，接着往后排，避免覆盖
                        max_idx = -1
                        try:
                            for f in os.listdir(SAVE_FOLDER):
                                if f.endswith(".jpg") and len(f) == 8:  # 0000.jpg
                                    try:
                                        idx = int(f[:4])
                                        if idx > max_idx:
                                            max_idx = idx
                                    except ValueError:
                                        pass
                        except OSError:
                            pass
                        photo_count = max_idx + 1
                        last_capture_time = time.ticks_ms()
                        print(f">>> 开始采集数据...（从 {photo_count:04d}.jpg 继续）")
                    else:
                        print(f">>> 停止采集，共保存 {photo_count} 张图片")
        else:
            key_released = True                     # 按键释放，允许下次触发

        # 定时拍照存储
        if capturing:
            now = time.ticks_ms()
            if time.ticks_diff(now, last_capture_time) >= CAPTURE_INTERVAL:
                # 从画面正中间裁出320*320的正方形
                crop_img = img.copy(roi=(crop_x, crop_y, PHOTO_W, PHOTO_H))
                filepath = f"{SAVE_FOLDER}/{photo_count:04d}.jpg"
                try:
                    crop_img.save(filepath)
                    print(f"[{photo_count:04d}] 已保存: {filepath}")
                except Exception as e:
                    print(f"[{photo_count:04d}] 保存失败: {e}")
                photo_count += 1
                last_capture_time = now

        # 在画面左上角显示采集状态
        if capturing:
            img.draw_string(10, 10, f"REC {photo_count}", color=(255, 0, 0), scale=2)
        else:
            img.draw_string(10, 10, "STOP", color=(0, 255, 0), scale=2)

        # 显示捕获的图像
        Display.show_image(img)



        print("fps:{}".format(clock.fps()))         # 打印帧率
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
