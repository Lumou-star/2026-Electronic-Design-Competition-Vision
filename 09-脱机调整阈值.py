import time, os, sys, gc
from media.sensor import *
from media.display import *
from media.media import *
from machine import TOUCH

# 显示模式选择
DISPLAY_MODE = "LCD"

# 根据模式设置显示宽高
if DISPLAY_MODE == "VIRT":
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080
elif DISPLAY_MODE == "LCD":
    DISPLAY_WIDTH = 800
    DISPLAY_HEIGHT = 480
elif DISPLAY_MODE == "HDMI":
    DISPLAY_WIDTH = 1920
    DISPLAY_HEIGHT = 1080
else:
    raise ValueError("未知的 DISPLAY_MODE，请选择 'VIRT', 'LCD' 或 'HDMI'")

# 显示设置函数
def display_test():
    print(f"显示测试，当前模式为 {DISPLAY_MODE}")

    # 根据模式初始化显示器
    if DISPLAY_MODE == "VIRT":
        Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, osd_num=4, fps=60)
    elif DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, osd_num=4, to_ide=True)
    elif DISPLAY_MODE == "HDMI":
        Display.init(Display.LT9611, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, osd_num=4, to_ide=True)

    # 初始化媒体管理器
    MediaManager.init()

# 函数：创建LAB二值化效果的图像
def create_lab_binary_effect(img, lab_threshold, bg_color=(0, 0, 0), fg_color=(255, 255, 255)):
    """
    创建LAB二值化效果的图像
    参数:
        img: 原始RGB565图像
        lab_threshold: (L_min, L_max, a_min, a_max, b_min, b_max)
        bg_color: 背景颜色 (不匹配的区域)
        fg_color: 前景颜色 (匹配的区域)
    """
    # 创建一个黑色背景的图像
    binary_effect = image.Image(img.width(), img.height(), image.RGB565)
    binary_effect.clear()

    # 设置默认背景色
    binary_effect.draw_rectangle(0, 0, img.width(), img.height(),
                                color=bg_color, fill=True)

    # 查找符合LAB阈值的色块
    blobs = img.find_blobs([lab_threshold], merge=True, margin=5, area_threshold=10)

    if blobs:
        for blob in blobs:
            # 在匹配的区域绘制前景色
            binary_effect.draw_rectangle(blob.x(), blob.y(), blob.w(), blob.h(),
                                        color=fg_color, fill=True)

    return binary_effect

# 定义变量区域
color_threshold_choose_flag = 0  # 0-矩形阈值, 1-激光阈值
lab_choose_flag = 0              # 0-L-, 1-L+, 2-A-, 3-A+, 4-B-, 5-B+

# LAB通道名称
lab_names = ["L-", "L+", "A-", "A+", "B-", "B+"]

# 初始化默认阈值
rect_threshold = (59, 246, -128, 127, -128, 127)
laser_threshold = (57, 100, -128, 127, -128, 127)

# 阈值范围
lab_ranges = {
    0: (0, 255),      # L- 范围
    1: (0, 255),      # L+ 范围
    2: (-128, 127),   # A- 范围
    3: (-128, 127),   # A+ 范围
    4: (-128, 127),   # B- 范围
    5: (-128, 127)    # B+ 范围
}

sensor = None
clock = time.clock()
sersor_width = 400
sersor_height = 304

try:
    # 实例化 TOUCH 设备
    tp = TOUCH(0)

    # 摄像头配置
    sensor = Sensor(width=sersor_width, height=sersor_height)
    sensor.reset()
    sensor.set_framesize(width=sersor_width, height=sersor_height)
    sensor.set_pixformat(Sensor.RGB565)
    sensor.set_vflip(enable=False)
    sensor.set_hmirror(enable=False)

    # 媒体初始化
    display_test()

    # 启动 sensor
    sensor.run()

    # 二值化显示模式
    binary_mode = 0  # 0:黑白, 1:彩色, 2:轮廓

    while True:
        clock.tick()
        os.exitpoint()

        # 抓取通道0的图像
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img = img.gaussian(1)

        # 获取触摸点数据
        p = tp.read(5)

        # 处理触摸输入
        if p != ():
            for idx, point in enumerate(p, start=1):
                x, y = point.x, point.y

                # 左侧按钮区域
                if 0 <= x < 400 and 176 <= y < 480:
                    local_y = y - 176

                    if 25 <= x <= 125 and 150 <= local_y <= 194:  # L+
                        lab_choose_flag = 1
                        print(f"选择 L+ 通道")
                    elif 150 <= x <= 250 and 150 <= local_y <= 194:  # A+
                        lab_choose_flag = 3
                        print(f"选择 A+ 通道")
                    elif 275 <= x <= 375 and 150 <= local_y <= 194:  # B+
                        lab_choose_flag = 5
                        print(f"选择 B+ 通道")
                    elif 25 <= x <= 125 and 238 <= local_y <= 282:  # L-
                        lab_choose_flag = 0
                        print(f"选择 L- 通道")
                    elif 150 <= x <= 250 and 238 <= local_y <= 282:  # A-
                        lab_choose_flag = 2
                        print(f"选择 A- 通道")
                    elif 275 <= x <= 375 and 238 <= local_y <= 282:  # B-
                        lab_choose_flag = 4
                        print(f"选择 B- 通道")

                # 右侧按钮区域
                elif 400 <= x < 800 and 176 <= y < 480:
                    local_x = x - 400
                    local_y = y - 176

                    if 150 <= local_x <= 250 and 150 <= local_y <= 194:  # 加
                        if color_threshold_choose_flag == 0:  # 矩形阈值
                            if 0 <= lab_choose_flag < 6:
                                lst = list(rect_threshold)
                                min_val, max_val = lab_ranges[lab_choose_flag]
                                if lst[lab_choose_flag] < max_val:
                                    lst[lab_choose_flag] += 1
                                    rect_threshold = tuple(lst)
                                    print(f"矩形阈值 {lab_names[lab_choose_flag]}+: {rect_threshold}")

                        elif color_threshold_choose_flag == 1:  # 激光阈值
                            if 0 <= lab_choose_flag < 6:
                                lst = list(laser_threshold)
                                min_val, max_val = lab_ranges[lab_choose_flag]
                                if lst[lab_choose_flag] < max_val:
                                    lst[lab_choose_flag] += 1
                                    laser_threshold = tuple(lst)
                                    print(f"激光阈值 {lab_names[lab_choose_flag]}+: {laser_threshold}")

                    elif 275 <= local_x <= 375 and 150 <= local_y <= 194:  # 模式切换
                        binary_mode = (binary_mode + 1) % 3
                        mode_names = ["黑白", "彩色", "轮廓"]
                        print(f"二值化模式: {mode_names[binary_mode]}")

                    elif 150 <= local_x <= 250 and 238 <= local_y <= 282:  # 减
                        if color_threshold_choose_flag == 0:  # 矩形阈值
                            if 0 <= lab_choose_flag < 6:
                                lst = list(rect_threshold)
                                min_val, max_val = lab_ranges[lab_choose_flag]
                                if lst[lab_choose_flag] > min_val:
                                    lst[lab_choose_flag] -= 1
                                    rect_threshold = tuple(lst)
                                    print(f"矩形阈值 {lab_names[lab_choose_flag]}-: {rect_threshold}")

                        elif color_threshold_choose_flag == 1:  # 激光阈值
                            if 0 <= lab_choose_flag < 6:
                                lst = list(laser_threshold)
                                min_val, max_val = lab_ranges[lab_choose_flag]
                                if lst[lab_choose_flag] > min_val:
                                    lst[lab_choose_flag] -= 1
                                    laser_threshold = tuple(lst)
                                    print(f"激光阈值 {lab_names[lab_choose_flag]}-: {laser_threshold}")

                    elif 275 <= local_x <= 375 and 238 <= local_y <= 282:  # 切换
                        color_threshold_choose_flag = (color_threshold_choose_flag + 1) % 2
                        mode_names = ["矩形阈值", "激光阈值"]
                        print(f"切换到: {mode_names[color_threshold_choose_flag]}")

        # 根据当前模式创建处理后的图像
        if color_threshold_choose_flag == 0:  # 矩形阈值
            current_threshold = rect_threshold
            if binary_mode == 0:  # 黑白模式
                img_processed = create_lab_binary_effect(img, current_threshold,
                                                        bg_color=(0, 0, 0),
                                                        fg_color=(255, 255, 255))
            elif binary_mode == 1:  # 彩色模式
                img_processed = create_lab_binary_effect(img, current_threshold,
                                                        bg_color=(0, 0, 0),
                                                        fg_color=(0, 255, 0))  # 绿色
            else:  # 轮廓模式
                img_processed = img.copy()
                blobs = img.find_blobs([current_threshold])
                if blobs:
                    for blob in blobs:
                        img_processed.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)

        elif color_threshold_choose_flag == 1:  # 激光阈值
            current_threshold = laser_threshold
            if binary_mode == 0:  # 黑白模式
                img_processed = create_lab_binary_effect(img, current_threshold,
                                                        bg_color=(0, 0, 0),
                                                        fg_color=(255, 255, 255))
            elif binary_mode == 1:  # 彩色模式
                img_processed = create_lab_binary_effect(img, current_threshold,
                                                        bg_color=(0, 0, 0),
                                                        fg_color=(255, 0, 0))  # 红色
            else:  # 轮廓模式
                img_processed = img.copy()
                blobs = img.find_blobs([current_threshold])
                if blobs:
                    for blob in blobs:
                        img_processed.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)

        # 显示FPS
        img.draw_string_advanced(10, 0, 30, "fps: {}".format(clock.fps() // 1), (0, 0, 0))

        # 按键图层绘制
        img_button1 = img.copy()
        img_button1.clear()
        img_button2 = img_button1.copy()

        # 左侧按钮区域
        button_colors = []
        for i in range(6):
            if i == lab_choose_flag:
                if i in [0, 1]:
                    button_colors.append((255, 0, 0))
                elif i in [2, 3]:
                    button_colors.append((0, 255, 0))
                else:
                    button_colors.append((0, 0, 255))
            else:
                button_colors.append((255, 255, 255))

        img_button1.draw_rectangle(25, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(150, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(275, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(25, 238, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(150, 238, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(275, 238, 100, 44, color=(255, 255, 255), thickness=2)

        img_button1.draw_string_advanced(55, 150, 30, "L+", color=button_colors[1])
        img_button1.draw_string_advanced(180, 150, 30, "A+", color=button_colors[3])
        img_button1.draw_string_advanced(305, 150, 30, "B+", color=button_colors[5])
        img_button1.draw_string_advanced(55, 238, 30, "L-", color=button_colors[0])
        img_button1.draw_string_advanced(180, 238, 30, "A-", color=button_colors[2])
        img_button1.draw_string_advanced(305, 238, 30, "B-", color=button_colors[4])

        # 显示当前阈值
        if color_threshold_choose_flag == 0:
            threshold_text = f"L:[{rect_threshold[0]},{rect_threshold[1]}] A:[{rect_threshold[2]},{rect_threshold[3]}] B:[{rect_threshold[4]},{rect_threshold[5]}]"
            img_button1.draw_string_advanced(55, 200, 12, threshold_text, color=(255, 255, 255))
        elif color_threshold_choose_flag == 1:
            threshold_text = f"L:[{laser_threshold[0]},{laser_threshold[1]}] A:[{laser_threshold[2]},{laser_threshold[3]}] B:[{laser_threshold[4]},{laser_threshold[5]}]"
            img_button1.draw_string_advanced(55, 200, 12, threshold_text, color=(255, 255, 255))

        img_button1.draw_string_advanced(350, 200, 20, f"选择:{lab_names[lab_choose_flag]}", color=(255, 255, 255))

        # 右侧按钮区域 - 简化布局
        img_button2.draw_rectangle(150, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button2.draw_rectangle(275, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button2.draw_rectangle(150, 238, 100, 44, color=(255, 255, 255), thickness=2)
        img_button2.draw_rectangle(275, 238, 100, 44, color=(255, 255, 255), thickness=2)

        # 绘制右侧按钮文字
        img_button2.draw_string_advanced(180, 150, 30, "加", color=(255, 255, 255))
        img_button2.draw_string_advanced(305, 150, 30, "模式", color=(255, 255, 255))
        img_button2.draw_string_advanced(180, 238, 30, "减", color=(255, 255, 255))
        img_button2.draw_string_advanced(305, 238, 30, "切换", color=(255, 255, 255))

        # 显示当前模式
        mode_names = ["矩形", "激光"]
        display_names = ["黑白", "彩色", "轮廓"]
        img_button2.draw_string_advanced(350, 200, 20, f"主:{mode_names[color_threshold_choose_flag]}", color=(255, 255, 255))
        img_button2.draw_string_advanced(350, 220, 15, f"显:{display_names[binary_mode]}", color=(255, 255, 255))

        # 压缩图像
        img.compressed_for_ide()
        img_processed.compressed_for_ide()

        # 显示到LCD屏
        Display.show_image(img_button1, x=0, y=176, layer=Display.LAYER_OSD0)
        Display.show_image(img_button2, x=400, y=176, layer=Display.LAYER_OSD1)
        Display.show_image(img, layer=Display.LAYER_OSD2)
        Display.show_image(img_processed, x=(DISPLAY_WIDTH) // 2, layer=Display.LAYER_OSD3)

        # 释放内存
        img_button1 = None
        img_button2 = None
        img = None
        img_processed = None
        gc.collect()

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
    import traceback
    traceback.print_exc()
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
