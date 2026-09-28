# 导入所需模块
import time, os, gc
from machine import TOUCH
from media.sensor import *
from media.display import *
from media.media import *

'''颜色阈值读取'''
f = open("/data/color_threshold.txt", 'r')

rect_threshold = f.readline()
start_index = rect_threshold.find('(') + 1
end_index = rect_threshold.find(')')
num_str = rect_threshold[start_index:end_index]
rect_threshold = tuple(map(int, num_str.split(',')))

laser_threshold = f.readline()
start_index = laser_threshold.find('(') + 1
end_index = laser_threshold.find(')')
num_str = laser_threshold[start_index:end_index]
laser_threshold = tuple(map(int, num_str.split(',')))

K230_laser_point = f.readline()
start_index = K230_laser_point.find('(') + 1
end_index = K230_laser_point.find(')')
num_str = K230_laser_point[start_index:end_index]
K230_laser_point = tuple(map(int, num_str.split(',')))

print(rect_threshold)
print(laser_threshold)
print(K230_laser_point)
f.close()

# 显示模式选择：可以是 "VIRT"、"LCD" 或 "HDMI"
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

def display_test():
    print(f"显示测试，当前模式为 {DISPLAY_MODE}")
    if DISPLAY_MODE == "VIRT":
        Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, osd_num=4, fps=60)
    elif DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, osd_num=4, to_ide=True)
    elif DISPLAY_MODE == "HDMI":
        Display.init(Display.LT9611, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, osd_num=4, to_ide=True)
    MediaManager.init()

'''变量声明区'''
sensor = None
clock = time.clock()
sersor_width = 400
sersor_height = 304
color_threshold_choose_flag = 0  # 0-rect_threshold  1-laser_threshold  2-K230_laser_point
lab_choose_flag = 0              # 0-L- 1-L+ 2-A- 3-A+ 4-B- 5-B+
K230_laser_point_index = 0       # 0-X 1-Y

try:
    # 实例化 TOUCH 设备 0
    tp = TOUCH(0)

    # 摄像头配置
    sensor = Sensor(width=sersor_width, height=sersor_height)
    sensor.reset()
    sensor.set_framesize(width=sersor_width, height=sersor_height)
    sensor.set_pixformat(Sensor.RGB565)
    sensor.set_vflip(enable=True)
    sensor.set_hmirror(enable=True)

    # 媒体初始化
    display_test()

    # 启动 sensor
    sensor.run()
    while True:
        clock.tick()
        os.exitpoint()

        # 抓取通道0的图像
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img = img.gaussian(1)

        # 获取最多 5 个触摸点数据
        p = tp.read(5)

        # 触摸阈值调节
        if p != ():
            for idx, point in enumerate(p, start=1):
                if 25 < point.x < 125 and 326 < point.y < 370:  # L+
                    lab_choose_flag = 1
                elif 150 < point.x < 250 and 326 < point.y < 370:  # A+
                    lab_choose_flag = 3
                elif 275 < point.x < 375 and 326 < point.y < 370:  # B+
                    lab_choose_flag = 5
                elif 25 < point.x < 125 and 414 < point.y < 458:  # L-
                    lab_choose_flag = 0
                elif 150 < point.x < 250 and 414 < point.y < 458:  # A-
                    lab_choose_flag = 2
                elif 275 < point.x < 375 and 414 < point.y < 458:  # B-
                    lab_choose_flag = 4
                elif 425 < point.x < 525 and 326 < point.y < 370:  # X
                    K230_laser_point_index = 0
                elif 425 < point.x < 525 and 414 < point.y < 458:  # Y
                    K230_laser_point_index = 1

                if 550 < point.x < 650 and 326 < point.y < 370:  # 加
                    if color_threshold_choose_flag == 0 and lab_choose_flag <= 1:
                        if rect_threshold[lab_choose_flag] >= 255:
                            lst = list(rect_threshold)
                            lst[lab_choose_flag] = 255
                            rect_threshold = tuple(lst)
                        else:
                            lst = list(rect_threshold)
                            lst[lab_choose_flag] += 1
                            rect_threshold = tuple(lst)
                    if color_threshold_choose_flag == 1 and lab_choose_flag <= 1:
                        if laser_threshold[lab_choose_flag] >= 255:
                            lst = list(laser_threshold)
                            lst[lab_choose_flag] = 255
                            laser_threshold = tuple(lst)
                        else:
                            lst = list(laser_threshold)
                            lst[lab_choose_flag] += 1
                            laser_threshold = tuple(lst)
                    if color_threshold_choose_flag == 2:
                        if K230_laser_point[K230_laser_point_index] >= 800:
                            lst = list(K230_laser_point)
                            lst[K230_laser_point_index] = 800
                            K230_laser_point = tuple(lst)
                        else:
                            lst = list(K230_laser_point)
                            lst[K230_laser_point_index] += 1
                            K230_laser_point = tuple(lst)

                elif 675 < point.x < 775 and 326 < point.y < 370:  # 保存
                    f = open("/data/color_threshold.txt", 'w')
                    f.write('rect_threshold=' + str(rect_threshold) + '\n')
                    f.write('laser_threshold=' + str(laser_threshold) + '\n')
                    f.write('K230_laser_point=' + str(K230_laser_point) + '\n')
                    f.close()

                elif 550 < point.x < 650 and 414 < point.y < 458:  # 减
                    if color_threshold_choose_flag == 0 and lab_choose_flag <= 1:
                        if rect_threshold[lab_choose_flag] <= 0:
                            lst = list(rect_threshold)
                            lst[lab_choose_flag] = 0
                            rect_threshold = tuple(lst)
                        else:
                            lst = list(rect_threshold)
                            lst[lab_choose_flag] -= 1
                            rect_threshold = tuple(lst)
                    if color_threshold_choose_flag == 1 and lab_choose_flag <= 1:
                        if laser_threshold[lab_choose_flag] <= 0:
                            lst = list(laser_threshold)
                            lst[lab_choose_flag] = 0
                            laser_threshold = tuple(lst)
                        else:
                            lst = list(laser_threshold)
                            lst[lab_choose_flag] -= 1
                            laser_threshold = tuple(lst)
                    if color_threshold_choose_flag == 2:
                        if K230_laser_point[K230_laser_point_index] <= 0:
                            lst = list(K230_laser_point)
                            lst[K230_laser_point_index] = 0
                            K230_laser_point = tuple(lst)
                        else:
                            lst = list(K230_laser_point)
                            lst[K230_laser_point_index] -= 1
                            K230_laser_point = tuple(lst)

                elif 675 < point.x < 775 and 414 < point.y < 458:  # 切换
                    color_threshold_choose_flag = (color_threshold_choose_flag + 1) % 3

        # 根据当前选择的阈值类型生成二值化预览图
        if color_threshold_choose_flag == 0:
            img_binary = img.to_grayscale(copy=True)
            img_binary.binary([rect_threshold])
        elif color_threshold_choose_flag == 1:
            img_binary = img.to_grayscale(copy=True)
            img_binary.binary([laser_threshold])
        elif color_threshold_choose_flag == 2:
            img_binary = img.copy()

        img.draw_string_advanced(10, 0, 30, "fps: {}".format(clock.fps() // 1), (0, 0, 0))
        img.compressed_for_ide()

        # 按键图层绘制 - 左侧（L/A/B通道选择）
        img_button1 = img.copy()
        img_button1.clear()
        img_button2 = img_button1.copy()

        img_button1.draw_rectangle(25, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(150, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(275, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(25, 238, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(150, 238, 100, 44, color=(255, 255, 255), thickness=2)
        img_button1.draw_rectangle(275, 238, 100, 44, color=(255, 255, 255), thickness=2)

        img_button1.draw_string_advanced(55, 150, 30, "L+", color=(255, 255, 255))
        img_button1.draw_string_advanced(180, 150, 30, "A+", color=(255, 255, 255))
        img_button1.draw_string_advanced(305, 150, 30, "B+", color=(255, 255, 255))
        img_button1.draw_string_advanced(55, 238, 30, "L-", color=(255, 255, 255))
        img_button1.draw_string_advanced(180, 238, 30, "A-", color=(255, 255, 255))
        img_button1.draw_string_advanced(305, 238, 30, "B-", color=(255, 255, 255))

        if color_threshold_choose_flag == 0:
            img_button1.draw_string_advanced(55, 200, 20, str(rect_threshold), color=(255, 255, 255))
        elif color_threshold_choose_flag == 1:
            img_button1.draw_string_advanced(55, 200, 20, str(laser_threshold), color=(255, 255, 255))
        img_button1.draw_string_advanced(350, 200, 20, str(lab_choose_flag), color=(255, 255, 255))

        # 按键图层绘制 - 右侧（操作按钮）
        img_button2.draw_rectangle(25, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button2.draw_rectangle(150, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button2.draw_rectangle(275, 150, 100, 44, color=(255, 255, 255), thickness=2)
        img_button2.draw_rectangle(25, 238, 100, 44, color=(255, 255, 255), thickness=2)
        img_button2.draw_rectangle(150, 238, 100, 44, color=(255, 255, 255), thickness=2)
        img_button2.draw_rectangle(275, 238, 100, 44, color=(255, 255, 255), thickness=2)

        img_button2.draw_string_advanced(55, 150, 30, "X", color=(255, 255, 255))
        img_button2.draw_string_advanced(180, 150, 30, "加", color=(255, 255, 255))
        img_button2.draw_string_advanced(305, 150, 30, "保存", color=(255, 255, 255))
        img_button2.draw_string_advanced(55, 238, 30, "Y", color=(255, 255, 255))
        img_button2.draw_string_advanced(180, 238, 30, "减", color=(255, 255, 255))
        img_button2.draw_string_advanced(305, 238, 30, "切换", color=(255, 255, 255))

        img_button2.draw_string_advanced(350, 200, 20, str(color_threshold_choose_flag), color=(255, 255, 255))

        if color_threshold_choose_flag == 2:
            img_binary.draw_cross(K230_laser_point[0], K230_laser_point[1], color=(255, 0, 0), size=10, thickness=1)

        img_button2.draw_string_advanced(55, 200, 20, str(K230_laser_point_index), color=(255, 255, 255))

        # 显示到LCD屏
        Display.show_image(img_button1, x=0, y=176, layer=Display.LAYER_OSD0)
        Display.show_image(img_button2, x=400, y=176, layer=Display.LAYER_OSD1)
        Display.show_image(img, layer=Display.LAYER_OSD2)
        Display.show_image(img_binary, x=(DISPLAY_WIDTH) // 2, layer=Display.LAYER_OSD3)

        img_button1 = None
        img_button2 = None
        img = None
        img_binary = None

        gc.collect()

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
