# 立创·庐山派-K230-CanMV开发板资料与相关扩展板软硬件资料官网全部开源
# 开发板官网：www.lckfb.com
# 技术支持常驻论坛，任何技术问题欢迎随时交流学习
# 立创论坛：www.jlc-bbs.com/lckfb
# 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
# 不靠卖板赚钱，以培养中国工程师为己任

import time, os, sys
import image

from media.sensor import *
from media.display import *
from media.media import *

# 图像采集分辨率
picture_width  = 480
picture_height = 240

sensor_id = 2
sensor = None

# ========== 红色激光笔阈值（LAB色彩空间）==========
threshold_red = [(76, 98, -48, 11, -12, 23),(80, 100, -128, 127, -128, 127)]


def find_max(blobs):
    """寻找面积最大的色块"""
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[4] > max_size:
            max_blob = blob
            max_size = blob[4]
    return max_blob

# 显示模式
DISPLAY_MODE = "LCD"

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

try:
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    # 镜像翻转，如要翻转则取消注释
    # sensor.set_hmirror(True)
    # sensor.set_vflip(True)

    # 单通道，240×180
    sensor.set_framesize(width=picture_width, height=picture_height, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # 判断显示通道
    if DISPLAY_MODE == "VIRT":
        Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, fps=60)
    elif DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
    elif DISPLAY_MODE == "HDMI":
        Display.init(Display.LT9611, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)

    MediaManager.init()
    sensor.run()

    # 创建屏幕大小的显示缓冲区
    disp_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.RGB565)

    # 计算缩放比例和居中偏移
    scale = max(DISPLAY_WIDTH / picture_width, DISPLAY_HEIGHT / picture_height)
    scaled_w = int(picture_width * scale)
    scaled_h = int(picture_height * scale)
    offset_x = (DISPLAY_WIDTH - scaled_w) // 2
    offset_y = (DISPLAY_HEIGHT - scaled_h) // 2

    while True:
        os.exitpoint()

        # 捕捉一帧图像
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        # 采集的原始图像进行 Gamma 校正
        img.gamma_corr(1.3)

        # 寻找红色激光笔
        blobs = img.find_blobs(threshold_red,  merge=True)
        if blobs:
            max_blob = find_max(blobs)
            if max_blob:
                cx = max_blob[5]   # blob 中心 x
                cy = max_blob[6]   # blob 中心 y
                w, h = max_blob[2], max_blob[3]
                # 用十字箭头框选
                img.draw_cross(cx, cy, color=(0, 255, 0), size=6, thickness=1)
                #img.draw_rectangle(max_blob[0], max_blob[1], w, h, color=(0, 255, 0), thickness=1)


        # 将摄像头图像按比例放大后居中绘制到缓冲区
        disp_img.clear()
        disp_img.draw_image(img, offset_x, offset_y, x_scale=scale, y_scale=scale)
        Display.show_image(disp_img)

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
