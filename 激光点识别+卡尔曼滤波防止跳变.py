# 立创·庐山派-K230-CanMV开发板资料与相关扩展板软硬件资料官网全部开源
# 开发板官网：www.lckfb.com
# 技术支持常驻论坛，任何技术问题欢迎随时交流学习
# 立创论坛：www.jlc-bbs.com/lckfb
# 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
# 不靠卖板赚钱，以培养中国工程师为己任

import time, os, sys

from media.sensor import *
from media.display import *
from media.media import *

picture_width = 800
picture_height = 480

sensor_id = 2
sensor = None

# 红色激光笔阈值
threshold_red = [(69, 82, 0, 68, 5, 33),(72, 99, 9, 35, -20, 47)]


# ========== 卡尔曼滤波器（1D，位置+速度）+ 新息门控 ==========
class KalmanFilter1D:
    def __init__(self, R=30.0, Q=0.02, gate=4.0):
        # 状态: [position, velocity]
        self.x = [0.0, 0.0]
        # 协方差矩阵 P
        self.P = [[1.0, 0.0], [0.0, 1.0]]
        self.R = R          # 测量噪声（越大越平滑，推荐 20~60）
        self.Q = Q          # 过程噪声（越小越平滑，推荐 0.005~0.05）
        self.gate = gate    # 新息门控阈值（sigma 倍数，超过视为野值）
        self.initialized = False
        self.lost_frames = 0

    def update(self, z, dt=1.0):
        """输入测量值 z，返回滤波后的估计值；z=None 时纯预测"""
        if z is None:
            # 目标丢失，仅预测（匀速滑行）
            self.lost_frames += 1
            if self.lost_frames > 5:
                self.x[1] *= 0.5  # 丢太久就减速，防止漂飞
            self.x[0] = self.x[0] + self.x[1] * dt
            return self.x[0]

        if not self.initialized:
            self.x[0] = z
            self.initialized = True
            return z

        self.lost_frames = 0

        # ---- 预测步骤 ----
        # x = F @ x,  F = [[1, dt], [0, 1]]
        pos_pred = self.x[0] + self.x[1] * dt
        vel_pred = self.x[1]

        # P = F @ P @ F^T + Q  (P 对称: p01 = p10)
        p00, p01, p11 = self.P[0][0], self.P[0][1], self.P[1][1]
        p00_pred = p00 + 2 * dt * p01 + dt * dt * p11 + self.Q
        p01_pred = p01 + dt * p11
        p11_pred = p11 + self.Q

        # ---- 新息门控（outlier rejection）----
        # H = [1, 0], 只观测位置
        y = z - pos_pred               # 新息
        S = p00_pred + self.R           # 新息协方差
        sigma = S ** 0.5                # 标准差

        if abs(y) > self.gate * sigma:
            # 野值！跳过更新，仅预测
            self.x = [pos_pred, vel_pred]
            self.P[0][0] = p00_pred
            self.P[0][1] = p01_pred
            self.P[1][0] = p01_pred
            self.P[1][1] = p11_pred
            return pos_pred

        # ---- 更新步骤 ----
        K0 = p00_pred / S               # 位置增益
        K1 = p01_pred / S               # 速度增益

        pos = pos_pred + K0 * y
        vel = vel_pred + K1 * y

        # P = (I - K@H) @ P_pred
        self.P[0][0] = (1 - K0) * p00_pred
        self.P[0][1] = (1 - K0) * p01_pred
        self.P[1][0] = self.P[0][1]     # 保持对称
        self.P[1][1] = p11_pred - K1 * p01_pred

        self.x = [pos, vel]
        return pos


# 寻找最大色块函数
def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[4] > max_size:  # blob[4] 是像素数量
            max_blob = blob
            max_size = blob[4]
    return max_blob


blob_cx = 0
blob_cy = 0

# 卡尔曼滤波器（X 和 Y 各一个独立滤波）
# R（测量噪声）：越大越平滑但响应越慢，推荐 5~30
# Q（过程噪声）：越大越跟手但滤波越弱，推荐 0.01~0.2
kf_x = KalmanFilter1D(R=5, Q=0.01)
kf_y = KalmanFilter1D(R=5, Q=0.01)


# 显示模式选择：可以是 "VIRT"、"LCD" 或 "HDMI"
DISPLAY_MODE = "LCD"

# 根据模式设置显示宽高
if DISPLAY_MODE == "VIRT":
    # 虚拟显示器模式
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080
elif DISPLAY_MODE == "LCD":
    # 3.1寸屏幕模式
    DISPLAY_WIDTH = 800
    DISPLAY_HEIGHT = 480
elif DISPLAY_MODE == "HDMI":
    # HDMI扩展板模式
    DISPLAY_WIDTH = 1920
    DISPLAY_HEIGHT = 1080
else:
    raise ValueError("未知的 DISPLAY_MODE，请选择 'VIRT', 'LCD' 或 'HDMI'")

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

    # 设置通道0的输出尺寸为1920x1080
    sensor.set_framesize(width=picture_width, height=picture_height, chn=CAM_CHN_ID_0)
    # 设置通道0的输出像素格式为RGB565
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # 根据模式初始化显示器
    if DISPLAY_MODE == "VIRT":
        Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, fps=60)
    elif DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
    elif DISPLAY_MODE == "HDMI":
        Display.init(Display.LT9611, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)

    # 初始化媒体管理器
    MediaManager.init()
    # 启动传感器
    sensor.run()

    while True:
        os.exitpoint()

        # 捕获通道0的图像（彩色原图，用于最终显示）
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        
        blobs = img.find_blobs(threshold_red, x_stride=10, y_stride=10, merge=True)

        if blobs:
            max_blob = find_max(blobs)
            if max_blob:
                # blob结构: (x, y, w, h, pixels, cx, cy)
                raw_cx = max_blob[5]
                raw_cy = max_blob[6]
                # 卡尔曼滤波平滑
                blob_cx = int(kf_x.update(raw_cx))
                blob_cy = int(kf_y.update(raw_cy))

        # 在原图上画十字线
        img.draw_cross(blob_cx, blob_cy, color=(0, 255, 0), size=6, thickness=2)

        # 显示彩色原图，中心对齐
        Display.show_image(img, x=int((DISPLAY_WIDTH - picture_width) / 2), y=int((DISPLAY_HEIGHT - picture_height) / 2))

       

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