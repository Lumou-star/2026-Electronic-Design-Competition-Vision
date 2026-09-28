# Find Rects Example
#
# This example shows off how to find rectangles in the image using the quad threshold
# detection code from our April Tags code. The quad threshold detection algorithm
# detects rectangles in an extremely robust way and is much better than Hough
# Transform based methods. For example, it can still detect rectangles even when lens
# distortion causes those rectangles to look bent. Rounded rectangles are no problem!
# (But, given this the code will also detect small radius circles too)...

#x,y互换了位置使用屏幕时
#使用 cv_lite.rgb888_find_rectangles_with_corners（8.2）(cx=243,cy=158)
import time, os, gc, sys,math
from math import atan,sqrt,atan2,sqrt
from media.sensor import *
from media.display import *
from media.media import *
import cv_lite
from time import ticks_ms
from machine import UART
from machine import FPIOA
from machine import Pin
import ustruct

# 实例化FPIOA
fpioa = FPIOA()
# 设置PIN60为PWM通道0
fpioa.set_function(11, fpioa.UART2_TXD)
fpioa.set_function(12, fpioa.UART2_RXD)
fpioa.set_function(32, FPIOA.GPIO32)
fpioa.set_function(33, FPIOA.GPIO33)

INC_KEY = Pin(32, Pin.IN, Pin.PULL_UP)   # 增大
DEC_KEY = Pin(33, Pin.IN, Pin.PULL_UP)    # 减小

# UART2: baudrate 115200, 8bits, parity none, one stopbits
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

DETECT_WIDTH = ALIGN_UP(800, 16)
DETECT_HEIGHT = 480
lst=[[1,2],[1,2],[1,2],[1,2]]
sensor = None
#image_shape = [480,800]  # 高，宽
image_shape = [320,480]  # 高，宽
# -------------------------------
# 可调参数（建议调试时调整）/ Adjustable parameters (recommended for tuning)
# -------------------------------
canny_thresh1       = 30        # Canny 边缘检测低阈值 / Canny edge low threshold（降低以适应运动模糊）
canny_thresh2       = 120       # Canny 边缘检测高阈值 / Canny edge high threshold
approx_epsilon      = 0.025     # 多边形拟合精度（比例） / Polygon approximation precision (ratio) — 运动时适当放宽
area_min_ratio      = 0.001     # 最小面积比例（0~1） / Minimum area ratio (0~1)
max_angle_cos       = 0.7       # 最大角余弦（值越小越接近矩形） / Max cosine of angle — 放宽以容忍运动形变
gaussian_blur_size  = 3         # 高斯模糊核大小（奇数） / Gaussian blur kernel size (odd number)（减小以减少运动模糊）
length_threshold    = 180       # 对边长度差阈值（放宽以适应运动视角变化）
SPEED_SCALE         = 0.5       # 输出缩放系数（<1.0 减速，>1.0 加速，1.0 原速）
S_THRESHOLD         = 2000      # 面积阈值 / Area threshold
last=0
last_send = 0
dx = 0
dy = 0

# ---- 目标丢失保持与平滑 ----
last_valid_dx = 0          # 最后有效 dx
last_valid_dy = 0          # 最后有效 dy
lost_frames = 0             # 连续丢失帧计数
MAX_LOST_FRAMES = 8         # 丢失后保持最后有效值的帧数（约160ms）
ema_alpha = 0.4             # EMA 平滑系数（0~1，越小越平滑，越大响应越快）
ema_dx = None               # EMA 累积值（None 表示待初始化）
ema_dy = None
ema_init = False            # EMA 初始化标志

# ============================================================
# 脱机多参数调整系统 / Offline Multi-Parameter Tuning System
# ============================================================
# 参数配置列表：每个参数包含 name(显示名), key(变量名), min, max, step, decimals(显示小数位)
PARAM_CONFIG = [
    {"name": "S_THRESHOLD",       "min": 500,   "max": 10000, "step": 500,  "decimals": 0},
    {"name": "canny_thresh1",     "min": 5,     "max": 255,   "step": 5,    "decimals": 0},
    {"name": "canny_thresh2",     "min": 5,     "max": 255,   "step": 5,    "decimals": 0},
    {"name": "approx_epsilon",    "min": 0.005, "max": 0.200, "step": 0.005,"decimals": 3},
    {"name": "area_min_ratio",    "min": 0.0001,"max": 0.100, "step": 0.0005,"decimals": 4},
    {"name": "max_angle_cos",     "min": 0.30,  "max": 0.95,  "step": 0.05, "decimals": 2},
    {"name": "gaussian_blur_size","min": 1,     "max": 15,    "step": 2,    "decimals": 0},
    {"name": "length_threshold",  "min": 10,    "max": 300,   "step": 10,   "decimals": 0},
    {"name": "SPEED_SCALE",       "min": 0.1,   "max": 3.0,   "step": 0.1,  "decimals": 1},
]

param_index = 0            # 当前选中的参数索引
# 按键消抖与长按相关
last_key_time = 0
DEBOUNCE_DELAY = 200       # 短按消抖 200ms
LONG_PRESS_DELAY = 800     # 长按判定 800ms
inc_press_start = 0        # INC键按下时刻(0=未按下)
dec_press_start = 0        # DEC键按下时刻(0=未按下)
inc_long_triggered = False # 长按是否已触发（防重复）
dec_long_triggered = False
param_just_switched = False # 刚切换参数标志（用于显示提示）


def get_param_value(name):
    """根据参数名获取当前值"""
    global canny_thresh1, canny_thresh2, approx_epsilon, area_min_ratio
    global max_angle_cos, gaussian_blur_size, length_threshold, SPEED_SCALE, S_THRESHOLD
    mapping = {
        "canny_thresh1": canny_thresh1,
        "canny_thresh2": canny_thresh2,
        "approx_epsilon": approx_epsilon,
        "area_min_ratio": area_min_ratio,
        "max_angle_cos": max_angle_cos,
        "gaussian_blur_size": gaussian_blur_size,
        "length_threshold": length_threshold,
        "SPEED_SCALE": SPEED_SCALE,
        "S_THRESHOLD": S_THRESHOLD,
    }
    return mapping.get(name, 0)


def set_param_value(name, value):
    """根据参数名设置新值，并对特殊参数做约束（如高斯核必须为奇数）"""
    global canny_thresh1, canny_thresh2, approx_epsilon, area_min_ratio
    global max_angle_cos, gaussian_blur_size, length_threshold, SPEED_SCALE, S_THRESHOLD
    if name == "canny_thresh1":
        canny_thresh1 = int(value)
    elif name == "canny_thresh2":
        canny_thresh2 = int(value)
    elif name == "approx_epsilon":
        approx_epsilon = value
    elif name == "area_min_ratio":
        area_min_ratio = value
    elif name == "max_angle_cos":
        max_angle_cos = value
    elif name == "gaussian_blur_size":
        # 确保高斯核为奇数
        v = int(value)
        if v % 2 == 0:
            v = v + 1 if v < 15 else v - 1
        gaussian_blur_size = v
    elif name == "length_threshold":
        length_threshold = int(value)
    elif name == "SPEED_SCALE":
        SPEED_SCALE = value
    elif name == "S_THRESHOLD":
        S_THRESHOLD = int(value)


def handle_buttons(current_time):
    """
    处理按键输入：短按调整值，长按切换参数。
    返回 (param_changed: bool) 表示参数是否发生变化。
    """
    global param_index, last_key_time
    global inc_press_start, dec_press_start
    global inc_long_triggered, dec_long_triggered
    global param_just_switched
    global S_THRESHOLD  # 保留兼容

    param_changed = False
    cfg = PARAM_CONFIG[param_index]
    inc_raw = INC_KEY.value()  # 按下为0
    dec_raw = DEC_KEY.value()

    # ---- INC 键处理 ----
    if inc_raw == 0:  # 按下
        if inc_press_start == 0:
            inc_press_start = current_time  # 记录按下时刻
            inc_long_triggered = False
        press_duration = time.ticks_diff(current_time, inc_press_start)
        # 长按检测（超过800ms且未触发过）
        if press_duration >= LONG_PRESS_DELAY and not inc_long_triggered:
            inc_long_triggered = True
            # 切换到下一个参数
            param_index = (param_index + 1) % len(PARAM_CONFIG)
            param_just_switched = True
            param_changed = True
            print(f"切换到参数: {PARAM_CONFIG[param_index]['name']}")
    else:  # 松开
        if inc_press_start != 0:
            press_duration = time.ticks_diff(current_time, inc_press_start)
            if press_duration < LONG_PRESS_DELAY and not inc_long_triggered:
                # 短按：增加当前参数值
                if time.ticks_diff(current_time, last_key_time) > DEBOUNCE_DELAY:
                    last_key_time = current_time
                    name = cfg["name"]
                    cur = get_param_value(name)
                    new_val = min(cfg["max"], cur + cfg["step"])
                    # 对整数参数做四舍五入
                    if cfg["decimals"] == 0:
                        new_val = int(new_val)
                    else:
                        new_val = round(new_val, cfg["decimals"])
                    set_param_value(name, new_val)
                    param_changed = True
                    print(f"{name} 增加到: {new_val}")
        inc_press_start = 0
        inc_long_triggered = False

    # ---- DEC 键处理 ----
    if dec_raw == 0:  # 按下
        if dec_press_start == 0:
            dec_press_start = current_time
            dec_long_triggered = False
        press_duration = time.ticks_diff(current_time, dec_press_start)
        if press_duration >= LONG_PRESS_DELAY and not dec_long_triggered:
            dec_long_triggered = True
            # 切换到上一个参数
            param_index = (param_index - 1) % len(PARAM_CONFIG)
            param_just_switched = True
            param_changed = True
            print(f"切换到参数: {PARAM_CONFIG[param_index]['name']}")
    else:  # 松开
        if dec_press_start != 0:
            press_duration = time.ticks_diff(current_time, dec_press_start)
            if press_duration < LONG_PRESS_DELAY and not dec_long_triggered:
                # 短按：减少当前参数值
                if time.ticks_diff(current_time, last_key_time) > DEBOUNCE_DELAY:
                    last_key_time = current_time
                    name = cfg["name"]
                    cur = get_param_value(name)
                    new_val = max(cfg["min"], cur - cfg["step"])
                    if cfg["decimals"] == 0:
                        new_val = int(new_val)
                    else:
                        new_val = round(new_val, cfg["decimals"])
                    set_param_value(name, new_val)
                    param_changed = True
                    print(f"{name} 减少到: {new_val}")
        dec_press_start = 0
        dec_long_triggered = False

    return param_changed


def draw_param_osd(img):
    """
    在屏幕上绘制参数调整 OSD（On-Screen Display）。
    显示当前参数名、值、范围，以及操作提示。
    """
    global param_index, param_just_switched, S_THRESHOLD

    cfg = PARAM_CONFIG[param_index]
    name = cfg["name"]
    cur_val = get_param_value(name)
    fmt = f"{{:.{cfg['decimals']}f}}"
    val_str = fmt.format(cur_val)

    # 第1行：当前参数名和值
    line1 = f"[{param_index+1}/{len(PARAM_CONFIG)}] {name}: {val_str}"
    img.draw_string_advanced(5, 0, 25, line1, color=(0, 255, 255))

    # 第2行：范围提示
    min_str = fmt.format(cfg["min"])
    max_str = fmt.format(cfg["max"])
    line2 = f"Range:[{min_str}, {max_str}] Step:{fmt.format(cfg['step'])}"
    img.draw_string_advanced(5, 25, 20, line2, color=(200, 200, 200))

    # 第3行：操作提示（刚切换参数时高亮显示）
    if param_just_switched:
        tip_color = (255, 255, 0)  # 黄色高亮
    else:
        tip_color = (180, 180, 180)  # 灰色
    img.draw_string_advanced(5, 45, 18, "INC:+ DEC:- Long:Next", color=tip_color)

    # 切换提示定时清除（约1秒后恢复灰色）
    if param_just_switched:
        param_just_switched = False


def format_coord(coord):
    # 格式化为：符号位（+/-） + 3 位数值（补零）
    return f"{coord:+04d}"  # 如 -123 → "-123"，12 → "+012"
def split_to_2d(arr, cols=1):
    return [arr[i:i + cols] for i in range(0, len(arr), cols)]
def get_vertices(rect):
    x, y, w, h = rect
    # 计算四个顶点坐标
    top_left = (x, y)
    top_right = (x + w, y)
    bottom_right = (x + w, y + h)
    bottom_left = (x, y + h)
    return [top_left, top_right, bottom_right, bottom_left]
def find_max(arr):
    max_size=0
    max_blob=None
    for s in range (len(arr)):
        if arr[s][2]*arr[s][3] > max_size:
            max_blob=arr[s]
            max_size = arr[s][2]*arr[s][3]
    return max_blob

 # 定义一个函数来判断两条线段是否平行
def are_segments_parallel(theta1, theta2, tolerance=30):
    # 计算角度差
    angle_difference = abs(theta1 - theta2)
    if(angle_difference>180):angle_difference=angle_difference-180
    # 检查角度差是否为0度或180度（考虑浮点数精度）
    return math.isclose(angle_difference, 0, abs_tol=tolerance) or math.isclose(angle_difference, 180, abs_tol=tolerance)
def are_segments_vertical(theta1, theta2, tolerance=30):
    # 计算角度差
    angle_difference = abs(theta1 - theta2)
    if(angle_difference>180):angle_difference=angle_difference-180
    # 检查角度差是否为0度或180度（考虑浮点数精度）
    return math.isclose(angle_difference, 90, abs_tol=tolerance)
def find_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    def calculate_determinant(A, B):
        return A[0] * B[1] - A[1] * B[0]

    # 向量 AB 和 AC
    AB = (x2 - x1, y2 - y1)
    AC = (x3 - x1, y3 - y1)
    # 向量 CD
    CD = (x4 - x3, y4 - y3)

    # 计算叉积 determinant(AB, CD)
    det = calculate_determinant(AB, CD)

    if det == 0:
        # 直线平行或共线，没有唯一交点
        return None

    # 向量 DA
    DA = (x1 - x4, y1 - y4)

    # 计算参数 t 和 u
    t = calculate_determinant(AC, CD) / det
    u = calculate_determinant(AC, AB) / det

    # 计算交点坐标
    intersection_x = x1 + t * AB[0]
    intersection_y = y1 + t * AB[1]

    return int(intersection_x), int(intersection_y)

def fix_merged_corners(corners, rect, threshold=20):
    """
    检测并修复矩形角点合并问题。
    当任意两个角点距离小于阈值时，说明它们被错误合并，
    此时使用边界矩形 rect=[x,y,w,h] 重新生成四个角点。

    corners: [[x1,y1],[x2,y2],[x3,y3],[x4,y4]] — 检测到的4个角点
    rect: [x, y, w, h] — 边界矩形
    threshold: 角点合并判定距离阈值（像素），默认20
    返回: 修正后的4个角点
    """
    from math import sqrt

    # 检查任意两个角点是否距离过近（合并）
    for i in range(4):
        for j in range(i + 1, 4):
            dist = sqrt(
                (corners[i][0] - corners[j][0]) ** 2 +
                (corners[i][1] - corners[j][1]) ** 2
            )
            if dist < threshold:
                x, y, w, h = rect
                print(f"检测到角点{i}和{j}合并 (距离={dist:.1f}px)，使用边界矩形修正")
                # 用边界矩形重建四个角点：左上、右上、右下、左下
                return [
                    [x, y],
                    [x + w, y],
                    [x + w, y + h],
                    [x, y + h]
                ]
    return corners

#def are_segments_vertical(len1, len2, tolerance=0.2):
#        # 计算角度差
#        len_bili = 1en1/1en2
#        if(angle_difference>180):angle_difference=angle_difference-180
#        # 检查角度差是否为0度或180度（考虑浮点数精度）
#        return math.isclose(angle_difference, 90, abs_tol=tolerance)
def camera_init():
    global sensor

    # construct a Sensor object with default configure
    sensor = Sensor(id=2)
    # sensor reset
    sensor.reset()
    # set hmirror
    # sensor.set_hmirror(False)
    # sensor vflip
    # sensor.set_vflip(False)

    # set chn0 output size
#    sensor.set_framesize(width=DETECT_WIDTH,height=DETECT_HEIGHT)
    sensor.set_framesize(width=480, height=320)
    # set chn0 output format
    sensor.set_pixformat(Sensor.RGB888)
    # sensor.set_vflip(enable=True)
    # sensor.set_hmirror(enable=True)
    # ---- 关闭自动曝光以减少运动模糊 ----
    # 自动曝光在运动时拖影严重，改为手动固定短曝光
    try:
        sensor.set_auto_exposure(0)
        sensor.set_auto_gain(0)
    except:
        pass  # 部分固件不支持可忽略
    # use IDE as display output
    #使用时x，y坐标在UI上发生改变，真实坐标不变
    Display.init(Display.ST7701, width= DETECT_WIDTH, height = DETECT_HEIGHT,fps=100,to_ide = True)

#    Display.init(Display.VIRT, width= DETECT_WIDTH, height = DETECT_HEIGHT,fps=100,to_ide = True)

    # init media manager
    MediaManager.init()
    # sensor start run
    sensor.run()

def camera_deinit():
    global sensor
    # sensor stop run
    sensor.stop()
    # deinit display
    Display.deinit()
    # sleep
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # release media buffer
    MediaManager.deinit()

def capture_picture():#UI显示上x和y对调
    global sensor, last_send, dx, dy, last_valid_dx, last_valid_dy, lost_frames, ema_dx, ema_dy, ema_init
    global canny_thresh1, canny_thresh2, approx_epsilon, area_min_ratio
    global max_angle_cos, gaussian_blur_size, length_threshold, SPEED_SCALE, S_THRESHOLD

    fps = time.clock()

    while True:
        fps.tick()
        rect_flag=0
        current_time = ticks_ms()

        try:
            os.exitpoint()
#            global S
            img = sensor.snapshot()

            # ---- 在图像中心(240, 160)画十字准星 ----
          #  CROSS_COLOR = (255, 255, 255)      # 白色
          #  CROSS_LEN = 10                     # 十字线半长
         #   img.draw_line(240 - CROSS_LEN, 160, 240 + CROSS_LEN, 160, color=CROSS_COLOR, thickness=2)  # 水平线
          #  img.draw_line(240, 160 - CROSS_LEN, 240, 160 + CROSS_LEN, color=CROSS_COLOR, thickness=2)  # 垂直线
         #   img.draw_circle(240, 160, 3, color=(0, 255, 0), fill=True, thickness=2)                    # 中心绿点

            # ---- 脱机按键处理（每帧调用）----
            handle_buttons(current_time)

            img_np = img.to_numpy_ref()  # 获取 RGB888 ndarray 引用 / Get RGB888 ndarray reference
            rects = cv_lite.rgb888_find_rectangles_with_corners(
                image_shape, img_np,
                canny_thresh1, canny_thresh2,
                approx_epsilon,
                area_min_ratio,
                max_angle_cos,
                gaussian_blur_size
            )

            print("rects",rects)
            if(rects):
                max_rects = find_max(rects)
                #print("max_rects",max_rects)
                c=[[1,2],[1,2],[1,2],[1,2]]

                for i in range(4):
                    c[i][0] = max_rects[2*i+4]
                    c[i][1] = max_rects[2*i+5]
                # 检测并修正合并的角点（上面两个角点被识别为一个点时自动修正）
                c = fix_merged_corners(c, [max_rects[0], max_rects[1], max_rects[2], max_rects[3]])
                for s in range (1,5):
                            if s == 4:
                                s=0
                            img.draw_line(c[s-1][0],c[s-1][1],c[s][0],c[s][1],color=(255, 0, 0),thickness=3)#img.draw_edges(c)等价 画矩形外框
                            img.draw_circle(c[s-1][0],c[s-1][1],2,color=(0, 0, 255),fill=True,thickness=3)
                len1=sqrt(pow(c[0][0]-c[1][0],2)+pow(c[0][1]-c[1][1],2))#0左上
                len2=sqrt(pow(c[2][0]-c[3][0],2)+pow(c[2][1]-c[3][1],2))#1左下
                len3=sqrt(pow(c[0][0]-c[3][0],2)+pow(c[0][1]-c[3][1],2))#2右下
                len4=sqrt(pow(c[1][0]-c[2][0],2)+pow(c[1][1]-c[2][1],2))#3右上

                #print("len1",len1)
                #print("len2",len2)
                #print("len3",len3)
                #print("len4",len4)

                S=max_rects[2]*max_rects[3]
                #print("S",S)

                err1=(abs(len1-len2))#对边差值
                err2=(abs(len3-len4))
#                print("err1",err1)
#                print("err2",err2)
                if(S>S_THRESHOLD and err1<length_threshold and err2<length_threshold and len1>30 and len2>30 and len3>30 and len4>30):
#                    bili=float(len1/len3)
#                    print("bili",bili)
                   # 计算每条线段的 theta):#
                    theta1 = math.atan2(c[0][1]-c[1][1], c[0][0]-c[1][0])
                    theta2 = math.atan2(c[2][1]-c[3][1], c[2][0]-c[3][0])
                    theta3 = math.atan2(c[0][1]-c[3][1], c[0][0]-c[3][0])
                    theta4 = math.atan2(c[1][1]-c[2][1], c[1][0]-c[2][0])

                    # 将弧度转换为角度
                    theta1_degrees = math.degrees(theta1)
                    theta2_degrees = math.degrees(theta2)
                    theta3_degrees = math.degrees(theta3)
                    theta4_degrees = math.degrees(theta4)
#                    print("theta1",theta1_degrees)
#                    print("theta2",theta2_degrees)
#                    print("theta3",theta3_degrees)
#                    print("theta4",theta4_degrees)

                    #线段是否平行和垂直
                    is_line1_line2_parallel = are_segments_parallel(theta1_degrees, theta2_degrees)
                    is_line3_line4_parallel = are_segments_parallel(theta3_degrees, theta4_degrees)
                    is_line1_line3_vertical = are_segments_vertical(theta1_degrees, theta3_degrees)

                    if (is_line1_line3_vertical and is_line1_line2_parallel and is_line3_line4_parallel  ):#标准矩形的条件
                          rect_flag=1
                          for s in range (4):#保存四个顶点
                              lst[s][0]=c[s][0]
                              lst[s][1]=c[s][1]
                    else: rect_flag=0
                else:rect_flag=0
            else:rect_flag=0
            if rect_flag==1:#识别出标准矩形
                intersection = find_intersection(lst[0][0], lst[0][1], lst[2][0], lst[2][1], lst[1][0], lst[1][1], lst[3][0], lst[3][1])
                img.draw_circle(intersection[0], intersection[1], 1, color=(255, 0, 0), thickness=4)
#                global last_dot
#                distance = math.sqrt((intersection[0] - last_dot[0]) ** 2 + (intersection[1] - last_dot[1]) ** 2)
                # 如果距离小于或等于阈值，则发送数据
#                print("(cx=%s,cy=%s)"%(last_dot[0], last_dot[1]))
                print("(cx=%s,cy=%s)"%(intersection[0], intersection[1]))
                img.draw_string_advanced(0,0,30,"(x=%s,y=%s)"%(intersection[0], intersection[1]),color=(255, 255, 0),scale=3)
#                if distance <= 100 or last_dot[0]==0 :#坐标变换合理时
#                    for s in range (1,5):
#                           if s == 4:
#                               s=0
#                           img.draw_line(c[s-1][0],c[s-1][1],c[s][0],c[s][1],color=(0, 255, 0),thickness=3)#img.draw_edges(c)等价 画矩形外框
#                           img.draw_circle(c[s-1][0],c[s-1][1],2,color=(0, 0, 255),fill=True,thickness=3)
#                    uart.write("(x=%s,y=%s)" % (intersection[0], intersection[1]))
#                    img.draw_string_advanced(0,0,50,"(x=%s,y=%s)"%(intersection[0], intersection[1]),color=(255, 255, 0),scale=3)
##                   print("(cx=%s,cy=%s)"%(intersection[0], intersection[1]))
#                    last_dot=intersection
                for s in range (1,5):
                       if s == 4:
                           s=0
                       img.draw_line(c[s-1][0],c[s-1][1],c[s][0],c[s][1],color=(0, 255, 0),thickness=3)#img.draw_edges(c)等价 画矩形外框
                       img.draw_circle(c[s-1][0],c[s-1][1],2,color=(0, 0, 255),fill=True,thickness=3)
                wucha_center = (240-intersection[0], 160-intersection[1])
                raw_dx = 240 - intersection[0]
                raw_dy = 160 - intersection[1]
                # EMA 平滑滤波
                if not ema_init:
                    ema_dx = raw_dx
                    ema_dy = raw_dy
                    ema_init = True
                else:
                    ema_dx = ema_alpha * raw_dx + (1 - ema_alpha) * ema_dx
                    ema_dy = ema_alpha * raw_dy + (1 - ema_alpha) * ema_dy
                dx = int(ema_dx)
                dy = int(ema_dy)
                # 保存最后有效值（丢失时保持用）
                last_valid_dx = dx
                last_valid_dy = dy
                lost_frames = 0
                #uart.write("(x=%s,y=%s)" % (intersection[0], intersection[1]))
                img.draw_string_advanced(200,0,30,"fps:%s"%int(fps.fps()),color=(0, 255, 0))
                # 绘制参数 OSD（识别成功时显示在底部）
                draw_param_osd(img)
                Display.show_image(img) # Take a picture and display the image.
            else:
                img.draw_string_advanced(20,50,30,"fps:%s"%int(fps.fps()),color=(0, 255, 0))
                # 绘制参数 OSD（未识别时也显示，方便调参）
                draw_param_osd(img)
                lost_frames += 1
                if lost_frames <= MAX_LOST_FRAMES:
                    # 短暂丢失：保持最后有效值，避免 PID 跳变
                    dx = last_valid_dx
                    dy = last_valid_dy
                    img.draw_string_advanced(10, 80, 20, "HOLD:%d"%lost_frames, color=(0, 255, 255))
                else:
                    # 长时间丢失：归零
                    dx = 0
                    dy = 0
                Display.show_image(img)
            # ---- 串口2发送偏差数据包（节流：每50ms最多发一次）----
            # 格式: 0x01 0x02 0x03 | dx(s16 LE) | dy(s16 LE) | 0xFE  共8字节
            now = time.ticks_ms()
            if now - last_send >= 20 and rect_flag == 1:
                last_send = now
                # 输出缩放：SPEED_SCALE < 1.0 可减慢电机速度
                tx_dx = int(dx * SPEED_SCALE)
                tx_dy = int(dy * SPEED_SCALE)
                packet = ustruct.pack("<BBBhhB", 0x01, 0x02, 0x03, tx_dx, tx_dy, 0xFE)
                uart.write(packet)
#            print("fps",fps.fps())

        except KeyboardInterrupt as e:
            print("user stop: ", e)
            break
        except BaseException as e:
            print(f"Exception {e}")
            break

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    try:
        print("camera init")
        camera_init()
        camera_is_init = True
        print("camera capture")
        capture_picture()
    except Exception as e:
        print(f"Exception {e}")
    finally:
        if camera_is_init:
            print("camera deinit")
            camera_deinit()

if __name__ == "__main__":
    main()
