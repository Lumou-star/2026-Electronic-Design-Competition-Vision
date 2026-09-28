import time
import os
import sys
import math
from media.sensor import *
from media.display import *
from media.media import *
from time import ticks_ms
from machine import FPIOA
from machine import Pin
from machine import Timer
from machine import UART

sensor = None
blue = 90, 100, -11, 10, -60, 43
black = (0,95)

# 物理世界矩形尺寸（厘米）
RECT_WIDTH_CM = 27.6
RECT_HEIGHT_CM = 19.1
CIRCLE_RADIUS_CM = 6.0
6
# 目标点坐标 (160, 116) - 图像中心
TARGET_POINT = (161, 115)

# 全局状态变量
current_base_point_index = 0  # 当前识别的圆形基准点编号
base_point_counter = 0        # 满足条件帧数计数
detect_counter = 0            # 矩形识别计数器
lost_counter = 0              # 矩形丢失计数器
min_detect_frames = 3         # 连续检测阈值（从5改为2）
min_lost_frames = 6           # 连续丢失阈值（从5改为2）
flag_detected = False         # 滤波后的检测状态

def vector_angle_diff(v1, v2):
    """计算两个向量之间的角度差（单位：度）"""
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    det = v1[0]*v2[1] - v1[1]*v2[0]
    angle = math.atan2(det, dot) * (180 / math.pi)
    return abs(angle)

def get_line_intersection(line1, line2):
    """计算两条直线的交点"""
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    # 计算第一条直线的参数：A1*x + B1*y = C1
    A1 = y2 - y1
    B1 = x1 - x2
    C1 = A1 * x1 + B1 * y1

    # 计算第二条直线的参数：A2*x + B2*y = C2
    A2 = y4 - y3
    B2 = x3 - x4
    C2 = A2 * x3 + B2 * y3

    # 计算行列式
    det = A1 * B2 - A2 * B1

    if det == 0:  # 直线平行
        # 使用备用方法计算中点
        return ((x1 + x3) / 2, (y1 + y3) / 2)
    else:
        # 计算交点
        x = (B2 * C1 - B1 * C2) / det
        y = (A1 * C2 - A2 * C1) / det
        return (x, y)

def calculate_perspective_circle(center, corners, radius_cm):
    """
    计算考虑透视失真的圆点
    center: 矩形中心点 (x, y)
    corners: 矩形的四个角点 [左上, 右上, 右下, 左下]
    radius_cm: 圆的物理半径（厘米）
    返回：(圆点列表, 平均半径)
    """
    # 计算矩形在图像中的宽度和高度（像素）
    width_px = math.sqrt((corners[1][0] - corners[0][0])**2 + (corners[1][1] - corners[0][1])**2)
    height_px = math.sqrt((corners[2][0] - corners[1][0])**2 + (corners[2][1] - corners[1][1])**2)

    # 计算像素到厘米的转换比例
    px_per_cm_x = width_px / RECT_WIDTH_CM
    px_per_cm_y = height_px / RECT_HEIGHT_CM

    # 计算圆的半径（像素），考虑透视变形
    radius_x = radius_cm * px_per_cm_x
    radius_y = radius_cm * px_per_cm_y
    avg_radius = (radius_x + radius_y) / 2  # 平均半径用于阈值计算

    # 生成圆上的点（考虑透视变形）
    circle_points = []
    num_points = 16  # 修改为16个点

    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        # 计算圆点在矩形坐标系中的位置
        x = center[0] + radius_x * math.cos(angle)
        y = center[1] + radius_y * math.sin(angle)
        circle_points.append((int(x), int(y)))

    return circle_points, avg_radius

def sort_corners(corners, center):
    """
    优化角点排序逻辑，确保排序结果更准确
    返回：[左上, 右上, 右下, 左下]
    """
    # 改进1: 使用角度排序前，先找出最上边两个点
    top_points = []
    bottom_points = []

    # 计算所有点的y值
    y_values = [p[1] for p in corners]
    median_y = sum(y_values) / len(y_values)

    # 分为上下两组点
    for p in corners:
        if p[1] < median_y:
            top_points.append(p)
        else:
            bottom_points.append(p)

    # 确保上下两组各有两个点
    if len(top_points) != 2 or len(bottom_points) != 2:
        # 如果分组失败，使用旧方法作为备选
        angles = []
        for point in corners:
            dx = point[0] - center[0]
            dy = point[1] - center[1]
            angle = math.atan2(dy, dx)
            angles.append(angle)

        sorted_indices = sorted(range(len(angles)), key=lambda i: angles[i])
        sorted_corners = [corners[i] for i in sorted_indices]
        return sorted_corners

    # 排序上边的点：按x坐标从左到右
    top_points.sort(key=lambda p: p[0])

    # 排序下边的点：按x坐标从左到右
    bottom_points.sort(key=lambda p: p[0])

    # 左上角是上边左点
    top_left = top_points[0]
    top_right = top_points[1]

    # 左下角和右下角取决于矩形方向
    # 使用向量交叉法确定方向
    vec1 = (top_right[0] - top_left[0], top_right[1] - top_left[1])
    vec2 = (bottom_points[0][0] - top_left[0], bottom_points[0][1] - top_left[1])
    cross = vec1[0] * vec2[1] - vec1[1] * vec2[0]

    if cross > 0:  # 矩形顺时针方向
        bottom_left = bottom_points[0]
        bottom_right = bottom_points[1]
    else:  # 矩形逆时针方向
        bottom_left = bottom_points[1]
        bottom_right = bottom_points[0]

    return [top_left, top_right, bottom_right, bottom_left]

def sending_data(flag, sign_dx_center, dx_center, sign_dy_center, dy_center,
                 base_index, sign_dx_base, dx_base, sign_dy_base, dy_base):
    """
    串口数据包发送函数 (修改为10个参数)
    数据包格式:
        [0xAA][长度][1-flag][2-sign_dx_center][3-dx_center][4-sign_dy_center][5-dy_center]
        [6-base_index][7-sign_dx_base][8-dx_base][9-sign_dy_base][10-dy_base][校验位]
    校验位 = (前11字节数据的和)的低8位
    """
    global uart2

    # 数据包固定长度: 帧头(1) + 长度(1) + 10个数据 + 校验位(1) = 13字节
    # 长度字段: 表示从长度字节之后到校验位之前的字节数 (12字节)
    PACKET_LENGTH = 0x0D  # 11字节 (不包括帧头和长度字段)

    # 构建数据包的前部分 (帧头+长度+10个数据)
    packet = [
        0xAA,           # 帧头
        PACKET_LENGTH,  # 包长度
        flag,           # 标志位
        sign_dx_center, # x差符号
        dx_center,      # x差值
        sign_dy_center, # y差符号
        dy_center,      # y差值
        base_index,     # 基准点编号
        sign_dx_base,   # 基准点x差符号
        dx_base,        # 基准点x差值
        sign_dy_base,   # 基准点y差符号
        dy_base         # 基准点y差值
    ]

    # 计算校验位（前11个字节和的后8位/低8位）
    checksum = sum(packet) & 0xFF

    # 通过串口发送完整数据包 (13字节)
    uart2.write(bytes(packet) + bytes([checksum]))
    print(bytes(packet) + bytes([checksum]))

try:
    flag_key = 0
    print("camera_test")
    fpioa = FPIOA()
    fpioa.set_function(33, FPIOA.GPIO33)
    fpioa.set_function(53, FPIOA.GPIO53)
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    pin = Pin(33, Pin.OUT)
    pin.value(0)

    uart2 = UART(UART.UART2,115200)

    key = Pin(53, Pin.IN, Pin.PULL_DOWN)
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)
    sensor.set_pixformat(Sensor.RGB565)
    time.sleep(1)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # 用于存储上一帧的矩形信息
    prev_min_corners = None
    prev_center = None
    prev_circle_points = None
    prev_avg_radius = 0
    prev_has_rect = False  # 用于跟踪上一帧是否有检测到矩形

    while True:
        if key.value() == 1:
            while key.value() == 1:
                pass
            flag_key = (flag_key + 1) % 2
            time.sleep_ms(20)  # 释放后延时防止连按

        clock.tick()
        os.exitpoint()

        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img_binary = img.to_grayscale(copy=True)
        img_binary = img_binary.binary([black])
        img_binary.dilate()
        #img_binary.erode(1)
        rects = img_binary.find_rects(threshold=8000)

        # 初始化最小矩形变量
        min_rect = None
        min_area = float('inf')  # 初始化为无穷大
        min_corners = None
        min_black_ratio = 0
        survivors = []  # 幸存下来的矩形

        if rects is not None:
            for rect in rects:
                corners = rect.corners()
                # 确保四边形有4个顶点
                if len(corners) != 4:
                    continue

                # 计算所有内角误差
                angles = []
                max_angle_error = 0
                for i in range(4):
                    # 获取三个连续点 (前一个点-当前点-后一个点)
                    p0 = corners[(i-1) % 4]
                    p1 = corners[i]
                    p2 = corners[(i+1) % 4]

                    # 创建两个向量
                    vec1 = (p0[0]-p1[0], p0[1]-p1[1])
                    vec2 = (p2[0]-p1[0], p2[1]-p1[1])

                    # 计算角度差(理想应为180°-90°=90°差异)
                    angle_diff = vector_angle_diff(vec1, vec2)
                    angle_error = abs(angle_diff - 90)  # 计算与直角的偏差
                    angles.append(angle_error)
                    if angle_error > max_angle_error:
                        max_angle_error = angle_error

                # 计算平均偏差
                avg_angle_error = sum(angles) / len(angles)

                # 角度偏差检查 - 适当放宽条件提高灵敏度
                if max_angle_error > 45 or avg_angle_error > 30:
                    continue

                # 计算当前矩形的面积（使用像素数量）
                current_area = rect.w() * rect.h()
                if current_area < 5000:  # 确保矩形面积大于5000像素
                    continue

                # ========== 中心区域滤波 ==========
                # 计算矩形中心点
                center = get_line_intersection([corners[0], corners[2]], [corners[1], corners[3]])
                center_x, center_y = int(center[0]), int(center[1])

                # 计算中心区域大小（根据矩形大小动态调整）
                rect_width = max(5, min(15, int(math.sqrt(current_area) / 20)))
                check_size = max(5, min(20, rect_width))  # 控制大小在5-20像素之间
                half_size = check_size // 2

                # 计算检查区域边界
                x_start = max(center_x - half_size, 0)
                x_end = min(center_x + half_size, img.width() - 1)
                y_start = max(center_y - half_size, 0)
                y_end = min(center_y + half_size, img.height() - 1)

                # 检查区域内有效像素数量
                valid_pixels = 0
                total_pixels = 0

                # 遍历检查区域
                for y in range(y_start, y_end):
                    for x in range(x_start, x_end):
                        pixel_value = img_binary.get_pixel(x, y)
                        # 检查像素值是否为0（黑色）
                        if isinstance(pixel_value, tuple):
                            pixel_value = pixel_value[0]  # 如果是元组则取第一个元素
                        if pixel_value == 0:  # 黑色像素
                            valid_pixels += 1
                        total_pixels += 1

                # 计算黑色像素比例
                if total_pixels > 0:
                    black_ratio = valid_pixels / total_pixels
                else:
                    black_ratio = 0.0

                # 如果黑色像素不足50%，跳过该矩形
                if black_ratio < 0.25:
                    continue

                # 将该矩形加入幸存者列表
                survivors.append((rect, corners, black_ratio, max_angle_error, avg_angle_error))

                # 更新最小矩形（按面积排序）
                if current_area < min_area:
                    min_area = current_area
                    min_rect = rect
                    min_corners = corners
                    min_black_ratio = black_ratio

        # 从幸存者中选择最小面积矩形作为最终结果
        if len(survivors) > 0:
            min_area = float('inf')
            min_rect = None
            min_corners = None
            min_black_ratio = 0
            for rect, corners, black_ratio, max_angle_error, avg_angle_error in survivors:
                area = rect.w() * rect.h()
                if area < min_area:
                    min_area = area
                    min_rect = rect
                    min_corners = corners
                    min_black_ratio = black_ratio
        else:
            min_rect = None
            min_corners = None

        # 标记当前帧是否有检测到矩形
        current_has_rect = min_corners is not None

        # 如果没有检测到新的最小矩形，尝试使用上一帧的矩形
        if not current_has_rect and prev_min_corners is not None:
            min_corners = prev_min_corners
            use_prev = True
        else:
            use_prev = False

        # 检测标志滤波处理 - 优化为更灵敏的滤波
        if current_has_rect:
            # 新检测到矩形
            if not flag_detected:
                detect_counter += 1
                if detect_counter >= min_detect_frames:
                    flag_detected = True
                    detect_counter = 0
            else:
                lost_counter = 0  # 持续检测中重置丢失计数
        else:
            # 丢失矩形
            if flag_detected:
                lost_counter += 1
                if lost_counter >= min_lost_frames:
                    flag_detected = False
                    lost_counter = 0
            else:
                detect_counter = 0  # 持续丢失中重置检测计数

        # 初始化串口数据变量
        flag_byte = 0xBB if flag_detected else 0xCC
        base_index = current_base_point_index
        sign_dx_center, dx_center_val, sign_dy_center, dy_center_val = 0, 0, 0, 0
        sign_dx_base, dx_base_val, sign_dy_base, dy_base_val = 0, 0, 0, 0

        # 只处理最小矩形
        if min_corners is not None and flag_detected:
            # 如果是新的矩形（非上一帧的），更新缓存
            if not use_prev:
                prev_min_corners = min_corners

            # 绘制验证通过的矩形 (绿线) - 如果是当前帧检测到的
            if not use_prev:
                img.draw_line(min_corners[0][0], min_corners[0][1], min_corners[1][0], min_corners[1][1], color=(0, 255, 0), thickness=2)
                img.draw_line(min_corners[1][0], min_corners[1][1], min_corners[2][0], min_corners[2][1], color=(0, 255, 0), thickness=2)
                img.draw_line(min_corners[2][0], min_corners[2][1], min_corners[3][0], min_corners[3][1], color=(0, 255, 0), thickness=2)
                img.draw_line(min_corners[3][0], min_corners[3][1], min_corners[0][0], min_corners[0][1], color=(0, 255, 0), thickness=2)

            # ============ 使用对角线交点获取中心点 ============
            diagonal1 = [min_corners[0], min_corners[2]]
            diagonal2 = [min_corners[1], min_corners[3]]
            center = get_line_intersection(diagonal1, diagonal2)
            center_x, center_y = int(center[0]), int(center[1])

            # 在中心位置绘制黄色圆圈 (半径2像素)
            img.draw_circle(center_x, center_y, 2, color=(255, 255, 0), thickness=1)

            # ============ 绘制考虑透视失真的圆 ============
            # 对矩形角点进行排序（左上、右上、右下、左下）
            sorted_corners = sort_corners(min_corners, (center_x, center_y))

            # 决定是否重新计算圆点
            recalc_circle = False

            # 仅在矩形为新检测或状态变化时重新计算
            if not use_prev or (not prev_has_rect and current_has_rect):
                circle_points, avg_radius = calculate_perspective_circle(
                    (center_x, center_y),
                    sorted_corners,
                    CIRCLE_RADIUS_CM
                )
                prev_circle_points = circle_points
                prev_avg_radius = avg_radius
                prev_center = (center_x, center_y)
                recalc_circle = True

            # 使用缓存的圆点（如果可用）
            if prev_circle_points:
                # 动态计算阈值 (平均半径的1/8，最小为5像素)
                threshold_val = max(5, int(prev_avg_radius / 8))

                # 绘制拟合圆（绿色连线）
                if recalc_circle:
                    for i in range(len(prev_circle_points)):
                        start_point = prev_circle_points[i]
                        end_point = prev_circle_points[(i+1) % len(prev_circle_points)]
                        img.draw_line(start_point[0], start_point[1], end_point[0], end_point[1], color=(0, 255, 0), thickness=2)

                # 绘制基准点
                for i, point in enumerate(prev_circle_points):
                    x, y = point
                    if i == current_base_point_index:
                        # 当前基准点为红色
                        img.draw_circle(x, y, 3, color=(255, 0, 0), thickness=1)  # 实心圆
                    else:
                        # 其他点为黄色
                        img.draw_circle(x, y, 3, color=(255, 255, 0), thickness=1)  # 实心圆

                # 获取当前基准点
                current_point = prev_circle_points[current_base_point_index]
                cp_x, cp_y = current_point

                # 计算当前基准点与目标点的差值
                dx = TARGET_POINT[0] - cp_x
                dy = TARGET_POINT[1] - cp_y

                # 计算绝对值
                abs_dx = abs(dx)
                abs_dy = abs(dy)

                # 检查是否满足条件
                if abs_dx < threshold_val and abs_dy < threshold_val:
                    base_point_counter += 1
                else:
                    base_point_counter = 0

                # 如果连续满足阈值超过2帧，切换到下一个基准点
                if base_point_counter >= 2:
                    current_base_point_index = (current_base_point_index + 1) % 16
                    base_point_counter = 0
                    print("基准点切换到:", current_base_point_index)

                # 计算中心点差值
                dx_center = TARGET_POINT[0] - center_x
                dy_center = TARGET_POINT[1] - center_y

                # 计算基准点差值
                dx_base = TARGET_POINT[0] - cp_x
                dy_base = TARGET_POINT[1] - cp_y

                # 处理中心点差值数据
                if dx_center < 0:
                    sign_dx_center = 1
                    dx_center_val = min(255, int(abs(dx_center)))
                else:
                    sign_dx_center = 0
                    dx_center_val = min(255, int(abs(dx_center)))

                if dy_center < 0:
                    sign_dy_center = 1
                    dy_center_val = min(255, int(abs(dy_center)))
                else:
                    sign_dy_center = 0
                    dy_center_val = min(255, int(abs(dy_center)))

                # 处理基准点差值数据
                if dx_base < 0:
                    sign_dx_base = 1
                    dx_base_val = min(255, int(abs(dx_base)))
                else:
                    sign_dx_base = 0
                    dx_base_val = min(255, int(abs(dx_base)))

                if dy_base < 0:
                    sign_dy_base = 1
                    dy_base_val = min(255, int(abs(dy_base)))
                else:
                    sign_dy_base = 0
                    dy_base_val = min(255, int(abs(dy_base)))

        # 如果没有检测到矩形，重置缓存
        if min_corners is None:
            prev_min_corners = None
            prev_center = None
            prev_circle_points = None

        # 更新上一帧状态
        prev_has_rect = current_has_rect

        # 显示状态信息
        img.draw_circle(TARGET_POINT[0], TARGET_POINT[1], 3, color=(255, 0, 255), thickness=1)
        img.draw_string_advanced(10, 10, 15, "fps: {:.1f}".format(clock.fps()), color=(255, 0, 0))
        img.draw_string_advanced(10, 30, 15, "status: {}".format("Tracking" if flag_detected else "Lost"),
                                color=(0, 255, 0) if flag_detected else (255, 0, 0))
        if min_corners is not None and not use_prev:
            img.draw_string_advanced(10, 50, 15, "valid ratio: {:.2f}".format(min_black_ratio), color=(255, 255, 255))
            img.draw_string_advanced(10, 70, 15, f"base point: {current_base_point_index}", color=(255, 0, 0))

        # 发送串口数据
        sending_data(
            flag_byte,
            sign_dx_center,
            dx_center_val,
            sign_dy_center,
            dy_center_val,
            base_index,
            sign_dx_base,
            dx_base_val,
            sign_dy_base,
            dy_base_val
        )

        img.compressed_for_ide()
        Display.show_image(img, x=(800-320)//2, y=(480-240)//2)
        time.sleep_ms(10)

finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
