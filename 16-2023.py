# 内外双矩形 + 红绿激光识别
# 基于 cv_lite OpenCV Canny 边缘检测方案优化
# 从 2025电赛 移植矩形几何验证逻辑
import time, os, sys
from math import sqrt, atan2, degrees

from media.sensor import *
from media.display import *
from media.media import *
import cv_lite
from machine import UART, FPIOA

sensor_id = 2
sensor = None

# ==================== 采集参数 ====================
SENSOR_WIDTH  = 480
SENSOR_HEIGHT = 320
# cv_lite 要求 image_shape = [height, width]
IMAGE_SHAPE = [SENSOR_HEIGHT, SENSOR_WIDTH]

# ==================== cv_lite 矩形检测参数 ====================
CANNY_THRESH1      = 50        # Canny 边缘检测低阈值
CANNY_THRESH2      = 150       # Canny 边缘检测高阈值
APPROX_EPSILON     = 0.02      # 多边形拟合精度（比例），越小越精确
AREA_MIN_RATIO     = 0.001     # 最小面积比例（0~1，相对画面面积）
MAX_ANGLE_COS      = 0.5       # 最大角余弦（越小越接近直角）
GAUSSIAN_BLUR_SIZE = 5         # 高斯模糊核大小（奇数）

# ==================== 矩形几何验证参数 ====================
RECT_AREA_MIN     = 5000       # 矩形面积下限（像素）
MIN_SIDE_LENGTH   = 30         # 最小边长（像素）
LENGTH_THRESHOLD  = 120        # 对边长度差上限（像素）
AREA_RATIO_MIN    = 1.3        # 内外框面积比下限
AREA_RATIO_MAX    = 10.0       # 内外框面积比上限
PARALLEL_TOL      = 30         # 平行判定角度容差（度）
VERTICAL_TOL      = 30         # 垂直判定角度容差（度）

# ==================== 激光检测 (LAB, 双阈值 = 黑底 + 白底) ====================
# 红色激光 — LO=黑底(暗激光)  HI=白底(亮光晕)
RED_LASER_LO = (25,  90,  16, 79, -19, 76)
RED_LASER_HI = (0, 100, 19, 98, 7, 65)

# 绿色激光 — LO=黑底  HI=白底
GREEN_LASER_LO = (25,  90,  -71, -18, -5, 53)
GREEN_LASER_HI = (120, 255, -71, -18, -5, 53)


# ==================== 辅助函数 ====================

def fix_merged_corners(corners, rect, threshold=20):
    """
    检测并修复矩形角点合并问题。
    当任意两个角点距离小于阈值时，说明它们被错误合并，
    此时使用边界矩形 rect=[x,y,w,h] 重新生成四个角点。
    """
    for i in range(4):
        for j in range(i + 1, 4):
            d = sqrt((corners[i][0]-corners[j][0])**2 + (corners[i][1]-corners[j][1])**2)
            if d < threshold:
                x, y, w, h = rect
                print(f"角点{i}和{j}合并 (距离={d:.1f}px)，使用边界矩形修正")
                return [[x, y], [x+w, y], [x+w, y+h], [x, y+h]]
    return corners


def inner_inside_outer(outer_bb, inner_bb):
    """检查内框是否在外框内部（含 margin 容差）"""
    ox, oy, ow, oh = outer_bb
    ix, iy, iw, ih = inner_bb
    m = 3
    return (ix >= ox-m and iy >= oy-m and
            ix+iw <= ox+ow+m and iy+ih <= oy+oh+m)


def sort_corners(corners):
    """将角点排序为 左上、右上、右下、左下"""
    c = sorted(corners, key=lambda p: p[1])
    top = sorted(c[:2], key=lambda p: p[0])
    bottom = sorted(c[2:], key=lambda p: p[0])
    return [top[0], top[1], bottom[1], bottom[0]]  # TL, TR, BR, BL


def are_segments_parallel(theta1, theta2, tolerance=PARALLEL_TOL):
    """判断两条线段是否平行（角度差在 tolerance 度内）"""
    diff = abs(theta1 - theta2)
    if diff > 180:
        diff = diff - 180
    return diff <= tolerance or abs(diff - 180) <= tolerance


def are_segments_vertical(theta1, theta2, tolerance=VERTICAL_TOL):
    """判断两条线段是否垂直（角度差约 90 度）"""
    diff = abs(theta1 - theta2)
    if diff > 180:
        diff = diff - 180
    return abs(diff - 90) <= tolerance


def validate_rectangle(corners):
    """
    验证是否为标准矩形（2025电赛方案的核心逻辑）：
    1. 四边长度足够
    2. 对边长度接近（非梯形）
    3. 对边平行
    4. 邻边垂直
    返回: (is_valid, corners_sorted)
    """
    c = sort_corners(corners)

    # 边长
    len_top    = sqrt((c[0][0]-c[1][0])**2 + (c[0][1]-c[1][1])**2)   # 上边 TL→TR
    len_bottom = sqrt((c[2][0]-c[3][0])**2 + (c[2][1]-c[3][1])**2)   # 下边 BR→BL
    len_left   = sqrt((c[0][0]-c[3][0])**2 + (c[0][1]-c[3][1])**2)   # 左边 TL→BL
    len_right  = sqrt((c[1][0]-c[2][0])**2 + (c[1][1]-c[2][1])**2)   # 右边 TR→BR

    # 最小边长检查
    if min(len_top, len_bottom, len_left, len_right) < MIN_SIDE_LENGTH:
        return False, c

    # 对边长度差（排除梯形等非矩形）
    if abs(len_top - len_bottom) > LENGTH_THRESHOLD:
        return False, c
    if abs(len_left - len_right) > LENGTH_THRESHOLD:
        return False, c

    # 各边角度（度）
    theta_top    = degrees(atan2(c[0][1]-c[1][1], c[0][0]-c[1][0]))
    theta_bottom = degrees(atan2(c[2][1]-c[3][1], c[2][0]-c[3][0]))
    theta_left   = degrees(atan2(c[0][1]-c[3][1], c[0][0]-c[3][0]))
    theta_right  = degrees(atan2(c[1][1]-c[2][1], c[1][0]-c[2][0]))

    # 对边平行
    if not are_segments_parallel(theta_top, theta_bottom):
        return False, c
    if not are_segments_parallel(theta_left, theta_right):
        return False, c

    # 邻边垂直
    if not are_segments_vertical(theta_top, theta_left):
        return False, c

    return True, c


def find_max_rect(rects_list):
    """从矩形列表中找出面积最大的"""
    max_size = 0
    max_rect = None
    for r in rects_list:
        area = r[2] * r[3]  # w * h
        if area > max_size:
            max_rect = r
            max_size = area
    return max_rect


# ==================== 主程序 ====================

try:
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    # RGB888 是 cv_lite 的硬性要求
    sensor.set_framesize(chn=CAM_CHN_ID_0, width=SENSOR_WIDTH, height=SENSOR_HEIGHT)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()

    # ====== UART 初始化 ======
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, 115200)

    clock = time.clock()
    clock.reset()

    last_outer, last_inner = None, None
    last_red, last_green = None, None
    drop_count = 0

    while True:
        clock.tick()
        os.exitpoint()

        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # ==== cv_lite OpenCV 矩形检测 ====
        img_np = img.to_numpy_ref()  # 获取 RGB888 ndarray 引用（零拷贝）
        rects_raw = cv_lite.rgb888_find_rectangles_with_corners(
            IMAGE_SHAPE, img_np,
            CANNY_THRESH1, CANNY_THRESH2,
            APPROX_EPSILON,
            AREA_MIN_RATIO,
            MAX_ANGLE_COS,
            GAUSSIAN_BLUR_SIZE
        )

        outer_corners, inner_corners = None, None

        if rects_raw:
            # rects_raw 格式: [[x,y,w,h,c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y], ...]
            valid_rects = []

            for r in rects_raw:
                x, y, w, h = r[0], r[1], r[2], r[3]
                area = w * h
                if area < RECT_AREA_MIN:
                    continue

                # 提取角点
                corners = [[r[4], r[5]], [r[6], r[7]], [r[8], r[9]], [r[10], r[11]]]
                corners = fix_merged_corners(corners, [x, y, w, h])

                # 几何验证（对边平行、邻边垂直、边长等）
                is_valid, sorted_c = validate_rectangle(corners)
                if not is_valid:
                    continue

                valid_rects.append((area, sorted_c, [x, y, w, h]))

            # 按面积从大到小排序
            valid_rects.sort(key=lambda r: r[0], reverse=True)

            # 寻找内外矩形配对
            for i in range(len(valid_rects)):
                if outer_corners:
                    break
                for j in range(i + 1, len(valid_rects)):
                    oa, oc, obb = valid_rects[i]
                    ia, ic, ibb = valid_rects[j]
                    ratio = oa / ia
                    if (inner_inside_outer(obb, ibb) and
                            AREA_RATIO_MIN < ratio < AREA_RATIO_MAX):
                        outer_corners, inner_corners = oc, ic
                        break

        # 丢帧保持（矩形检测失败时用上一帧数据）
        if outer_corners:
            last_outer, last_inner = outer_corners, inner_corners
            drop_count = 0
        else:
            drop_count += 1
            if drop_count <= 5 and last_outer:
                outer_corners, inner_corners = last_outer, last_inner

        # ==== 激光检测（双阈值：LO覆盖黑底，HI覆盖白底）====
        def detect_laser(img, th_lo, th_hi):
            """双阈值查找，合并结果后取最大 blob"""
            b1 = img.find_blobs([th_lo], pixels_threshold=1, merge=True, margin=8, x_stride=2)
            b2 = img.find_blobs([th_hi], pixels_threshold=1, merge=True, margin=8, x_stride=2)
            blobs = (b1 if b1 else []) + (b2 if b2 else [])
            if not blobs:
                return None
            b = max(blobs, key=lambda x: x.pixels())
            return (b.cx(), b.cy()) if b.pixels() >= 1 else None

        red_pt = detect_laser(img, RED_LASER_LO, RED_LASER_HI)
        green_pt = detect_laser(img, GREEN_LASER_LO, GREEN_LASER_HI)

        if red_pt:
            last_red = red_pt
        elif last_red:
            red_pt = last_red
        if green_pt:
            last_green = green_pt
        elif last_green:
            green_pt = last_green

        # ==== 串口数据发送（保持原协议不变）====
        # 帧头 + 左上 + 右上 + 右下 + 左下 + 红色激光 + 绿色激光 + 帧尾 = 28字节
        data = bytearray(28)
        data[0] = 0x01
        data[1] = 0x02
        data[2] = 0x03

        if inner_corners:
            ic = sort_corners(inner_corners)

            tl_x = ic[0][0]
            tl_y = ic[0][1]
            tr_x = ic[1][0]
            tr_y = ic[1][1]
            br_x = ic[2][0]
            br_y = ic[2][1]
            bl_x = ic[3][0]
            bl_y = ic[3][1]
        else:
            tl_x = tl_y = tr_x = tr_y = br_x = br_y = bl_x = bl_y = 0

        # 左上 (4 bytes)
        data[3]  = (tl_x >> 8) & 0xFF
        data[4]  = tl_x & 0xFF
        data[5]  = (tl_y >> 8) & 0xFF
        data[6]  = tl_y & 0xFF
        # 右上 (4 bytes)
        data[7]  = (tr_x >> 8) & 0xFF
        data[8]  = tr_x & 0xFF
        data[9]  = (tr_y >> 8) & 0xFF
        data[10] = tr_y & 0xFF
        # 右下 (4 bytes)
        data[11] = (br_x >> 8) & 0xFF
        data[12] = br_x & 0xFF
        data[13] = (br_y >> 8) & 0xFF
        data[14] = br_y & 0xFF
        # 左下 (4 bytes)
        data[15] = (bl_x >> 8) & 0xFF
        data[16] = bl_x & 0xFF
        data[17] = (bl_y >> 8) & 0xFF
        data[18] = bl_y & 0xFF

        # 红色激光 (4 bytes)
        if red_pt:
            rx, ry = red_pt[0], red_pt[1]
            data[19] = (rx >> 8) & 0xFF
            data[20] = rx & 0xFF
            data[21] = (ry >> 8) & 0xFF
            data[22] = ry & 0xFF

        # 绿色激光 (4 bytes)
        if green_pt:
            gx, gy = green_pt[0], green_pt[1]
            data[23] = (gx >> 8) & 0xFF
            data[24] = gx & 0xFF
            data[25] = (gy >> 8) & 0xFF
            data[26] = gy & 0xFF

        # 帧尾
        data[27] = 0xAA

        uart.write(bytes(data))

        # ==== 绘制 ====
        stale = drop_count > 0

        if outer_corners:
            g = 150 if stale else 255
            for i in range(4):
                img.draw_line(outer_corners[i][0], outer_corners[i][1],
                              outer_corners[(i+1)%4][0], outer_corners[(i+1)%4][1],
                              color=(0, g, 0), thickness=2)
            for c in outer_corners:
                img.draw_circle(c[0], c[1], 4, color=(0, g, 0), thickness=1, fill=True)

        if inner_corners:
            r = 180 if stale else 255
            for i in range(4):
                img.draw_line(inner_corners[i][0], inner_corners[i][1],
                              inner_corners[(i+1)%4][0], inner_corners[(i+1)%4][1],
                              color=(r, 0, 0), thickness=2)
            for c in inner_corners:
                img.draw_circle(c[0], c[1], 4, color=(r, 0, 0), thickness=1, fill=True)

        if red_pt:
            img.draw_cross(red_pt[0], red_pt[1], color=(255, 0, 0), size=8, thickness=2)
            img.draw_circle(red_pt[0], red_pt[1], 6, color=(255, 0, 0), thickness=2)
            img.draw_string_advanced(red_pt[0]+10, red_pt[1]-10, 14, "R", color=(255, 0, 0))

        if green_pt and green_pt != red_pt:
            img.draw_cross(green_pt[0], green_pt[1], color=(0, 255, 0), size=8, thickness=2)
            img.draw_circle(green_pt[0], green_pt[1], 6, color=(0, 255, 0), thickness=2)
            img.draw_string_advanced(green_pt[0]+10, green_pt[1]-10, 14, "G", color=(0, 255, 0))

        # 状态文字
        s = ""
        if outer_corners:
            s += "Rect"
        if red_pt:
            s += " R"
        if green_pt:
            s += " G"
        if not s:
            s = "No target"
        img.draw_string_advanced(5, 5, 14, "%s  FPS:%.0f" % (s, clock.fps()), color=(255, 255, 255))

        Display.show_image(img)

        nr = len(valid_rects) if rects_raw else 0
        print("rects:%-2d  red:%s  green:%s  fps:%.0f" % (
            nr, "Y" if red_pt else "N", "Y" if green_pt else "N", clock.fps()))

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
