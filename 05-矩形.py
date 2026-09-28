# 05-矩形检测 + 激光点追踪（触屏调参版）
# 屏幕布局：左400×304=摄像头画面  右400×304=调试画面  底176=按钮
import time, os, sys, math, gc

from media.sensor import *
from media.display import *
from media.media import *
import cv_lite
from machine import UART, FPIOA, TOUCH

# ==================== 参数表 ====================
params = [
    {"name": "Canny低",  "val": 50,   "lo": 1,   "hi": 255, "step": 5},
    {"name": "Canny高",  "val": 150,  "lo": 1,   "hi": 255, "step": 5},
    {"name": "拟合精度",  "val": 0.02, "lo": 0.01,"hi": 0.10,"step": 0.005},
    {"name": "最小面积",  "val": 3000, "lo": 200, "hi": 30000,"step": 200},
    {"name": "高斯模糊",  "val": 5,    "lo": 1,   "hi": 9,   "step": 2},
    {"name": "激光LO",   "val": 200,  "lo": 0,   "hi": 255, "step": 5},
    {"name": "激光HI",   "val": 255,  "lo": 0,   "hi": 255, "step": 5},
]
param_idx = 0
SENSOR_ID   = 2
SENSOR_W    = 400
SENSOR_H    = 304
IMAGE_SHAPE = [SENSOR_H, SENSOR_W]
LASER_PIXELS_MIN = 2
LASER_PIXELS_MAX = 200
MIN_SIDE_LENGTH  = 20
LENGTH_THRESHOLD = 120
PARALLEL_TOL     = 30
VERTICAL_TOL     = 30

# ==================== 辅助函数 ====================

def sort_corners(corners):
    cx = sum(p[0] for p in corners) / 4.0
    cy = sum(p[1] for p in corners) / 4.0
    return sorted(corners, key=lambda p: -math.atan2(p[1] - cy, p[0] - cx))


def fix_merged_corners(corners, rect, threshold=20):
    for i in range(4):
        for j in range(i + 1, 4):
            d = math.sqrt((corners[i][0]-corners[j][0])**2 +
                          (corners[i][1]-corners[j][1])**2)
            if d < threshold:
                x, y, w, h = rect
                return [[x, y], [x+w, y], [x+w, y+h], [x, y+h]]
    return corners


def are_parallel(t1, t2, tol=PARALLEL_TOL):
    diff = abs(t1 - t2)
    if diff > 180: diff -= 180
    return diff <= tol or abs(diff - 180) <= tol


def are_vertical(t1, t2, tol=VERTICAL_TOL):
    diff = abs(t1 - t2)
    if diff > 180: diff -= 180
    return abs(diff - 90) <= tol


def validate_rect(corners):
    c = sort_corners(corners)
    len_top    = math.sqrt((c[0][0]-c[1][0])**2 + (c[0][1]-c[1][1])**2)
    len_bottom = math.sqrt((c[2][0]-c[3][0])**2 + (c[2][1]-c[3][1])**2)
    len_left   = math.sqrt((c[0][0]-c[3][0])**2 + (c[0][1]-c[3][1])**2)
    len_right  = math.sqrt((c[1][0]-c[2][0])**2 + (c[1][1]-c[2][1])**2)
    if min(len_top, len_bottom, len_left, len_right) < MIN_SIDE_LENGTH:
        return False, c
    if abs(len_top - len_bottom) > LENGTH_THRESHOLD:
        return False, c
    if abs(len_left - len_right) > LENGTH_THRESHOLD:
        return False, c
    t_top    = math.degrees(math.atan2(c[0][1]-c[1][1], c[0][0]-c[1][0]))
    t_bottom = math.degrees(math.atan2(c[2][1]-c[3][1], c[2][0]-c[3][0]))
    t_left   = math.degrees(math.atan2(c[0][1]-c[3][1], c[0][0]-c[3][0]))
    t_right  = math.degrees(math.atan2(c[1][1]-c[2][1], c[1][0]-c[2][0]))
    if not are_parallel(t_top, t_bottom) or not are_parallel(t_left, t_right):
        return False, c
    if not are_vertical(t_top, t_left):
        return False, c
    return True, c


def inner_inside_outer(outer_bb, inner_bb):
    ox, oy, ow, oh = outer_bb
    ix, iy, iw, ih = inner_bb
    m = 3
    return (ix >= ox-m and iy >= oy-m and
            ix+iw <= ox+ow+m and iy+ih <= oy+oh+m)


def find_laser(img_gray, lo, hi):
    img_bin = img_gray.binary([(lo, hi)])
    blobs = img_bin.find_blobs([], pixels_threshold=LASER_PIXELS_MIN,
                               area_threshold=LASER_PIXELS_MIN, merge=True)
    if not blobs:
        return None
    best_score = -1
    best_pt = None
    for b in blobs:
        if b.pixels() < LASER_PIXELS_MIN or b.pixels() > LASER_PIXELS_MAX:
            continue
        roundness = b.area() / max(b.w() * b.h(), 1)
        score = b.pixels() * roundness
        if score > best_score:
            best_score = score
            best_pt = (b.cx(), b.cy())
    return best_pt


def make_send_bytes(x, y, cx, cy):
    return bytes([0xFF,
        (x>>8)&0xFF, x&0xFF, (y>>8)&0xFF, y&0xFF,
        (cx>>8)&0xFF, cx&0xFF, (cy>>8)&0xFF, cy&0xFF, 0xFE])


# ==================== 主程序 ====================

try:
    sensor = Sensor(id=SENSOR_ID, width=SENSOR_W, height=SENSOR_H)
    sensor.reset()
    sensor.set_hmirror(True)
    sensor.set_vflip(True)
    sensor.set_framesize(width=SENSOR_W, height=SENSOR_H)
    sensor.set_pixformat(Sensor.RGB888)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True, osd_num=4)
    MediaManager.init()
    sensor.run()

    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, 115200)

    tp = TOUCH(0)
    clock = time.clock()
    clock.reset()

    target_corners = None
    target_locked  = False
    last_laser     = None

    while True:
        clock.tick()
        os.exitpoint()

        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # ---- 读取当前参数 ----
        canny1   = int(params[0]["val"])
        canny2   = int(params[1]["val"])
        eps      = float(params[2]["val"])
        area_min = int(params[3]["val"])
        blur     = int(params[4]["val"])
        laser_lo = int(params[5]["val"])
        laser_hi = int(params[6]["val"])
        if blur % 2 == 0:
            blur += 1

        # ---- cv_lite 矩形检测 ----
        img_np = img.to_numpy_ref()
        rects_raw = cv_lite.rgb888_find_rectangles_with_corners(
            IMAGE_SHAPE, img_np,
            canny1, canny2, eps, 0.001, 0.5, blur
        )

        # 调试画面：灰度版
        img_gray_full = img.to_grayscale()
        img_debug = img_gray_full.to_rgb565()

        valid_rects = []
        if rects_raw:
            for r in rects_raw:
                x, y, w, h = r[0], r[1], r[2], r[3]
                if w * h < area_min:
                    continue
                corners = [[r[4], r[5]], [r[6], r[7]], [r[8], r[9]], [r[10], r[11]]]
                corners = fix_merged_corners(corners, [x, y, w, h])
                ok, sc = validate_rect(corners)
                if ok:
                    valid_rects.append((w * h, sc, [x, y, w, h]))

            valid_rects.sort(key=lambda v: v[0], reverse=True)

            # 找内外矩形配对
            outer, inner = None, None
            for i in range(len(valid_rects)):
                if outer: break
                for j in range(i + 1, len(valid_rects)):
                    oa, oc, obb = valid_rects[i]
                    ia, ic, ibb = valid_rects[j]
                    if inner_inside_outer(obb, ibb) and 1.3 < oa/ia < 10.0:
                        outer, inner = oc, ic
                        break

            if outer and inner:
                target_corners = [list(inner[i]) for i in range(4)]
                for i in range(4):
                    dx = outer[i][0] - inner[i][0]
                    dy = outer[i][1] - inner[i][1]
                    target_corners[i][0] += int(abs(dx)/2) * (1 if dx > 0 else -1)
                    target_corners[i][1] += int(abs(dy)/2) * (1 if dy > 0 else -1)
                if not target_locked:
                    target_locked = True
                    print("锁定目标框:", target_corners)

        # ---- 绘制：摄像头画面 ----
        for _, rc, bb in valid_rects:
            for i in range(4):
                img.draw_line(rc[i][0], rc[i][1], rc[(i+1)%4][0], rc[(i+1)%4][1],
                              color=(0, 0, 255), thickness=1)

        if target_corners:
            for i in range(4):
                img.draw_line(target_corners[i][0], target_corners[i][1],
                              target_corners[(i+1)%4][0], target_corners[(i+1)%4][1],
                              color=(255, 255, 255), thickness=2)
            for p in target_corners:
                img.draw_circle(p[0], p[1], 3, color=(255, 255, 255),
                                thickness=1, fill=True)

        # ---- 绘制：调试画面 ----
        for _, rc, bb in valid_rects:
            for i in range(4):
                img_debug.draw_line(rc[i][0], rc[i][1], rc[(i+1)%4][0], rc[(i+1)%4][1],
                                    color=(0, 255, 0), thickness=1)

        # ---- 激光 ----
        laser_pt = find_laser(img_gray_full, laser_lo, laser_hi)
        if laser_pt:
            last_laser = laser_pt
        else:
            laser_pt = last_laser

        if laser_pt:
            img.draw_cross(laser_pt[0], laser_pt[1], color=(0, 255, 0), size=10, thickness=2)
            img.draw_circle(laser_pt[0], laser_pt[1], 8, color=(0, 255, 0), thickness=2)
            img_debug.draw_cross(laser_pt[0], laser_pt[1], color=(255, 0, 0), size=10, thickness=2)

        # ---- 串口 ----
        if target_corners and laser_pt:
            tx = sum(p[0] for p in target_corners) // 4
            ty = sum(p[1] for p in target_corners) // 4
            uart.write(make_send_bytes(tx, ty, laser_pt[0], laser_pt[1]))

        # ---- 触屏 ----
        touch_pts = tp.read(5)
        if touch_pts:
            for pt in touch_pts:
                tx, ty = pt.x, pt.y
                # 底部控制栏 (y: 304-480)
                if 304 <= ty < 480:
                    if 304 <= ty < 392:  # 第1行
                        if 0 <= tx < 160:
                            param_idx = (param_idx - 1) % len(params)
                            print("选中:", params[param_idx]["name"])
                        elif 160 <= tx < 320:
                            p = params[param_idx]
                            if p["val"] - p["step"]*10 >= p["lo"]:
                                p["val"] = round(p["val"] - p["step"]*10, 4)
                                print(f"{p['name']}: {p['val']}")
                        elif 320 <= tx < 480:
                            p = params[param_idx]
                            if p["val"] - p["step"] >= p["lo"]:
                                p["val"] = round(p["val"] - p["step"], 4)
                                print(f"{p['name']}: {p['val']}")
                        elif 480 <= tx < 640:
                            p = params[param_idx]
                            if p["val"] + p["step"] <= p["hi"]:
                                p["val"] = round(p["val"] + p["step"], 4)
                                print(f"{p['name']}: {p['val']}")
                        elif 640 <= tx < 800:
                            param_idx = (param_idx + 1) % len(params)
                            print("选中:", params[param_idx]["name"])
                    elif 392 <= ty < 480:  # 第2行
                        if 0 <= tx < 200:
                            p = params[param_idx]
                            if p["val"] + p["step"]*10 <= p["hi"]:
                                p["val"] = round(p["val"] + p["step"]*10, 4)
                                print(f"{p['name']}: {p['val']}")
                        elif 200 <= tx < 400:
                            target_locked = not target_locked
                            if not target_locked:
                                target_corners = None
                            print("锁定" if target_locked else "解锁")
                        elif 400 <= tx < 600:
                            params[0]["val"] = 50; params[1]["val"] = 150
                            params[2]["val"] = 0.02; params[3]["val"] = 3000
                            params[4]["val"] = 5; params[5]["val"] = 200
                            params[6]["val"] = 255
                            param_idx = 0
                            target_locked = False; target_corners = None
                            print("参数已重置")
                        elif 600 <= tx < 800:
                            target_locked = False; target_corners = None
                            print("重新检测...")

        # ---- 构建按钮图层 ----
        btn_layer = image.Image(800, 176, image.RGB565)
        btn_layer.clear()
        sel = params[param_idx]

        # 第1行 (88px)
        H1 = 88
        btn_layer.draw_rectangle(0, 0, 160, H1, color=(50,50,50), fill=True)
        btn_layer.draw_rectangle(0, 0, 160, H1, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(15, 25, 18, "<<参数", color=(255,255,255))

        btn_layer.draw_rectangle(160, 0, 160, H1, color=(50,50,50), fill=True)
        btn_layer.draw_rectangle(160, 0, 160, H1, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(170, 25, 18, "-10", color=(255,255,255))

        btn_layer.draw_rectangle(320, 0, 160, H1, color=(50,50,50), fill=True)
        btn_layer.draw_rectangle(320, 0, 160, H1, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(335, 25, 18, "-1", color=(255,255,255))

        btn_layer.draw_rectangle(480, 0, 160, H1, color=(50,50,50), fill=True)
        btn_layer.draw_rectangle(480, 0, 160, H1, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(495, 25, 18, "+1", color=(255,255,255))

        btn_layer.draw_rectangle(640, 0, 160, H1, color=(50,50,50), fill=True)
        btn_layer.draw_rectangle(640, 0, 160, H1, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(655, 25, 18, "参数>>", color=(255,255,255))

        # 第2行 (88px)
        H2 = 88
        btn_layer.draw_rectangle(0, H1, 200, H2, color=(50,50,50), fill=True)
        btn_layer.draw_rectangle(0, H1, 200, H2, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(15, H1+25, 18, "+10", color=(255,255,255))

        lk = (0,180,0) if target_locked else (180,0,0)
        lt = "解除锁定" if target_locked else "锁定矩形"
        btn_layer.draw_rectangle(200, H1, 200, H2, color=lk, fill=True)
        btn_layer.draw_rectangle(200, H1, 200, H2, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(240, H1+25, 18, lt, color=(255,255,255))

        btn_layer.draw_rectangle(400, H1, 200, H2, color=(50,50,50), fill=True)
        btn_layer.draw_rectangle(400, H1, 200, H2, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(435, H1+25, 18, "重置", color=(255,255,255))

        btn_layer.draw_rectangle(600, H1, 200, H2, color=(50,50,50), fill=True)
        btn_layer.draw_rectangle(600, H1, 200, H2, color=(255,255,255), thickness=1)
        btn_layer.draw_string_advanced(630, H1+25, 18, "重检测", color=(255,255,255))

        # 参数信息
        info = f"[{param_idx+1}/7] {sel['name']} = {sel['val']}"
        btn_layer.draw_string_advanced(10, H1+H2-40, 16, info, color=(255,255,0))
        info2 = f"矩形:{len(valid_rects)} {'锁定' if target_locked else '搜索'} FPS:{clock.fps():.0f}"
        btn_layer.draw_string_advanced(10, H1+H2-22, 16, info2, color=(200,200,200))

        # ---- 显示 ----
        img.compressed_for_ide()
        img_debug.compressed_for_ide()
        btn_layer.compressed_for_ide()

        Display.show_image(btn_layer, y=304, layer=Display.LAYER_OSD0)
        Display.show_image(img, layer=Display.LAYER_OSD1)
        Display.show_image(img_debug, x=400, layer=Display.LAYER_OSD2)

        img, img_debug, btn_layer = None, None, None
        gc.collect()

except KeyboardInterrupt as e:
    print("用户停止:", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
