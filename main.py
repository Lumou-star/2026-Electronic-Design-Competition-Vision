#"""
#K230 Steel Ball Detection V2

#YOLOv8 steel-ball detection demo for CanMV K230.
#The detected ball center is compared with the image center,
#and the horizontal pixel offset is sent through UART2.

#Tested environment:
#- Board: K230 LCKFB
#- Firmware: CanMV v1.6
#- Model input: 320 x 320
#- Camera AI frame: 640 x 360
#- UART: UART2, 115200 baud
#- TX: IO11
#- RX: IO12

#License: AGPL-3.0-only
#"""
from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8

from machine import FPIOA, UART, TOUCH, Timer

import os
import sys
import gc
import time
import ustruct


# ============================================================
# 1. YOLO V2 配置
# ============================================================

# 模型路径
MODEL_PATH = "/sdcard/best_v2.kmodel"

# 摄像头AI画面尺寸
RGB888P_SIZE = [640, 360]

# 新模型仍然使用320×320输入
MODEL_INPUT_SIZE = [320, 320]

# 类别名称保持不变
LABELS = ["ball"]

# 阈值
CONF_THRESHOLD = 0.30
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 20
BALL_BOX_THICKNESS = 4
BALL_CENTER_THICKNESS = 3


# ============================================================
# 2. UART配置
# ============================================================

UART_ID = UART.UART2

# K230 UART2引脚
UART_TX_PIN = 11
UART_RX_PIN = 12

UART_BAUDRATE = 115200

# 每隔至少50ms发送一次
UART_SEND_INTERVAL_MS = 10
UART_TARGET_LOST_STOP_MS = 1000

# 未检测到钢珠时发送
SPEED_SAMPLE_INTERVAL_MS = 200
SPEED_STOP_DEADBAND_CM = 0.15
SPEED_LOST_TIMEOUT_MS = 500

# IDE终端输出间隔
CALIBRATION_FILE = "/sdcard/pipe_calibration.txt"
DEFAULT_CALIBRATION = (80, 400, 720)
DEFAULT_ROI_BOUNDS = (115, 245)
CALIBRATION_NAMES = ("LEFT", "CENTER", "RIGHT", "TOP", "BOTTOM")
CALIBRATION_CM = (-12.5, 0.0, 12.5)
CALIBRATION_MIN_GAP = 2
CALIBRATION_STEP = 1
ROI_MIN_GAP = 2

BUTTON_PLUS = (15, 400, 175, 65)
BUTTON_MINUS = (210, 400, 175, 65)
BUTTON_MODE = (405, 400, 175, 65)
BUTTON_SAVE = (600, 400, 185, 65)
BUTTON_RECTS = (BUTTON_PLUS, BUTTON_MINUS, BUTTON_MODE, BUTTON_SAVE)
BUTTON_LABELS = ("+", "-", "MODE", "SAVE")
TOUCH_REPEAT_DELAY_MS = 450
TOUCH_REPEAT_INTERVAL_MS = 80
SAVE_NOTICE_MS = 1500

ARGB_BLACK = (255, 0, 0, 0)
ARGB_GRAY = (255, 50, 50, 50)
ARGB_WHITE = (255, 255, 255, 255)
ARGB_YELLOW = (255, 255, 255, 0)
ARGB_RED = (255, 255, 0, 0)
ARGB_GREEN = (255, 0, 255, 0)
ARGB_CYAN = (255, 0, 255, 255)


# ============================================================
# 3. 运行对象
# ============================================================

pl = None
yolo = None
uart = None
uart_sender = None

pipeline_created = False
yolo_created = False
uart_created = False


# ============================================================
# 4. 选择置信度最高的钢珠
# ============================================================

def filter_result_roi(result, points, roi_bounds):
    """Keep only YOLO boxes whose centers are inside the calibrated box."""
    if result is None or len(result) < 3:
        return result
    boxes, class_ids, scores = result[0], result[1], result[2]
    kept_boxes = []
    kept_class_ids = []
    kept_scores = []
    count = min(len(boxes), len(class_ids), len(scores))
    left_x, _, right_x = points
    top_y, bottom_y = roi_bounds
    for index in range(count):
        try:
            box = boxes[index]
            center_x = float(box[0]) + float(box[2]) * 0.5
            center_y = float(box[1]) + float(box[3]) * 0.5
            if left_x <= center_x <= right_x and \
                    top_y <= center_y <= bottom_y:
                kept_boxes.append(box)
                kept_class_ids.append(class_ids[index])
                kept_scores.append(scores[index])
        except Exception:
            pass
    return (kept_boxes, kept_class_ids, kept_scores)


def draw_detection_box(osd_img, points, roi_bounds, active_mode):
    left_x, _, right_x = points
    top_y, bottom_y = roi_bounds
    left_color = ARGB_RED if active_mode == 0 else ARGB_GREEN
    right_color = ARGB_RED if active_mode == 2 else ARGB_GREEN
    top_color = ARGB_RED if active_mode == 3 else ARGB_GREEN
    bottom_color = ARGB_RED if active_mode == 4 else ARGB_GREEN
    osd_img.draw_line(left_x, top_y, right_x, top_y,
                      color=top_color,
                      thickness=4 if active_mode == 3 else 2)
    osd_img.draw_line(left_x, bottom_y, right_x, bottom_y,
                      color=bottom_color,
                      thickness=4 if active_mode == 4 else 2)
    osd_img.draw_line(left_x, top_y, left_x, bottom_y,
                      color=left_color,
                      thickness=4 if active_mode == 0 else 2)
    osd_img.draw_line(right_x, top_y, right_x, bottom_y,
                      color=right_color,
                      thickness=4 if active_mode == 2 else 2)


def load_calibration(display_width, display_height):
    try:
        config_file = open(CALIBRATION_FILE, "r")
        values = [int(value.strip()) for value in
                  config_file.readline().strip().split(",")]
        config_file.close()
        if len(values) not in (3, 5) or not (
                0 <= values[0] < values[1] < values[2] < display_width):
            raise ValueError("invalid calibration")
        points = values[:3]
        bounds = values[3:5] if len(values) == 5 else \
            list(DEFAULT_ROI_BOUNDS)
        if not (0 <= bounds[0] < bounds[1] < display_height):
            raise ValueError("invalid roi bounds")
        print("calibration loaded:", points, "roi:", bounds)
        return points, bounds
    except Exception as error:
        print("using default calibration:", error)
        return list(DEFAULT_CALIBRATION), list(DEFAULT_ROI_BOUNDS)


def save_calibration(points, roi_bounds):
    try:
        config_file = open(CALIBRATION_FILE, "w")
        config_file.write("%d,%d,%d,%d,%d\n" %
                          tuple(points + roi_bounds))
        config_file.close()
        return "SAVED"
    except Exception as error:
        print("calibration save failed:", error)
        return "SAVE ERR"


def hit_button(x, y):
    for index, button in enumerate(BUTTON_RECTS):
        bx, by, width, height = button
        if bx <= x < bx + width and by <= y < by + height:
            return index
    return -1


def adjust_calibration(points, roi_bounds, mode, amount,
                       display_width, display_height):
    if mode == 0:
        low, high = 0, points[1] - CALIBRATION_MIN_GAP
    elif mode == 1:
        low = points[0] + CALIBRATION_MIN_GAP
        high = points[2] - CALIBRATION_MIN_GAP
    elif mode == 2:
        low, high = points[1] + CALIBRATION_MIN_GAP, display_width - 1
    elif mode == 3:
        low, high = 0, roi_bounds[1] - ROI_MIN_GAP
        roi_bounds[0] = max(low, min(high, roi_bounds[0] + amount))
        return
    else:
        low, high = roi_bounds[0] + ROI_MIN_GAP, display_height - 1
        roi_bounds[1] = max(low, min(high, roi_bounds[1] + amount))
        return
    points[mode] = max(low, min(high, points[mode] + amount))


def pixel_to_cm(x_display, points):
    left_x, center_x, right_x = points
    x_display = max(left_x, min(right_x, float(x_display)))
    if x_display <= center_x:
        span = max(1, center_x - left_x)
        return -12.5 + (x_display - left_x) * 12.5 / span
    span = max(1, right_x - center_x)
    return (x_display - center_x) * 12.5 / span


def process_touch(tp, state, points, roi_bounds, mode, now_ms,
                  display_width, display_height):
    button = -1
    touches = tp.read(1)
    if touches:
        button = hit_button(touches[0].x, touches[0].y)
    if button < 0:
        state["button"] = -1
        state["start"] = 0
        state["repeat"] = 0
        return mode, None
    if button != state["button"]:
        state["button"] = button
        state["start"] = now_ms
        state["repeat"] = now_ms
        if button == 0:
            adjust_calibration(points, roi_bounds, mode, CALIBRATION_STEP,
                               display_width, display_height)
        elif button == 1:
            adjust_calibration(points, roi_bounds, mode, -CALIBRATION_STEP,
                               display_width, display_height)
        elif button == 2:
            mode = (mode + 1) % len(CALIBRATION_NAMES)
        else:
            return mode, save_calibration(points, roi_bounds)
        return mode, None
    if button <= 1 and time.ticks_diff(now_ms, state["start"]) >= \
            TOUCH_REPEAT_DELAY_MS and time.ticks_diff(
                now_ms, state["repeat"]) >= TOUCH_REPEAT_INTERVAL_MS:
        adjust_calibration(
            points, roi_bounds, mode,
            CALIBRATION_STEP if button == 0 else -CALIBRATION_STEP,
            display_width, display_height
        )
        state["repeat"] = now_ms
    return mode, None


def draw_calibration_ui(osd_img, points, roi_bounds, mode,
                        pressed_button, notice, ball_x, fps, speed_cm_s,
                        display_width):
    marker_y = (roi_bounds[0] + roi_bounds[1]) // 2
    if mode == 1:
        osd_img.draw_circle(points[1], marker_y, 10,
                            color=ARGB_RED, thickness=3)
        osd_img.draw_cross(points[1], marker_y,
                           color=ARGB_RED, size=24, thickness=3)
    if mode < 3:
        calibration_text = "CAL %s %+.1fcm X:%d L:%d C:%d R:%d" % (
            CALIBRATION_NAMES[mode], CALIBRATION_CM[mode], points[mode],
            points[0], points[1], points[2]
        )
    else:
        calibration_text = "CAL %s Y:%d TOP:%d BOTTOM:%d" % (
            CALIBRATION_NAMES[mode], roi_bounds[mode - 3],
            roi_bounds[0], roi_bounds[1]
        )
    osd_img.draw_string_advanced(8, 8, 22, calibration_text,
                                 color=ARGB_YELLOW)
    osd_img.draw_string_advanced(
        8, 48, 20, "BALL_X:%d  LEFT_X:%d  RIGHT_X:%d" %
        (int(round(ball_x)), points[0], points[2]), color=ARGB_CYAN
    )
    osd_img.draw_string_advanced(
        620, 8, 24, "POS:%+.1f cm" % pixel_to_cm(ball_x, points),
        color=ARGB_GREEN
    )
    osd_img.draw_string_advanced(680, 40, 22, "FPS:%.1f" % fps,
                                 color=ARGB_GREEN)
    osd_img.draw_string_advanced(
        570, 72, 22, "SPEED:%.1f cm/s" % speed_cm_s,
        color=ARGB_GREEN
    )
    osd_img.draw_rectangle(0, 390, display_width, 90,
                           color=ARGB_BLACK, fill=True)
    for index, button in enumerate(BUTTON_RECTS):
        x, y, width, height = button
        pressed = index == pressed_button
        fill_color = ARGB_GREEN if pressed else ARGB_GRAY
        text_color = ARGB_BLACK if pressed else ARGB_WHITE
        osd_img.draw_rectangle(x, y, width, height,
                               color=fill_color, fill=True)
        osd_img.draw_rectangle(x, y, width, height,
                               color=ARGB_WHITE, thickness=3)
        label_x = x + 70 if index < 2 else x + 48
        osd_img.draw_string_advanced(label_x, y + 16, 28,
                                     BUTTON_LABELS[index], color=text_color)
    if notice:
        osd_img.draw_string_advanced(650, 365, 20, notice,
                                     color=ARGB_GREEN)


def make_position_packet(ball_x, left_x, right_x):
    ball_x = max(0, min(65535, int(round(ball_x))))
    left_x = max(0, min(65535, int(round(left_x))))
    right_x = max(0, min(65535, int(round(right_x))))
    return ustruct.pack(">BBBHHHB", 0x01, 0x02, 0x03,
                        ball_x, left_x, right_x, 0xAA)


class UartPeriodicSender:
    def __init__(self, uart_device, ball_x, left_x, right_x):
        self.uart = uart_device
        self.timer = None
        self.packet = make_position_packet(ball_x, left_x, right_x)
        self.enabled = False
        self.last_target_ms = time.ticks_ms()

    def update(self, ball_x, left_x, right_x):
        self.packet = make_position_packet(ball_x, left_x, right_x)

    def target_seen(self, now_ms):
        self.last_target_ms = now_ms
        self.enabled = True

    def _send(self, timer):
        try:
            if not self.enabled:
                return
            if time.ticks_diff(time.ticks_ms(), self.last_target_ms) >= \
                    UART_TARGET_LOST_STOP_MS:
                self.enabled = False
                return
            self.uart.write(self.packet)
        except Exception:
            pass

    def start(self):
        self.timer = Timer(-1)
        self.timer.init(period=UART_SEND_INTERVAL_MS,
                        mode=Timer.PERIODIC, callback=self._send)

    def stop(self):
        if self.timer is not None:
            try:
                self.timer.deinit()
            except Exception:
                pass
            self.timer = None


def get_best_ball(result):
    """
    CanMV v1.6 YOLOv8检测结果：

    result[0] = boxes
    result[1] = class_ids
    result[2] = scores

    单个box：
    [x, y, width, height]

    坐标属于显示坐标系。
    """

    if result is None:
        return None

    try:
        if len(result) < 3:
            return None

        boxes = result[0]
        class_ids = result[1]
        scores = result[2]

        count = min(
            len(boxes),
            len(class_ids),
            len(scores)
        )

    except Exception:
        return None

    if count <= 0:
        return None

    best_ball = None
    best_score = -1.0

    for index in range(count):

        try:
            box = boxes[index]

            if len(box) < 4:
                continue

            box_x = int(box[0])
            box_y = int(box[1])
            box_w = int(box[2])
            box_h = int(box[3])

            class_id = int(class_ids[index])
            score = float(scores[index])

        except Exception:
            continue

        # 当前只有ball一个类别
        if class_id != 0:
            continue

        if score < CONF_THRESHOLD:
            continue

        if box_w <= 0 or box_h <= 0:
            continue

        if score > best_score:

            best_score = score

            center_x = box_x + box_w // 2
            center_y = box_y + box_h // 2

            best_ball = (
                center_x,
                center_y,
                box_x,
                box_y,
                box_w,
                box_h,
                score
            )

    return best_ball


def draw_best_ball_marker(osd_img, best_ball):
    """Draw only the selected steel ball with a thick marker."""
    if best_ball is None:
        return
    center_x, center_y, box_x, box_y, box_w, box_h, score = best_ball
    osd_img.draw_rectangle(
        int(box_x), int(box_y), int(box_w), int(box_h),
        color=(0, 255, 0), thickness=BALL_BOX_THICKNESS
    )
    osd_img.draw_cross(
        int(center_x), int(center_y), color=(0, 255, 0),
        size=18, thickness=BALL_CENTER_THICKNESS
    )
    osd_img.draw_string_advanced(
        int(box_x), max(0, int(box_y) - 30), 24,
        "ball %.2f" % score, color=(0, 255, 0)
    )


def run_migrated():
    global pl, yolo, uart, uart_sender
    global pipeline_created, yolo_created, uart_created

    os.exitpoint(os.EXITPOINT_ENABLE)
    model_info = os.stat(MODEL_PATH)
    print("model:", MODEL_PATH, "bytes:", model_info[6])

    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    uart = UART(
        UART_ID,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    uart_created = True

    pl = PipeLine(rgb888p_size=RGB888P_SIZE, display_mode="lcd")
    pl.create()
    pipeline_created = True
    display_size = pl.get_display_size()
    display_width = int(display_size[0])
    display_height = int(display_size[1])

    tp = TOUCH(0)
    calibration_points, roi_bounds = load_calibration(
        display_width, display_height
    )
    calibration_mode = 0
    touch_state = {"button": -1, "start": 0, "repeat": 0}
    save_notice = None
    save_notice_until = 0

    yolo = YOLOv8(
        task_type="detect",
        mode="video",
        kmodel_path=MODEL_PATH,
        labels=LABELS,
        rgb888p_size=RGB888P_SIZE,
        model_input_size=MODEL_INPUT_SIZE,
        display_size=display_size,
        conf_thresh=CONF_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,
        max_boxes_num=MAX_BOXES_NUM,
        debug_mode=0,
    )
    yolo_created = True
    yolo.config_preprocess()

    last_ball_x = float(calibration_points[1])
    uart_sender = UartPeriodicSender(
        uart, last_ball_x,
        calibration_points[0], calibration_points[2]
    )
    uart_sender.start()
    print("UART2 timer ready: 115200, period 10 ms")
    print("touch calibration UI enabled")

    frame_count = 0
    fps_window_start_ms = time.ticks_ms()
    fps_window_frames = 0
    current_fps = 0.0
    speed_sample_ms = fps_window_start_ms
    speed_sample_position_cm = pixel_to_cm(
        last_ball_x, calibration_points
    )
    last_ball_seen_ms = speed_sample_ms
    current_speed_cm_s = 0.0

    while True:
        os.exitpoint()
        now_ms = time.ticks_ms()
        calibration_mode, save_result = process_touch(
            tp, touch_state, calibration_points, roi_bounds,
            calibration_mode, now_ms, display_width, display_height
        )
        if save_result is not None:
            save_notice = save_result
            save_notice_until = time.ticks_add(now_ms, SAVE_NOTICE_MS)
        elif save_notice is not None and time.ticks_diff(
                now_ms, save_notice_until) >= 0:
            save_notice = None

        img = pl.get_frame()
        result = yolo.run(img)
        result = filter_result_roi(result, calibration_points, roi_bounds)

        pl.osd_img.clear()
        best_ball = get_best_ball(result)
        draw_best_ball_marker(pl.osd_img, best_ball)
        if best_ball is not None:
            ball_x, ball_y, _, _, _, _, _ = best_ball
            ball_x = max(0, min(display_width - 1, int(ball_x)))
            ball_y = max(0, min(display_height - 1, int(ball_y)))
            last_ball_x = float(ball_x)
            last_ball_seen_ms = now_ms
            speed_elapsed_ms = time.ticks_diff(now_ms, speed_sample_ms)
            if speed_elapsed_ms >= SPEED_SAMPLE_INTERVAL_MS:
                ball_position_cm = pixel_to_cm(
                    last_ball_x, calibration_points
                )
                displacement_cm = abs(
                    ball_position_cm - speed_sample_position_cm
                )
                if displacement_cm <= SPEED_STOP_DEADBAND_CM:
                    current_speed_cm_s = 0.0
                else:
                    current_speed_cm_s = (
                        displacement_cm * 1000.0 / speed_elapsed_ms
                    )
                speed_sample_position_cm = ball_position_cm
                speed_sample_ms = now_ms
        elif time.ticks_diff(now_ms, last_ball_seen_ms) >= \
                SPEED_LOST_TIMEOUT_MS:
            current_speed_cm_s = 0.0
            speed_sample_position_cm = pixel_to_cm(
                last_ball_x, calibration_points
            )
            speed_sample_ms = now_ms

        uart_sender.update(
            last_ball_x, calibration_points[0], calibration_points[2]
        )
        if best_ball is not None:
            uart_sender.target_seen(now_ms)

        frame_count += 1
        fps_window_frames += 1
        fps_elapsed_ms = time.ticks_diff(now_ms, fps_window_start_ms)
        if fps_elapsed_ms >= 1000:
            current_fps = fps_window_frames * 1000.0 / fps_elapsed_ms
            fps_window_frames = 0
            fps_window_start_ms = now_ms

        draw_detection_box(
            pl.osd_img, calibration_points, roi_bounds, calibration_mode
        )
        draw_calibration_ui(
            pl.osd_img, calibration_points, roi_bounds, calibration_mode,
            touch_state["button"], save_notice, last_ball_x,
            current_fps, current_speed_cm_s, display_width
        )
        pl.show_image()
        if frame_count % 30 == 0:
            gc.collect()


try:
    print("main.py UI + UART migrated from 1.py")
    run_migrated()
    raise KeyboardInterrupt

    print("=" * 52)
    print("K230 YOLOv8 ball detection V2 + UART DX")
    print("=" * 52)

    os.exitpoint(os.EXITPOINT_ENABLE)

    # ========================================================
    # 5. 检查V2模型
    # ========================================================

    try:
        model_info = os.stat(MODEL_PATH)

        print("V2模型路径：", MODEL_PATH)
        print("V2模型大小：{} bytes".format(model_info[6]))

    except Exception as error:
        print("没有找到V2模型：", MODEL_PATH)
        print("请确认模型已经上传到SD卡")
        raise error

    # ========================================================
    # 6. 初始化UART2
    # ========================================================

    fpioa = FPIOA()

    fpioa.set_function(
        UART_TX_PIN,
        FPIOA.UART2_TXD
    )

    fpioa.set_function(
        UART_RX_PIN,
        FPIOA.UART2_RXD
    )

    uart = UART(
        UART_ID,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE
    )

    uart_created = True

    print("UART2初始化成功")
    print("TX：IO{}".format(UART_TX_PIN))
    print("RX：IO{}".format(UART_RX_PIN))
    print("波特率：", UART_BAUDRATE)

    # ========================================================
    # 7. 初始化PipeLine
    # ========================================================

    pl = PipeLine(
        rgb888p_size=RGB888P_SIZE,
        display_mode="lcd"
    )

    pl.create()

    pipeline_created = True

    display_size = pl.get_display_size()

    display_width = int(display_size[0])
    display_height = int(display_size[1])

    print("AI画面：", RGB888P_SIZE)
    print("显示画面：", display_size)

    # 显示坐标系中心
    display_center_x = display_width // 2
    display_center_y = display_height // 2

    # 640×360摄像头坐标系中心
    ai_center_x = RGB888P_SIZE[0] // 2

    print(
        "显示中心：({}, {})".format(
            display_center_x,
            display_center_y
        )
    )

    print("AI坐标中心X：", ai_center_x)

    # ========================================================
    # 8. 初始化YOLOv8 V2
    # ========================================================

    yolo = YOLOv8(
        task_type="detect",
        mode="video",

        kmodel_path=MODEL_PATH,
        labels=LABELS,

        rgb888p_size=RGB888P_SIZE,
        model_input_size=MODEL_INPUT_SIZE,
        display_size=display_size,

        conf_thresh=CONF_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,

        max_boxes_num=MAX_BOXES_NUM,
        debug_mode=0
    )

    yolo_created = True

    yolo.config_preprocess()

    print("YOLOv8 V2初始化成功")
    print("置信度阈值：", CONF_THRESHOLD)
    print("开始识别并发送DX")

    # ========================================================
    # 9. 定时状态
    # ========================================================

    last_uart_time = (
        time.ticks_ms()
        -
        UART_SEND_INTERVAL_MS
    )

    last_ide_print_time = (
        time.ticks_ms()
        -
        IDE_PRINT_INTERVAL_MS
    )

    latest_dx = NO_TARGET_VALUE
    latest_score = 0.0

    frame_count = 0

    # ========================================================
    # 10. 实时检测
    # ========================================================

    while True:
        os.exitpoint()

        frame_count += 1

        # 获取摄像头AI帧
        img = pl.get_frame()

        # 使用V2模型推理
        result = yolo.run(img)

        # 清除上一帧OSD
        pl.osd_img.clear()

        # 绘制YOLO检测框
        yolo.draw_result(
            result,
            pl.osd_img
        )

        # 红色十字：画面中心
        pl.osd_img.draw_cross(
            display_center_x,
            display_center_y,
            color=(255, 0, 0),
            size=16,
            thickness=2
        )

        # ====================================================
        # 11. 获取钢珠中心
        # ====================================================

        best_ball = get_best_ball(result)

        if best_ball is not None:

            (
                ball_display_x,
                ball_display_y,
                box_x,
                box_y,
                box_w,
                box_h,
                latest_score
            ) = best_ball

            # 限制显示坐标范围
            if ball_display_x < 0:
                ball_display_x = 0

            elif ball_display_x >= display_width:
                ball_display_x = display_width - 1

            if ball_display_y < 0:
                ball_display_y = 0

            elif ball_display_y >= display_height:
                ball_display_y = display_height - 1

            # =================================================
            # 显示坐标换算回640×360坐标
            # =================================================

            ball_ai_x = (
                ball_display_x
                *
                RGB888P_SIZE[0]
                //
                display_width
            )

            # 横向差值
            latest_dx = ball_ai_x - ai_center_x

            # 绿色十字：钢珠中心
            pl.osd_img.draw_cross(
                ball_display_x,
                ball_display_y,
                color=(0, 255, 0),
                size=12,
                thickness=2
            )

            # 黄色连线
            pl.osd_img.draw_line(
                display_center_x,
                display_center_y,
                ball_display_x,
                ball_display_y,
                color=(255, 255, 0),
                thickness=2
            )

            # 显示V2、DX和置信度
            pl.osd_img.draw_string_advanced(
                10,
                10,
                24,
                "V2 DX={}".format(latest_dx),
                color=(255, 255, 0)
            )

            pl.osd_img.draw_string_advanced(
                10,
                40,
                20,
                "CONF={:.3f}".format(latest_score),
                color=(0, 255, 0)
            )

        else:

            latest_dx = NO_TARGET_VALUE
            latest_score = 0.0

            pl.osd_img.draw_string_advanced(
                10,
                10,
                24,
                "V2 DX=NO TARGET",
                color=(255, 255, 0)
            )

        # ====================================================
        # 12. 每隔至少50ms发送UART
        # ====================================================

        current_time = time.ticks_ms()

        if (
            time.ticks_diff(
                current_time,
                last_uart_time
            )
            >= UART_SEND_INTERVAL_MS
        ):

            uart_message = "{}\r\n".format(
                latest_dx
            )

            uart.write(
                uart_message.encode()
            )

            last_uart_time = current_time

        # ====================================================
        # 13. IDE终端定期打印
        # ====================================================

        if (
            time.ticks_diff(
                current_time,
                last_ide_print_time
            )
            >= IDE_PRINT_INTERVAL_MS
        ):

            print(
                "V2 DX={}, score={:.3f}".format(
                    latest_dx,
                    latest_score
                )
            )

            last_ide_print_time = current_time

        # 显示画面
        pl.show_image()

        # 每30帧回收一次内存
        if frame_count % 30 == 0:
            gc.collect()


except KeyboardInterrupt:
    print("用户停止V2测试")


except BaseException as error:
    print("V2运行异常：")

    try:
        sys.print_exception(error)

    except Exception:
        print(error)


finally:

    if uart_sender is not None:
        uart_sender.stop()

    if yolo_created and yolo is not None:
        try:
            yolo.deinit()
        except Exception:
            pass

    if pipeline_created and pl is not None:
        try:
            pl.destroy()
        except Exception:
            pass

    if uart_created and uart is not None:
        try:
            uart.deinit()
        except Exception:
            pass

    gc.collect()
    time.sleep_ms(100)

    print("V2测试程序结束")
