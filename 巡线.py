# K230 CanMV 1.8 line following
# Display RGB image, detect black line by grayscale binary threshold.

import time, os, gc
import ustruct
from machine import FPIOA, UART
from media.sensor import *
from media.display import *
from media.media import *

DISPLAY_MODE = "lcd"
IMG_W = 640
IMG_H = 480
SENSOR_ID = 2

# Binary threshold for black line after converting RGB frame to grayscale.
# Pixel <= BLACK_THRESH is treated as black line.
BLACK_THRESH = 85

ROIS = (
    (0, 400, 640, 60, 0.34),
    (0, 330, 640, 60, 0.26),
    (0, 260, 640, 60, 0.19),
    (0, 190, 640, 60, 0.13),
    (0, 120, 640, 60, 0.08),
)

MIN_PIXELS = 80
MIN_AREA = 80
MERGE_BLOBS = True
GAUSSIAN_ENABLE = False

# Right-angle turn helper.
RIGHT_ANGLE_ENABLE = True
RIGHT_ANGLE_ERR = 190
TURN_HOLD_FRAMES = 8
TURN_TRIGGER_OFFSET = 90

UART_ENABLE = True
UART2_TX_PIN = 11
UART2_RX_PIN = 12
UART_BAUD = 115200
SEND_PERIOD_MS = 20

uart2 = None
last_error = 0
last_send_ms = 0
turn_state = 0
turn_count = 0


def clamp_s16(v):
    v = int(v)
    if v < -32768:
        return -32768
    if v > 32767:
        return 32767
    return v


def uart_init():
    global uart2
    if not UART_ENABLE:
        return
    fpioa = FPIOA()
    fpioa.set_function(UART2_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART2_RX_PIN, FPIOA.UART2_RXD)
    uart2 = UART(UART.UART2, baudrate=UART_BAUD,
                 bits=UART.EIGHTBITS,
                 parity=UART.PARITY_NONE,
                 stop=UART.STOPBITS_ONE)


def uart_deinit():
    global uart2
    if uart2:
        uart2.deinit()
        uart2 = None


def send_error(error):
    if uart2 is None:
        return
    packet = ustruct.pack(">BBBhB", 0x01, 0x02, 0x03, clamp_s16(error), 0xAA)
    uart2.write(packet)


def draw_text(img, x, y, s, color=(255, 255, 255), size=16):
    try:
        img.draw_string_advanced(x, y, size, s, color=color)
    except Exception:
        pass


def blob_rect(blob):
    try:
        return blob.rect()
    except Exception:
        return (blob[0], blob[1], blob[2], blob[3])


def blob_center(blob):
    try:
        return blob.cx(), blob.cy()
    except Exception:
        return blob[5], blob[6]


def blob_pixels(blob):
    try:
        return blob.pixels()
    except Exception:
        try:
            return blob[4]
        except Exception:
            x, y, w, h = blob_rect(blob)
            return w * h


def to_gray_for_detect(rgb_img):
    try:
        return rgb_img.to_grayscale(copy=True)
    except Exception:
        try:
            return rgb_img.to_grayscale()
        except Exception:
            return rgb_img


def find_best_blob(gray_img, roi):
    try:
        blobs = gray_img.find_blobs([(0, BLACK_THRESH)],
                                    roi=roi,
                                    pixels_threshold=MIN_PIXELS,
                                    area_threshold=MIN_AREA,
                                    merge=MERGE_BLOBS,
                                    margin=8)
    except TypeError:
        blobs = gray_img.find_blobs([(0, BLACK_THRESH)], roi=roi)

    if not blobs:
        return None
    return max(blobs, key=blob_pixels)


def process_frame(rgb_img):
    global last_error, turn_state, turn_count

    gray = to_gray_for_detect(rgb_img)
    if GAUSSIAN_ENABLE:
        try:
            gray.gaussian(1)
        except Exception:
            pass

    center_x = IMG_W // 2
    weighted_sum = 0
    weight_sum = 0
    found_count = 0
    points = []
    roi_centers = [None for _ in ROIS]

    rgb_img.draw_line(center_x, 0, center_x, IMG_H - 1, color=(0, 255, 0), thickness=1)

    for idx, roi in enumerate(ROIS):
        x, y, w, h, weight = roi
        rgb_img.draw_rectangle(x, y, w, h, color=(0, 180, 255), thickness=1)

        blob = find_best_blob(gray, (x, y, w, h))
        if blob is None:
            continue

        bx, by, bw, bh = blob_rect(blob)
        cx, cy = blob_center(blob)

        found_count += 1
        weighted_sum += cx * weight
        weight_sum += weight
        points.append((cx, cy))
        roi_centers[idx] = (cx, cy)

        # Mark the detected black line on RGB display.
        rgb_img.draw_rectangle(bx, by, bw, bh, color=(255, 0, 0), thickness=2)
        rgb_img.draw_cross(cx, cy, color=(255, 0, 0), size=10, thickness=2)

    if weight_sum > 0:
        line_x = int(weighted_sum / weight_sum)
        error = line_x - center_x

        if RIGHT_ANGLE_ENABLE and turn_count <= 0:
            bottom = roi_centers[0]
            upper_found = 0
            for p in roi_centers[2:]:
                if p is not None:
                    upper_found += 1
            if bottom is not None and upper_found == 0:
                bottom_error = bottom[0] - center_x
                if bottom_error > TURN_TRIGGER_OFFSET:
                    turn_state = 1
                    turn_count = TURN_HOLD_FRAMES
                elif bottom_error < -TURN_TRIGGER_OFFSET:
                    turn_state = -1
                    turn_count = TURN_HOLD_FRAMES

        if turn_count > 0:
            error = turn_state * RIGHT_ANGLE_ERR
            turn_count -= 1
        else:
            turn_state = 0

        last_error = error

        rgb_img.draw_cross(line_x, IMG_H - 30, color=(255, 255, 0), size=14, thickness=2)
        rgb_img.draw_line(center_x, IMG_H - 1, line_x, IMG_H - 30,
                          color=(255, 255, 0), thickness=2)
        for i in range(len(points) - 1):
            rgb_img.draw_line(points[i][0], points[i][1],
                              points[i + 1][0], points[i + 1][1],
                              color=(255, 255, 0), thickness=2)
    else:
        error = last_error

    draw_text(rgb_img, 5, 5, "ERR:%d FIND:%d" % (error, found_count),
              color=(255, 255, 255), size=24)
    draw_text(rgb_img, 5, 34, "BIN<=%d T:%d C:%d" % (BLACK_THRESH, turn_state, turn_count),
              color=(255, 255, 255), size=16)
    return error


sensor = None

try:
    print("start temp.py")
    sensor = Sensor(id=SENSOR_ID, width=1280, height=720, fps=90)
    sensor.reset()
    sensor.set_framesize(width=IMG_W, height=IMG_H)
    sensor.set_pixformat(Sensor.RGB888)

    if DISPLAY_MODE == "lcd":
        Display.init(Display.ST7701, to_ide=True, quality=50)
    else:
        Display.init(Display.VIRT, width=IMG_W, height=IMG_H, to_ide=True, quality=50)

    uart_init()
    sensor.run()
    print("sensor run")
    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()
        error = process_frame(img)
        Display.show_image(img)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_send_ms) >= SEND_PERIOD_MS:
            send_error(error)
            last_send_ms = now

        print("fps:%.1f err:%d" % (clock.fps(), error))
        gc.collect()

except KeyboardInterrupt:
    pass
except Exception as e:
    print("error:", e)
finally:
    uart_deinit()
    if sensor:
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)