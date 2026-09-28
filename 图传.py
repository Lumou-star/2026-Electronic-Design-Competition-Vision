# K230 无线图传 AP 热点模式 —— v3：MJPEG 流（长连接推帧，零延迟）
from media.sensor import *
from media.display import *
from media.media import *
import time, os, network, usocket, gc

AP_SSID     = "gxxy"
AP_PASSWORD = "99999999"
PORT        = 80
JPEG_QUALITY = 25       # WiFi 图传：体积优先
CAMERA_START_DELAY_MS = 1000
CAMERA_RETRY_DELAY_MS = 100
CAMERA_CAPTURE_RETRIES = 50

MJPEG_BOUNDARY = "--K230_FRAME"
MJPEG_HEADER = (
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=K230_FRAME\r\n"
    "Connection: close\r\n"
    "\r\n"
).encode()


def send_all(sock, data):
    total = 0
    while total < len(data):
        try:
            sent = sock.send(data[total:])
            if sent <= 0:
                return False
            total += sent
        except OSError:
            return False
    return True


def build_html():
    body = (
        "<!DOCTYPE html>\n"
        "<html><head><meta charset=\"UTF-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>K230</title></head>\n"
        "<body style=\"margin:0;background:#1a1a2e;text-align:center\">\n"
        "<h3 style=\"color:#eee;padding-top:8px\">K230 实时图传 (MJPEG)</h3>\n"
        "<img src=\"/stream\""
        " style=\"width:100%;max-width:640px;border:2px solid #333\">\n"
        "<p style=\"color:#888;font-size:12px\">MJPEG 长连接流，低延迟实时画面</p>\n"
        "</body></html>"
    )
    body_bytes = body.encode()
    header = ("HTTP/1.1 200 OK\r\n"
              "Content-Type: text/html; charset=utf-8\r\n"
              "Content-Length: %d\r\n"
              "Connection: close\r\n"
              "\r\n") % len(body_bytes)
    return header.encode() + body_bytes


def build_mjpeg_part(jpeg_data):
    jpeg_len = len(jpeg_data)
    return (
        MJPEG_BOUNDARY + "\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: " + str(jpeg_len) + "\r\n"
        "\r\n"
    ).encode() + jpeg_data + b"\r\n"


# ====== 1. 初始化摄像头 ======
def capture_frame(retries=CAMERA_CAPTURE_RETRIES):
    """Retry transient channel-0 capture failures on camera startup."""
    last_error = None
    for attempt in range(retries):
        try:
            return sensor.snapshot(chn=CAM_CHN_ID_0)
        except RuntimeError as error:
            last_error = error
            if attempt == 0 or (attempt + 1) % 10 == 0:
                print("camera capture retry %d/%d: %s" %
                      (attempt + 1, retries, error))
            time.sleep_ms(CAMERA_RETRY_DELAY_MS)
    raise last_error


def format_elapsed_time(elapsed_ms):
    total_seconds = max(0, elapsed_ms // 1000)
    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    return "TIME %02d:%02d:%02d" % (hours, minutes, seconds)


def draw_stream_timer(img, elapsed_ms):
    """Burn the timer into both the LCD frame and MJPEG stream."""
    timer_text = format_elapsed_time(elapsed_ms)
    img.draw_rectangle(596, 8, 196, 34, color=(0, 0, 0), fill=True)
    img.draw_string_advanced(604, 12, 20, timer_text, color=(255, 255, 255))


print("=" * 40)
print("[1/3] 初始化摄像头...")
gc.collect()
sensor = Sensor(id=2)
sensor.reset()
sensor.set_framesize(chn=CAM_CHN_ID_0, width=800, height=480)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
Display.init(Display.ST7701, to_ide=True)
MediaManager.init()
sensor.run()
time.sleep_ms(CAMERA_START_DELAY_MS)
gc.collect()

for _ in range(5):
    img = capture_frame()
    Display.show_image(img)
    time.sleep_ms(50)
print("  摄像头就绪")

# ====== 2. 建立 AP 热点 ======
print("")
print("[2/3] 建立 AP 热点...")
try:
    ap = network.WLAN(network.AP_IF)
    ap.config(ssid=AP_SSID, key=AP_PASSWORD)
    server_ip = ap.ifconfig()[0]
except Exception:
    ap.config("ssid", AP_SSID)
    ap.config("key", AP_PASSWORD)
    server_ip = ap.ifconfig()[0]
print("  IP: %s  SSID: %s  密码: %s" % (server_ip, AP_SSID, AP_PASSWORD))
print("  >>> 浏览器打开 http://%s" % server_ip)

# ====== 3. MJPEG 流服务 ======
print("")
print("[3/3] 启动 MJPEG 流服务...")
s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
s.setsockopt(usocket.SOL_SOCKET, usocket.SO_REUSEADDR, 1)
addr = usocket.getaddrinfo(server_ip, PORT)[0][-1]
s.bind(addr)
s.listen(2)
s.setblocking(False)
print("  监听 %s:%d  (MJPEG 模式)" % (server_ip, PORT))
print("=" * 40)

HTML_DATA = build_html()

stream_client = None
last_jpeg = None
clock = time.clock()
html_count = 0
stream_count = 0
stream_frame = 0
err_count = 0
last_fps_print = time.ticks_ms()
timer_start_ms = time.ticks_ms()

try:
    while True:
        os.exitpoint()

        # ----- 抓帧 + 显示 -----
        img = capture_frame()
        clock.tick()
        draw_stream_timer(img, time.ticks_diff(time.ticks_ms(), timer_start_ms))
        Display.show_image(img)
        last_jpeg = bytes(img.compress(quality=JPEG_QUALITY))

        # ----- MJPEG 流推送 -----
        if stream_client is not None:
            part = build_mjpeg_part(last_jpeg)
            if send_all(stream_client, part):
                stream_frame += 1
            else:
                try: stream_client.close()
                except: pass
                stream_client = None
                print("[STREAM] 客户端断开（共推 %d 帧）" % stream_frame)
                stream_frame = 0

        # ----- fps 统计 -----
        now = time.ticks_ms()
        if time.ticks_diff(now, last_fps_print) >= 2000:
            fps = clock.fps()
            jpg_kb = len(last_jpeg) / 1024.0 if last_jpeg else 0
            sc = "ON(%d帧)" % stream_frame if stream_client else "OFF"
            print("  fps=%.1f  stream=%s  jpeg=%.1fKB | html=%d err=%d" % (
                fps, sc, jpg_kb, html_count, err_count))
            last_fps_print = now

        # ----- 接受新连接 -----
        try:
            cl, caddr = s.accept()
        except OSError:
            if stream_client is None:
                time.sleep_ms(5)
            gc.collect()
            continue

        # ----- 读请求 -----
        try:
            cl.settimeout(0.3)
            req = cl.recv(512).decode()
        except Exception:
            try: cl.close()
            except: pass
            continue

        # ----- 路由 -----
        try:
            if "/stream" in req:
                if stream_client is not None:
                    try: stream_client.close()
                    except: pass
                stream_client = cl
                stream_count += 1
                stream_frame = 0
                print("[STREAM #%d] 新客户端 %s（MJPEG 长连接）" % (
                    stream_count, caddr))
                send_all(cl, MJPEG_HEADER)
            else:
                html_count += 1
                print("[HTTP] 页面 #%d 来自 %s" % (html_count, caddr))
                send_all(cl, HTML_DATA)
                time.sleep_ms(50)
                try: cl.close()
                except: pass
        except Exception as e:
            err_count += 1
            try: cl.close()
            except: pass

        gc.collect()

except KeyboardInterrupt:
    print("\n用户中断...")
finally:
    if stream_client is not None:
        try: stream_client.close()
        except: pass
    sensor.stop()
    time.sleep_ms(200)
    s.close()
    Display.deinit()
    MediaManager.deinit()
    gc.collect()
    print("程序已退出")
