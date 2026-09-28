from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os,sys,gc,time,random,utime
import ujson
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo
from machine import UART, FPIOA, Pin

# 自定义YOLO检测类，继承自AIBase基类
class YOLOv12App(AIBase):
    def __init__(self, kmodel_path, model_input_size, anchors, confidence_threshold=0.5, nms_threshold=0.2, rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)  # 调用基类的构造函数
        self.class_id = ["seven","one","nine","six","three","eight","four","five","two"]
        # 将英文数字映射为实际数值: "one"->1, "two"->2, ...
        self.digit_map = {"one":1, "two":2, "three":3, "four":4, "five":5, "six":6, "seven":7, "eight":8, "nine":9}
        self.kmodel_path = kmodel_path  # 模型文件路径
        self.model_input_size = model_input_size  # 模型输入分辨率
        self.confidence_threshold = confidence_threshold  # 置信度阈值
        self.nms_threshold = nms_threshold  # NMS（非极大值抑制）阈值
        self.anchors = anchors  # 锚点数据，用于目标检测
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]  # sensor给到AI的图像分辨率，并对宽度进行16的对齐
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]  # 显示分辨率，并对宽度进行16的对齐
        self.debug_mode = debug_mode  # 是否开启调试模式
        self.ai2d = Ai2d(debug_mode)  # 实例化Ai2d，用于实现模型预处理
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)  # 设置Ai2d的输入输出格式和类型

    # 配置预处理操作，这里使用了pad和resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):  # 计时器，如果debug_mode大于0则开启
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size  # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，可以通过设置input_image_size自行修改输入尺寸
            top, bottom, left, right = self.get_padding_param()  # 获取padding参数
            print("padding: {} {} {} {}".format(top, bottom, left, right))
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [104, 117, 123])  # 填充边缘
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)  # 缩放图像
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])  # 构建预处理流程

    # 自定义当前任务的后处理，results是模型输出array列表
    def postprocess(self, results):
        counter = 0
        det_res = []
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 输出形状为[1, 1, 14, 2100]
            # 意思是，输出了2100个框，每个框有14个数据，其中前4个数据是xywh，后面10个是每个类别对应的置信度，这里的xy指的是中心点坐标
            for i in range(2100):
                result = results[0][0][:, i]
                max_score = max(result[4:])
                if max_score > self.confidence_threshold:
                    # 这里把位置信息恢复到1920 * 1080画布下的状态
                    x = result[0] * max(self.rgb888p_size) / max(self.model_input_size)
                    y = result[1] * max(self.rgb888p_size) / max(self.model_input_size)
                    w = result[2] * max(self.rgb888p_size) / max(self.model_input_size)
                    h = result[3] * max(self.rgb888p_size) / max(self.model_input_size)
                    det_res.append([x, y, w, h, list(result[4:]).index(max_score), max_score])
            # 这里还需要一个NMS处理，因为，套框问题比较严重，但是我这里暂时没空写这个东西了，于是简单的按照，每个类别只检测一个物体，按这个逻辑来写
            # 后续可能会把NMS也写在这里，现在暂时不写了
            det_res.sort(key=lambda x:x[-1], reverse=True)
            det_res_single = []
            added_class = []
            for result in det_res:
                if not result[-2] in added_class:
                    added_class.append(result[-2])
                    det_res_single.append(result)
        return det_res_single

    # 绘制检测结果到画面上
    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()  # 清除OSD图像
                for det in dets:
                    # 将检测框的坐标转换为显示分辨率下的坐标
                    x, y, w, h = map(lambda x: int(round(x, 0)), det[:4])
                    x = x * self.display_size[0] // self.rgb888p_size[0]
                    y = y * self.display_size[1] // self.rgb888p_size[1]
                    w = w * self.display_size[0] // self.rgb888p_size[0]
                    h = h * self.display_size[1] // self.rgb888p_size[1]
                    pl.osd_img.draw_rectangle(x - w//2, y - h // 2, w, h, color=(255, 0, 255, 0), thickness=2)  # 绘制矩形框
                    pl.osd_img.draw_string_advanced(x - w//2, y - h // 2, 80, "{} {}".format(self.class_id[det[-2]], round(det[-1], 2)), color=(255, 0, 255, 0)) # 画标签和置信度
            else:
                pl.osd_img.clear()

    # 获取padding参数
    def get_padding_param(self):
        dst_w = self.model_input_size[0]  # 模型输入宽度
        dst_h = self.model_input_size[1]  # 模型输入高度
        ratio_w = dst_w / self.rgb888p_size[0]  # 宽度缩放比例
        ratio_h = dst_h / self.rgb888p_size[1]  # 高度缩放比例
        ratio = min(ratio_w, ratio_h)  # 取较小的缩放比例
        new_w = int(ratio * self.rgb888p_size[0])  # 新宽度
        new_h = int(ratio * self.rgb888p_size[1])  # 新高度
        dw = (dst_w - new_w) / 2  # 宽度差
        dh = (dst_h - new_h) / 2  # 高度差
        top = int(round(0))
        bottom = int(round(dh * 2 + 0.1))
        left = int(round(0))
        right = int(round(dw * 2 - 0.1))
        return top, bottom, left, right

if __name__ == "__main__":
    # 显示模式，默认"hdmi",可以选择"hdmi"和"lcd"
    display_mode="lcd"
    # k230保持不变，k230d可调整为[640,360]
    rgb888p_size = [1920, 1080]

    if display_mode=="hdmi":
        display_size=[1920,1080]
    else:
        display_size=[800,480]
    # 设置模型路径和其他参数
    kmodel_path = "/sdcard/best.kmodel"
    # 其它参数
    confidence_threshold = 0.7
    nms_threshold = 0.2
    anchor_len = None
    det_dim = 4
    anchors_path = None
    anchors = None
    anchors = None

    # 状态机定义 
    STATE_INIT = 0              # 初始状态：等待识别到第一个数字
    STATE_WAIT_TARGET_ACK = 1   # 已发送目标数字，等待主控回复0xC1
    STATE_MONITOR = 2           # 监控状态：检测目标数字的左右位置
    STATE_WAIT_POS_ACK = 3      # 已发送位置帧，等待主控回复0xC1或停止帧
    STATE_STOP = 4              # 停止状态：收到结束指令

    current_state = STATE_INIT
    target_digit = None         # 存储启动时识别到的目标数字
    target_class_id = None      # 目标数字对应的class_id索引（用于后续帧中匹配）

    #  UART 初始化 
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, 115200)

    # 按下复位
    USER_BTN_PIN = 53
    fpioa.set_function(USER_BTN_PIN, FPIOA.GPIO33)  # 物理引脚53 → GPIO33功能
    user_btn = Pin(USER_BTN_PIN, Pin.IN, Pin.PULL_DOWN)  # 下拉模式：按下=1，松开=0
    key_released = True  # 按键释放标志（防止一次按下重复触发）

    def check_btn_reset():
        global key_released, current_state, target_digit, target_class_id
        if user_btn.value() == 1:               # 下拉模式：按下 = 1
            if key_released:                     # 只在新的一次按下时触发
                key_released = False
                utime.sleep_ms(20)               # 消抖延时
                if user_btn.value() == 1:        # 再次确认按下
                    # 执行复位
                    current_state = STATE_INIT
                    target_digit = None
                    target_class_id = None
                    uart.write(bytes([0x33]))
                    print("[按键] 用户按下复位键，回到初始状态")
        else:
            key_released = True                  # 按键释放，允许下次触发

    def process_uart_rx():
        while uart.any():
            b = uart.read(1)[0]
            if b == 0xC1:
                return "C1"
            if b == 0x33:
                return "RESET"

        return None

    # 初始化PipeLine，用于图像处理流程
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()  # 创建PipeLine实例
    # 初始化自定义YOLO检测实例
    yolo_det = YOLOv12App(kmodel_path, model_input_size=[320, 320], anchors=anchors, confidence_threshold=confidence_threshold, nms_threshold=nms_threshold, rgb888p_size=rgb888p_size, display_size=display_size, debug_mode=0)
    yolo_det.config_preprocess()  # 配置预处理

    print("送药小车启动")
    print("状态: 等待识别目标数字...")

    try:
        while True:
            os.exitpoint()                      # 检查是否有退出信号

            # ====== 按键复位检测(按下USER按键，重新开始识别) ======        
            check_btn_reset()

            # ====== 等待状态:只轮询串口,不做检测 ======
            # 避免在检测过程中消耗掉主控发来的0xC1
            if current_state == STATE_WAIT_TARGET_ACK or current_state == STATE_WAIT_POS_ACK:
                rx_result = process_uart_rx()

                # 收到0x33复位帧，执行总复位
                if rx_result == "RESET":
                    current_state = STATE_INIT
                    target_digit = None
                    target_class_id = None
                    uart.write(bytes([0x33]))
                    print("===== [串口] 收到0x33复位指令，回到初始状态 =====")
                    utime.sleep_ms(10)
                    continue

                if current_state == STATE_WAIT_TARGET_ACK and rx_result == "C1":
                    print("收到主控0xC1确认，目标数字={} 已锁定".format(target_digit))
                    current_state = STATE_MONITOR
                    print("状态: 开始监控目标数字的左右位置...")

                elif current_state == STATE_WAIT_POS_ACK and rx_result == "C1":
                    print("收到主控0xC1确认，继续监控...")
                    current_state = STATE_MONITOR

                utime.sleep_ms(10)  # 等待时短暂休眠，避免忙等
                continue
            # ====== 检测状态（INIT / MONITOR）：只做检测，不碰串口 ======
            with ScopedTiming("total",0):
                img = pl.get_frame()            # 获取当前帧数据
                res = yolo_det.run(img)         # 推理当前帧

                if current_state == STATE_INIT:
                    # 初始状态：识别到数字后发送给主控
                    if res:
                        best_det = max(res, key=lambda x: x[-1])
                        class_name = yolo_det.class_id[best_det[-2]]
                        digit_num = yolo_det.digit_map.get(class_name, 0)
                        if digit_num == 0:
                            yolo_det.draw_result(pl, res)
                            pl.show_image()
                            gc.collect()
                            continue
                        # 存储目标数字
                        target_digit = digit_num
                        target_class_id = best_det[-2]  # class_id在class_id列表中的索引
                        # 发送目标数字（单字节）
                        uart.write(bytes([digit_num]))
                        print("识别到目标数字: {}, 发送单字节: {:02X}".format(digit_num, digit_num))
                        current_state = STATE_WAIT_TARGET_ACK
                        print("状态: 等待主控确认(0xC1)...")

                elif current_state == STATE_MONITOR:
                    # 监控状态：在检测结果中查找目标数字，判断左右位置
                    if res:
                        # 找到所有目标数字对应的检测框
                        target_dets = [d for d in res if d[-2] == target_class_id]
                        if target_dets:
                            # 取置信度最高的目标检测框
                            best_target = max(target_dets, key=lambda x: x[-1])
                            center_x = best_target[0]  # 检测框中心x坐标
                            frame_width = rgb888p_size[0]  # 1920
                            half_width = frame_width // 2  # 960

                            if center_x < half_width:
                                # 左半区 -> 发送单字节 0x11
                                uart.write(bytes([0x11]))
                                print("目标数字{}在左半区(x={:.0f}<{})，发送单字节: 11".format(
                                    target_digit, center_x, half_width))
                            else:
                                # 右半区 -> 发送单字节 0x22
                                uart.write(bytes([0x22]))
                                print("目标数字{}在右半区(x={:.0f}>={})，发送单字节: 22".format(
                                    target_digit, center_x, half_width))
                            current_state = STATE_WAIT_POS_ACK
                            print("状态: 等待主控确认(0xC1)")

                # 绘制结果并显示
                yolo_det.draw_result(pl, res)
                pl.show_image()
                gc.collect()                    # 垃圾回收

    except Exception as e:
        print("异常:", e)                  # 打印异常信息
    finally:
        yolo_det.deinit()                       # 反初始化
        pl.destroy()                            # 销毁PipeLine实例
        uart.deinit()                           # 反初始化串口
        print("送药小车停止")
