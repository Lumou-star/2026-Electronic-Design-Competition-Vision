# 立创·庐山派-K230-CanMV开发板资料与相关扩展板软硬件资料官网全部开源
# 开发板官网：www.lckfb.com
# 技术支持常驻论坛，任何技术问题欢迎随时交流学习
# 立创论坛：www.jlc-bbs.com/lckfb
# 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
# 不靠卖板赚钱，以培养中国工程师为己任

import network
import time

SSID = "XYXY_Student"        # 路由器名称
PASSWORD = "192837465Aa." # 路由器密码

def sta_test():
    # 初始化STA模式（客户端模式）
    sta = network.WLAN(network.STA_IF)

    # 激活WiFi模块（相当于打开手机WIFI开关）
    if not sta.active():  # 判断是否已激活
        sta.active(True)
    print("WiFi模块激活状态:", sta.active())

    # 查看初始连接状态
    print("初始连接状态:", sta.status())

    # 扫描当前环境中的WIFI
    wifi_list = sta.scan()  # 扫描周围WiFi
    # 打印每个Wi-Fi信息
    for wifi in wifi_list:
        # 访问 rt_wlan_info 对象的属性
        ssid = wifi.ssid       # ssid 属性
        rssi = wifi.rssi       # rssi 属性
        print(f"SSID: {ssid}, 信号强度: {rssi}dBm")

    # 尝试连接路由器
    print(f"正在连接 {SSID}...")
    sta.connect(SSID, PASSWORD)

    # 等待连接结果（最多尝试5次）
    max_wait = 5
    while max_wait > 0:
        if sta.isconnected():  # 检查是否连接成功
            break
        max_wait -= 1
        time.sleep(1)  # 失败了就线休息一秒再说
        sta.connect(SSID, PASSWORD)
        print("剩余等待次数：", max_wait, "次")

    # 如果获取不到IP地址就一直在这等待
    while sta.ifconfig()[0] == '0.0.0.0':
        pass

    if sta.isconnected():
        print("\n连接成功！")
        # 重新获取并打印网络配置
        ip_info = sta.ifconfig()
        print(f"IP地址: {ip_info[0]}")
        print(f"子网掩码: {ip_info[1]}")
        print(f"网关: {ip_info[2]}")
        print(f"DNS服务器: {ip_info[3]}")
    else:
        print("连接失败，请检查密码或信号强度")

sta_test()

while True:
    # 持续死循环，等待用户打断并退出该循环
    time.sleep(0.5)  # 等待0.5秒