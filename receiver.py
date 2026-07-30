"""
K230 无线图传 PC 接收端（Python + OpenCV）
用法：
  1. pip install opencv-python numpy
  2. 修改 UDP_IP 为本机 IP
  3. python receiver.py
  4. 按 ESC 退出
"""
import socket
import numpy as np
import cv2
import time

# ===================== 配置 =====================
UDP_IP = "192.168.1.100"        # ← 改成本机 IP，和 K230 端 RECEIVER_IP 一致
UDP_PORT = 8080

MAGIC = 0xABCD1234
HEADER_SIZE = 8                 # 4字节魔数 + 4字节长度

# ===================== 初始化 Socket =====================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(5)              # 5秒超时，方便退出

print(f"等待 K230 视频流... UDP {UDP_IP}:{UDP_PORT}")

frame_count = 0
start_time = time.time()
fps = 0

try:
    while True:
        # 读取 8 字节包头
        try:
            data, addr = sock.recvfrom(HEADER_SIZE)
        except socket.timeout:
            print("超时：未收到数据，等待中...")
            continue

        if len(data) != HEADER_SIZE:
            continue

        # 解析包头
        magic = int.from_bytes(data[:4], 'big')
        if magic != MAGIC:
            continue

        img_size = int.from_bytes(data[4:8], 'big')
        if img_size <= 0 or img_size > 500_000:  # 不合理的大小跳过
            continue

        # 接收图像数据
        buffer = bytearray()
        while len(buffer) < img_size:
            try:
                chunk, _ = sock.recvfrom(min(4096, img_size - len(buffer)))
                buffer.extend(chunk)
            except socket.timeout:
                break

        if len(buffer) < img_size:
            continue

        # JPEG 解码 + 显示
        img = cv2.imdecode(np.frombuffer(buffer[:img_size], np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            cv2.imshow("K230 Video", img)

            # FPS 统计
            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                print(f"分辨率: {img.shape[1]}x{img.shape[0]}  FPS: {fps:.1f}")
                frame_count = 0
                start_time = time.time()

        if cv2.waitKey(1) & 0xFF == 27:   # ESC 退出
            break

except KeyboardInterrupt:
    pass
finally:
    sock.close()
    cv2.destroyAllWindows()
    print("接收端已关闭")
