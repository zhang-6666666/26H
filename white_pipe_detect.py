"""
K230 Mini — 白色水管 ROI + LAB 钢珠识别 + 手动曝光
"""

from media.sensor import *
from media.display import *
from media.media import *
import time

# ==================== 图像参数 ====================
FRAME_W = 640
FRAME_H = 480

# ==================== 手动曝光（CanMV v1.8 原生 API） ====================
EXPOSURE_US = 16000           # 曝光 (µs)，画面暗就加大

# ==================== 白色水管阈值（LAB） ====================
PIPE_THRESHOLD = (28, 52, -70, 11, -23, 9)

# ==================== 钢珠阈值（LAB — 金属灰） ====================
BALL_THRESHOLD = (25, 55, -30, 13, -23, 20)

# ==================== 过滤 & ROI 参数 ====================
PIPE_MIN_AREA = 800
PIPE_MIN_PIXELS = 800
BALL_MIN_AREA = 30
BALL_MIN_PIXELS = 20
ROI_EXPAND = 0.20


def expand_roi(x, y, w, h, ratio):
    dw = int(w * ratio / 2)
    dh = int(h * ratio / 2)
    rx = max(0, x - dw)
    ry = max(0, y - dh)
    rw = min(FRAME_W - rx, w + dw * 2)
    rh = min(FRAME_H - ry, h + dh * 2)
    return rx, ry, rw, rh


def init():
    sensor = Sensor(width=FRAME_W, height=FRAME_H)
    sensor.reset()
    sensor.set_framesize(width=FRAME_W, height=FRAME_H)
    sensor.set_pixformat(Sensor.RGB565)

    sensor.auto_exposure(True)

    Display.init(Display.ST7701, width=FRAME_W, height=FRAME_H, to_ide=True)
    MediaManager.init()
    sensor.run()
    time.sleep(0.3)
    return sensor


def main():
    sensor = init()
    # sensor.exposure(EXPOSURE_US)

    clock = time.clock()

    try:
        while True:
            clock.tick()

            img = sensor.snapshot()

            # ---- 1. 查找白色水管 ----
            blobs = img.find_blobs(
                [PIPE_THRESHOLD],
                area_threshold=PIPE_MIN_AREA,
                pixels_threshold=PIPE_MIN_PIXELS,
                merge=True, margin=10
            )

            if blobs:
                largest = blobs[0]
                for b in blobs:
                    if b[4] > largest[4]:
                        largest = b
                x, y, w, h = largest[0], largest[1], largest[2], largest[3]

                # ---- ROI 外扩 20% ----
                rx, ry, rw, rh = expand_roi(x, y, w, h, ROI_EXPAND)

                # ---- 画扩展 ROI ----
                img.draw_rectangle(rx, ry, rw, rh, color=(255, 255, 0), thickness=2)

                # ---- ROI 内 LAB 色块检测钢珠 ----
                balls = img.find_blobs(
                    [BALL_THRESHOLD],
                    roi=(rx, ry, rw, rh),
                    area_threshold=BALL_MIN_AREA,
                    pixels_threshold=BALL_MIN_PIXELS,
                    merge=True, margin=3
                )

                if balls:
                    # 只取面积最大的钢珠
                    best = balls[0]
                    for b in balls:
                        if b[4] > best[4]:
                            best = b

                    bx, by = best[5], best[6]
                    br = max(best[2], best[3]) // 2

                    img.draw_circle(bx, by, br, color=(255, 0, 0), thickness=2, fill=False)
                    img.draw_cross(bx, by, color=(255, 0, 0), size=5, thickness=1)

            # ---- FPS ----
            img.draw_string_advanced(2, 2, 16, "FPS:%.1f" % clock.fps(),
                                     color=(255,255,255))
            Display.show_image(img)

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        sensor.stop()
        MediaManager.deinit()
        print("释放完毕")

main()
