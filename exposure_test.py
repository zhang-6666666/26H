"""
GC2093 曝光测试
"""
from media.sensor import *
from media.display import *
from media.media import *
import time

CAM_W = 640
CAM_H = 480

sensor = Sensor(width=CAM_W, height=CAM_H)
sensor.reset()
sensor.set_framesize(width=CAM_W, height=CAM_H)
sensor.set_pixformat(Sensor.RGB565)

# auto_exposure 必须在 run() 之前
sensor.auto_exposure(False)

Display.init(Display.ST7701, width=CAM_W, height=CAM_H, to_ide=True)
MediaManager.init()
sensor.run()
time.sleep(0.5)

# 遍历曝光值，实时显示
for exp in (50000, 100000, 200000):
    sensor.exposure(exp)
    time.sleep(0.3)
    img = sensor.snapshot()
    img.draw_string_advanced(2, 2, 32, "EXP:%d" % exp, color=(255,255,255))
    Display.show_image(img)
    time.sleep(1.0)

sensor.stop()
MediaManager.deinit()
print("done")
