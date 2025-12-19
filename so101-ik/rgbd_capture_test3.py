#!/usr/bin/env python3
from pathlib import Path
import time
import numpy as np
import cv2
import pyrealsense2 as rs

SERIAL = "838212073725"

out = Path("captures")
out.mkdir(exist_ok=True)

pipeline = rs.pipeline()
config = rs.config()
config.enable_device(SERIAL)

# Jetson-safe profile
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

profile = pipeline.start(config)
time.sleep(0.5)  # let auto-exposure settle a bit

frames = pipeline.wait_for_frames(5000)
color = frames.get_color_frame()
depth = frames.get_depth_frame()

if not color or not depth:
    pipeline.stop()
    raise RuntimeError("Missing color or depth frame")

color_img = np.asanyarray(color.get_data())      # BGR
depth_img = np.asanyarray(depth.get_data())      # uint16

cv2.imwrite(str(out / "color.png"), color_img)
np.save(out / "depth.npy", depth_img),  # if you want raw

# Nice visualization
depth_vis = cv2.convertScaleAbs(depth_img, alpha=0.03)
cv2.imwrite(str(out / "depth_vis.png"), depth_vis)

pipeline.stop()
print("Saved captures/color.png, captures/depth.npy, captures/depth_vis.png")

