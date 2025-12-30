#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import pyrealsense2 as rs
from pathlib import path
import json

serial = "838212073725"

# load captured images from captures/ directory
script_dir = path(__file__).resolve().parent
captures_dir = script_dir / "outputs" /"realsense_capture"
print("captured images loaded.")

# load color image
color_img = o3d.io.read_image(str(captures_dir / "color.png"))
print("color imaged loaded.")
# load depth data from numpy file
depth_data = np.load(captures_dir / "depth_meters.npy")
print("depth data loaded")
# convert depth numpy array to open3d image
depth_img = o3d.geometry.image(depth_data)
print("depth image created.")
# create rgbd image
rgbd = o3d.geometry.rgbdimage.create_from_color_and_depth(
    color_img,
    depth_img,
    depth_scale=1.0,
    depth_trunc=3.0,  # truncate depth at 3 meters
    convert_rgb_to_intensity=false,
)
print("rgbd image created.")

with open(captures_dir / "intrinsic_data.json", "r") as f:
        intrinsics = json.load(f)
print("intrinsics loaded from json.")

# create open3d camera intrinsic
o3d_intrinsic = o3d.camera.pinholecameraintrinsic(
    width=intrinsics["width"],
    height=intrinsics["height"],
    fx=intrinsics["fx"],
    fy=intrinsics["fy"],
    cx=intrinsics["ppx"],
    cy=intrinsics["ppy"],
)
print("intrinsic created.")
# create point cloud from rgbd image
pcd = o3d.geometry.pointcloud.create_from_rgbd_image(rgbd, o3d_intrinsic)
print("point cloud created")
# flip the point cloud (realsense coordinate system)
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
print("point cloud flipped since original image is upside down.")
print(f"point cloud has {len(pcd.points)} points")
print("visualizing point cloud")
# save the point cloud
out_ply = captures_dir / "vision.ply"
o3d.io.write_point_cloud(str(out_ply), pcd)
print(f"saved point cloud to {out_ply}")out_ply = captures_dir / "cube.ply"
