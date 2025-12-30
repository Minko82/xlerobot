#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import pyrealsense2 as rs
from pathlib import Path

SERIAL = "838212073725"

# Load captured images from captures/ directory
script_dir = Path(__file__).resolve().parent
captures_dir = script_dir / "captures"

# Load color image
color_img = o3d.io.read_image(str(captures_dir / "color.png"))

# Load depth data from numpy file
depth_data = np.load(captures_dir / "depth.npy")

# Convert depth numpy array to Open3D image
depth_img = o3d.geometry.Image(depth_data)

# Create RGBD image
rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_img,
    depth_img,
    depth_scale=1000.0,  # RealSense depth is in millimeters
    depth_trunc=3.0,  # Truncate depth at 3 meters
    convert_rgb_to_intensity=False,
)

# Get camera intrinsics from RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(SERIAL)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

profile = pipeline.start(config)

# Get intrinsics from the color stream
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

pipeline.stop()

# Create Open3D camera intrinsic
o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    width=intrinsics.width,
    height=intrinsics.height,
    fx=intrinsics.fx,
    fy=intrinsics.fy,
    cx=intrinsics.ppx,
    cy=intrinsics.ppy,
)

# Create point cloud from RGBD image
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic)

# Flip the point cloud (RealSense coordinate system)
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

print(f"Point cloud has {len(pcd.points)} points")

# Visualize the point cloud
o3d.visualization.draw_geometries(
    [pcd], window_name="Cube Point Cloud", width=800, height=600, left=50, top=50
)
