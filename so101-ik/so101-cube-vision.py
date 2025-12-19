import open3d as o3d
from pathlib import Path

script_dir = Path(__file__).resolve().parent

color_raw = o3d.io.read_image(str(script_dir / "images" / "opencv__dev_video4.png"))
depth_raw = o3d.io.read_image(str(script_dir / "images" / "opencv__dev_video2.png"))

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw, convert_rgb_to_intensity=False
)
