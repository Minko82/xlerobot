"""Display the most recent captured point cloud (outputs/realsense_capture/vision.ply).

Usage:
    python diagnostics/visualize_point_cloud.py
"""

from pathlib import Path

import open3d as o3d

ply_path = Path("outputs") / "realsense_capture" / "vision.ply"

print(f"Loading point cloud from: {ply_path}")
pcd = o3d.io.read_point_cloud(str(ply_path))

print(f"Point cloud has {len(pcd.points)} points")
print(f"Point cloud has colors: {pcd.has_colors()}")

print("Displaying point cloud...")
o3d.visualization.draw_geometries(
    [pcd], window_name="Point Cloud Visualization", width=1024, height=768, point_show_normal=False
)
