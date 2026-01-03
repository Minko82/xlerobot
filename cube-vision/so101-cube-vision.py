#!/usr/bin/env python3
import open3d as o3d
import numpy as np
from pathlib import Path
import json


class CubeVision:
    def __init__(self):
        self.script_dir = Path(__file__).resolve().parent
        self.captures_dir = self.script_dir / "outputs" / "realsense_capture"
        self.inlier_cloud = None
        self.outlier_cloud = None
        self.pcd = None

    def create_point_cloud(self, scale_depth=1.0, truncate_depth=1.0):
        # load color image
        color_path = self.captures_dir / "color.png"
        if not color_path.exists():
            raise FileNotFoundError(f"Color image not found: {color_path}")
        color_img = o3d.io.read_image(str(color_path))
        print("color imaged loaded.")
        # load depth data from numpy file
        depth_path = self.captures_dir / "depth_meters.npy"
        if not depth_path.exists():
            raise FileNotFoundError(f"Depth data not found: {depth_path}")
        depth_data = np.load(depth_path)
        print("depth data loaded")
        # convert depth numpy array to open3d image
        depth_img = o3d.geometry.Image(depth_data.astype(np.float32))
        print("depth image created.")
        # create rgbd image
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_img,
            depth_img,
            depth_scale=scale_depth,
            depth_trunc=truncate_depth,
            convert_rgb_to_intensity=False,
        )
        print("rgbd image created.")

        intrinsic_path = self.captures_dir / "intrinsic_data.json"
        if not intrinsic_path.exists():
            raise FileNotFoundError(f"Intrinsic data not found: {intrinsic_path}")
        with open(intrinsic_path) as f:
            intrinsics = json.load(f)
        print("intrinsics loaded from json.")

        # create open3d camera intrinsic
        o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=intrinsics["width"],
            height=intrinsics["height"],
            fx=intrinsics["fx"],
            fy=intrinsics["fy"],
            cx=intrinsics["ppx"],
            cy=intrinsics["ppy"],
        )
        print(o3d_intrinsic)
        print("intrinsic created.")
        # create point cloud from rgbd image
        self.pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic)
        print("point cloud created")
        # flip the point cloud (realsense coordinate system)
        # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        print(f"point cloud has {len(self.pcd.points)} points")
        print("visualizing point cloud")
        # save the point cloud
        out_ply = self.captures_dir / "vision.ply"
        o3d.io.write_point_cloud(str(out_ply), self.pcd)
        print(f"saved point cloud to {out_ply}")

    def segment_out_plane(self):
        ply_path = self.captures_dir / "vision.ply"
        if not ply_path.exists():
            raise FileNotFoundError(f"Point cloud file not found: {ply_path}")
        self.pcd = o3d.io.read_point_cloud(str(ply_path))
        plane_model, inliers = self.pcd.segment_plane(
            distance_threshold=0.018, ransac_n=3, num_iterations=1000
        )

        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        self.inlier_cloud = self.pcd.select_by_index(inliers)
        self.outlier_cloud = self.pcd.select_by_index(inliers, invert=True)
        self.inlier_cloud.paint_uniform_color([0, 0.3, 1.0])  # Blue-ish plane

    def visualize_plane(self):
        if self.inlier_cloud is None or self.outlier_cloud is None:
            raise RuntimeError("Must call segment_out_plane() before visualize_plane()")
        o3d.visualization.draw_geometries([self.inlier_cloud, self.outlier_cloud])


if __name__ == "__main__":
    cube_vision = CubeVision()
    cube_vision.create_point_cloud()
    cube_vision.segment_out_plane()

    # Comment out to remove visualization
    cube_vision.visualize_plane()
