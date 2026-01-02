#!/usr/bin/env python3
import open3d as o3d
import numpy as np
from pathlib import Path
import json

script_dir = Path(__file__).resolve().parent
captures_dir = script_dir / "outputs" / "realsense_capture"


class CubeVision:
    def __init__(self):
        self.script_dir = Path(__file__).resolve().parent
        self.captures_dir = script_dir / "outputs" / "realsense_capture"

    def create_point_cloud(self, scale_depth=1.0, truncate_depth=1.0):
        # load color image
        color_img = o3d.io.read_image(str(self.captures_dir / "color.png"))
        print("color imaged loaded.")
        # load depth data from numpy file
        depth_data = np.load(captures_dir / "depth_meters.npy")
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

        with open(captures_dir / "intrinsic_data.json") as f:
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
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic)
        print("point cloud created")
        # flip the point cloud (realsense coordinate system)
        # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        print(f"point cloud has {len(pcd.points)} points")
        print("visualizing point cloud")
        # save the point cloud
        out_ply = captures_dir / "vision.ply"
        o3d.io.write_point_cloud(str(out_ply), pcd)
        print(f"saved point cloud to {out_ply}")

    def segment_out_plane(self):
        pcd = o3d.io.read_point_cloud(str(captures_dir / "vision.ply"))
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.018, ransac_n=3, num_iterations=1000)

        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        inlier_cloud.paint_uniform_color([0, 0.3, 1.0])  # Red plane
        o3d.io.write_point_cloud(str(captures_dir / "vision.ply"), pcd)
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


if __name__ == "__main__":
    cube_vision = CubeVision()
    cube_vision.create_point_cloud()
    cube_vision.segment_out_plane()
