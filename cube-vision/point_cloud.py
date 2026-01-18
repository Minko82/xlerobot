import open3d as o3d
import numpy as np
from pathlib import Path
import json
import matplotlib.pyplot as plt


class PointCloud:
    def __init__(self, captures_dir=None):
        if captures_dir is None:
            self.script_dir = Path(__file__).resolve().parent
            self.captures_dir = self.script_dir / "outputs" / "realsense_capture"
        else:
            self.captures_dir = Path(captures_dir)
        self.ply_path = self.captures_dir / "vision.ply"
        self.pcd = None
        self.inlier_cloud = None
        self.outlier_cloud = None
        self.plane_model = None

    def load_from_ply(self, ply_path):
        ply_path = Path(ply_path)
        if not ply_path.exists():
            raise FileNotFoundError(f"Point cloud file not found: {ply_path}")
        self.pcd = o3d.io.read_point_cloud(str(ply_path))
        print(f"Loaded point cloud with {len(self.pcd.points)} points from {ply_path}")

    def create_point_cloud_from_rgbd(self, scale_depth=1.0, truncate_depth=0.75, min_depth=0.35):
        # Load color image
        color_path = self.captures_dir / "color.png"
        if not color_path.exists():
            raise FileNotFoundError(f"Color image not found: {color_path}")
        color_img = o3d.io.read_image(str(color_path))
        print("Color image loaded")

        # Load depth data from numpy file
        depth_path = self.captures_dir / "depth_meters.npy"
        if not depth_path.exists():
            raise FileNotFoundError(f"Depth data not found: {depth_path}")
        depth_data = np.load(depth_path)
        print("Depth data loaded")
        print(f"Depth range: min={depth_data.min():.4f}, max={depth_data.max():.4f}")
        print(f"Points <= 0.10: {np.sum(depth_data <= 0.10)}")
        print(f"Points between 0.10 and 0.15: {np.sum((depth_data > 0.10) & (depth_data <= 0.15))}")
        depth_data[depth_data <= min_depth] = 0.0
        # Convert depth numpy array to open3d image
        depth_img = o3d.geometry.Image(depth_data.astype(np.float32))
        print("Depth image created")

        # Create RGBD image
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_img,
            depth_img,
            depth_scale=scale_depth,
            depth_trunc=truncate_depth,
            convert_rgb_to_intensity=False,
        )
        print("RGBD image created")

        # Load camera intrinsics
        intrinsic_path = self.captures_dir / "intrinsic_data.json"
        if not intrinsic_path.exists():
            raise FileNotFoundError(f"Intrinsic data not found: {intrinsic_path}")
        with open(intrinsic_path) as f:
            intrinsics = json.load(f)
        print("Intrinsics loaded from JSON")

        # Create Open3D camera intrinsic
        o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=intrinsics["width"],
            height=intrinsics["height"],
            fx=intrinsics["fx"],
            fy=intrinsics["fy"],
            cx=intrinsics["ppx"],
            cy=intrinsics["ppy"],
        )
        print(o3d_intrinsic)

        # Create point cloud from RGBD image
        self.pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic)
        print(f"Point cloud created with {len(self.pcd.points)} points")

    def save_to_ply(self):
        o3d.io.write_point_cloud(str(self.ply_path), self.pcd)
        print(f"Saved point cloud to {self.ply_path}")

    def segment_plane(self, distance_threshold=0.019, ransac_n=3, num_iterations=10000):
        self.plane_model, inliers = self.pcd.segment_plane(
            distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations
        )
        [a, b, c, d] = self.plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        self.pcd = self.pcd.select_by_index(inliers, invert=True)

    def segment_grippers(self):
        pass

    def dbscan_objects(self, min_points_per_object=300):
        labels = np.array(self.pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")

        # Apply colors to visualize clusters
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        self.pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        # Extract coordinates for each object
        points = np.asarray(self.pcd.points)
        objects = []
        for i in range(max_label + 1):
            cluster_mask = labels == i
            cluster_points = points[cluster_mask]
            centroid = cluster_points.mean(axis=0)
            if len(cluster_points) >= min_points_per_object:
                objects.append(
                    {
                        "label": i,
                        "centroid": centroid,
                        "points": cluster_points,
                        "num_points": len(cluster_points),
                    }
                )
                print(f"Object {i}: centroid={centroid}, {len(cluster_points)} points")

        return objects

    def visualize(self, window_name="Point Cloud Visualization"):
        o3d.visualization.draw_geometries([self.pcd], window_name=window_name, width=1024, height=768)


if __name__ == "__main__":
    # Create processor and load/create point cloud
    processor = PointCloud()

    processor.create_point_cloud_from_rgbd(scale_depth=1.0, truncate_depth=0.75)

    # Segment the table plane
    processor.segment_plane(distance_threshold=0.019)
    processor.dbscan_objects()
    processor.save_to_ply()

    # processor.dbscan_objects()
    # Visualize the segmentation
    processor.visualize()
