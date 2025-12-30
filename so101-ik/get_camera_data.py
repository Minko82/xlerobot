#!/usr/bin/env python3
import pyrealsense2 as rs
import json
from pathlib import Path

# Setup paths
script_dir = Path(__file__).resolve().parent
output_path = script_dir / "captures" / "intrinsics.json"
output_path.parent.mkdir(parents=True, exist_ok=True)  # Ensure directory exists

# 1. Connect to RealSense
pipeline = rs.pipeline()
config = rs.config()
# config.enable_device("838212073725")
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# 2. Start streaming just long enough to get metadata
profile = pipeline.start(config)

# 3. Extract Intrinsics
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

# 4. create a dictionary of the values
intrinsic_data = {
    "width": intrinsics.width,
    "height": intrinsics.height,
    "fx": intrinsics.fx,
    "fy": intrinsics.fy,
    "ppx": intrinsics.ppx,
    "ppy": intrinsics.ppy,
    "model": str(intrinsics.model),
}

# 5. Save to JSON
with open(output_path, "w") as f:
    json.dump(intrinsic_data, f, indent=4)

print(f"Intrinsics saved to {output_path}")

pipeline.stop()
