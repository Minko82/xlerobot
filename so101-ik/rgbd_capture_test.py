#!/usr/bin/env python3
from pathlib import Path

import numpy as np
from PIL import Image

from lerobot.cameras.configs import ColorMode
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig


def save_rgb(p: Path, rgb: np.ndarray):
    # Expect rgb shape (H, W, 3), dtype uint8
    Image.fromarray(rgb.astype(np.uint8), mode="RGB").save(str(p))


def save_depth_viz(p: Path, depth: np.ndarray):
    """
    depth is typically uint16 Z16 in *depth units* (camera units).
    This makes a viewable 8-bit image by clipping to 2m and scaling.
    """
    depth_f = depth.astype(np.float32)
    max_maybe = np.percentile(depth_f[depth_f > 0], 95) if np.any(depth_f > 0) else 1.0
    denom = max(max_maybe, 1.0)
    depth_8 = np.clip(depth_f / denom, 0.0, 1.0) * 255.0
    Image.fromarray(depth_8.astype(np.uint8), mode="L").save(str(p))


def main():
    out = Path("captures")
    out.mkdir(parents=True, exist_ok=True)

    serial = "838212073725"  # your D435

    cfg = RealSenseCameraConfig(
        serial_number_or_name=serial,
        color_mode=ColorMode.RGB,
        # If your config supports setting resolution/fps, you can add them here.
        # On Xavier, 640x480 @ 15 is usually safe for RGB-D.
    )

    cam = RealSenseCamera(cfg)

    cam.connect(warmup=False)
    try:
        frame = cam.read()

        # LeRobot sometimes returns dicts like {"rgb":..., "depth":...}
        # Your earlier script treated cam.read() as an image array; RealSense usually gives RGB-D.
        if isinstance(frame, dict):
            rgb = frame.get("rgb") or frame.get("color") or frame.get("image")
            depth = frame.get("depth")

            if rgb is None:
                raise RuntimeError(f"Got dict frame but no rgb keys found: {list(frame.keys())}")

            save_rgb(out / "color.png", rgb)

            if depth is not None:
                np.save(out / "depth.npy", depth)
                save_depth_viz(out / "depth.png", depth)
                print("Saved: color.png, depth.png, depth.npy")
            else:
                print("Saved: color.png (no depth in frame dict)")
        else:
            # If your RealSenseCamera is configured RGB-only, you may just get an RGB array.
            save_rgb(out / "color.png", frame)
            print("Saved: color.png (frame was not a dict)")

    finally:
        cam.disconnect()


if __name__ == "__main__":
    main()

