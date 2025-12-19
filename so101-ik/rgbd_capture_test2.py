from pathlib import Path
import numpy as np
from PIL import Image

from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.configs import ColorMode


def main():
    out = Path("captures")
    out.mkdir(exist_ok=True)

    cfg = RealSenseCameraConfig(
        serial_number_or_name="838212073725",
        color_mode=ColorMode.RGB,
        width=640,
        height=480,
        fps=15,          # important
        use_depth=True,  # important
    )

    cam = RealSenseCamera(cfg)

    cam.connect(warmup=False)

    # Read RGB and depth separately
    try:
        rgb = cam.read()
        depth = cam.read_depth()
    except Exception as e:
        cam.disconnect()
        raise RuntimeError(f"Failed to capture frames: {e}")

    Image.fromarray(rgb).save(out / "color.png")
    np.save(out / "depth.npy", depth)

    cam.disconnect()
    print("Saved RGB + depth")


if __name__ == "__main__":
    main()

