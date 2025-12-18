from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
from lerobot.cameras import ColorMode
from PIL import Image


# Create camera configuration
config = RealSenseCameraConfig(
    serial_number_or_name="video2", color_mode=ColorMode.RGB, width=640, height=480, fps=30
)

# Connect to camera
camera = RealSenseCamera(config)
camera.connect()

# Capture a single image
image = camera.read()

# Save the image
img_pil = Image.fromarray(image, mode="RGB")
img_pil.save("captured_image.png")

# Cleanup
camera.disconnect()
