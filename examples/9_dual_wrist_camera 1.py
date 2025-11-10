import cv2

# --- CONFIGURATION ---
# Indices from your v4l2-ctl output
CAMERA_INDEX_1 = 2  
CAMERA_INDEX_2 = 4

# New resolution settings (1280x720)
WIDTH = 640
HEIGHT = 480
# ---------------------

print(f"Opening camera {CAMERA_INDEX_1} and {CAMERA_INDEX_2}...")

# Create 'VideoCapture' objects for each camera
cap1 = cv2.VideoCapture(CAMERA_INDEX_1)
cap2 = cv2.VideoCapture(CAMERA_INDEX_2)

# Check if cameras opened successfully
if not cap1.isOpened():
    print(f"Error: Could not open camera {CAMERA_INDEX_1}.")
    exit()
if not cap2.isOpened():
    print(f"Error: Could not open camera {CAMERA_INDEX_2}.")
    exit()

print(f"Setting resolution to {WIDTH}x{HEIGHT} and codec to MJPEG...")

# Set resolution for camera 1
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
# Set codec to MJPEG for camera 1 (Reduces bandwidth)
cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

# Set resolution for camera 2
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
# Set codec to MJPEG for camera 2 (Reduces bandwidth)
cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

print("Cameras opened successfully. Press 'q' to quit.")

while True:
    # Read a frame from each camera
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    # If reading a frame was successful, show it
    if ret1:
        cv2.imshow('Wrist Camera 1 (Index 2)', frame1)
    if ret2:
        cv2.imshow('Wrist Camera 2 (Index 4)', frame2)

    # Check for the 'q' key to be pressed.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When the loop breaks, release the cameras and close windows
print("Cleaning up and closing...")
cap1.release()
cap2.release()
cv2.destroyAllWindows()
