
# 🦾 XLeRobot

**XLeRobot** is a customized version of [🤗 LeRobot](https://github.com/huggingface/lerobot) tailored for easier setup and use on both Mac and Linux systems.
It provides additional setup guidance, calibration steps, and practical improvements for getting your robot up and running quickly.

<br>

---

## 🚀 Installation & Setup

If you prefer to follow the original instructions, see:
🔗 [LeRobot Installation Docs](https://huggingface.co/docs/lerobot/installation)
🔗 [XLeRobot Installation Docs](https://xlerobot.readthedocs.io/en/latest/software/getting_started/install.html)

<br>

---

### 1. Install Conda

**Mac:**
```bash
curl -LO https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh
bash Miniconda3-latest-MacOSX-arm64.sh
```

**Linux:**
```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh
bash ~/miniconda.sh
```

Then **open a new terminal**.

<br>

---

### 2. Create the Environment

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh

conda create -y -n lerobot python=3.10
conda activate lerobot
conda install ffmpeg -c conda-forge
```

✅ **Tip:** Run `conda activate lerobot` every time you open a new terminal.

<br>

---

### 3. Install the XLe Robot

```bash
git clone https://github.com/Minko82/xle-robot.git
cd xle-robot
pip install -e .
pip install 'lerobot[all]'
pip install -e ".[feetech]"
```

#### If you see build errors (Linux only): 

If you encounter build errors, you may need to install additional dependencies. To install these for linux run:

```bash
sudo apt-get install cmake build-essential python-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev pkg-config
```

<br>

---
### 4. Arm Setup

We’ll assign fixed USB names to each arm so they remain consistent (`/dev/xle_right` and `/dev/xle_left`).

1. **Plug in only the right arm’s control board**, then run:
   ```bash
   udevadm info -a -n /dev/ttyACM0 | grep 'ATTRS{serial}'
   ```
   Example output:
   ```
   ATTRS{serial}=="A50285B1"
   ```
   Copy that serial number.

2. **Unplug the right arm, plug in the left arm**, and repeat to get its serial.

3. **Create a new rules file:**
   ```bash
   sudo nano /etc/udev/rules.d/99-so100-robot.rules
   ```

   Paste this (replace serials with yours):
   ```
   # Right Arm
   SUBSYSTEM=="tty", ATTRS{serial}=="YOUR_SERIAL_FOR_ARM_1", SYMLINK+="xle_right"

   # Left Arm
   SUBSYSTEM=="tty", ATTRS{serial}=="YOUR_SERIAL_FOR_ARM_2", SYMLINK+="xle_left"
   ```

4. **Apply the rules:**
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

5. **Copy calibration files:**
   ```bash
   cp left_arm.json right_arm.json ~/.cache/huggingface/lerobot/calibration/robots/
   ```

<br>

---

<details>
<summary><b>📸 Wrist Cameras Setup (click to expand)</b></summary>

<br>

### Setup Overview

1. **Find Camera Indices**
   ```bash
   v4l2-ctl --list-devices
   ```
   Example output:
   ```
   USB Camera (usb-0000:00:1a.0-1.2):
       /dev/video0
       /dev/video1
   ```
   The numbers (0, 1, etc.) are your camera indices.

2. **Install OpenCV**
   ```bash
   pip install opencv-python
   ```

3. **Update Example Script**
   Edit `examples/9_dual_wrist_camera.py` and set:
   ```python
   CAMERA_INDEX_1 = 0
   CAMERA_INDEX_2 = 1
   ```
   to match your detected camera indices.

</details>

---

<details>
<summary><b>🟦 RealSense Camera Setup (click to expand)</b></summary>

<br>

### 1. Install the SDK
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev
```

### 2. Install the Python Package
```bash
pip install pyrealsense2
```

### 3. Verify the Camera
```bash
realsense-viewer
```
✅ You should see both **color** and **depth** video streams.  
   -  If not, check your USB connection or reinstall the SDK.

</details>

<br>

---

### 5. Connect and Power Everything

1. Plug the **USB hub** into your laptop.  
2. Connect all four devices:
   - Two **wrist cameras**
   - Two **arm control boards**
3. _Optional:_ Plug the **RealSense camera** directly into your laptop.  
4. Turn on the power and verify that all indicator lights are active.

***⚠️ Important:*** If motors become unresponsive after a failure, unplug and reconnect their **motor power cables** to reset them.

<br>

---

### 6. Check Connected Devices

Use:
```bash
lerobot-find-port
```

Example output:
```
right /dev/xle_right
left  /dev/xle_left
```
<br>

---

### 7. Calibrating the Robot (Run once per device)

You need to calibrate your robot only **once per device**.

**Follower Arm**
```bash
lerobot-calibrate     --robot.type=so101_follower     --robot.port=/dev/xle_right
```

**Leader Arm**
```bash
lerobot-calibrate     --teleop.type=so101_leader     --teleop.port=/dev/xle_left
```

<br>

---

### 8. Run example code

Navigate to the example folder and run a script:
```bash
cd examples
python3 0_so100_keyboard_joint_control.py
```

Compatible example scripts are available in the `examples` folder. Additional scripts can be found in `examples/provided_examples`, but these have not yet been tested for full compatibility with XLeRobot.

<br>

---

## 💡 Credits

This project builds on top of [🤗 LeRobot](https://github.com/huggingface/lerobot) by Hugging Face Robotics and [XLeRobot](https://github.com/Vector-Wangel/XLeRobot).

Additional development, hardware integration, and testing have been contributed by [@Minko82](https://github.com/Minko82) and [@nanasci](https://github.com/nanasci).

<br>

---

## 📜 License

Apache License 2.0 — see the [LICENSE](./LICENSE) file for details.
