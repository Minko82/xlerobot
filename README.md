
# 🦾 XLeRobot

**XLeRobot** is a customized version of [🤗 LeRobot](https://github.com/huggingface/lerobot) tailored for easier setup and use on both Mac and Linux systems.
It provides additional setup guidance, calibration steps, and practical improvements for getting your robot up and running quickly.

---

## 🚀 Installation & Setup

If you prefer to follow the original instructions, see:
🔗 [LeRobot Installation Docs](https://huggingface.co/docs/lerobot/installation)
🔗 [XLeRobot Installation Docs](https://xlerobot.readthedocs.io/en/latest/software/getting_started/install.html)


### 1. Installing Conda

**For Mac:**

```bash
curl -LO https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh
bash Miniconda3-latest-MacOSX-arm64.sh
```

**For Linux:**

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh
bash ~/miniconda.sh
```

After installation, **open a new terminal**.

---

### 2. Environment Setup

Clone this repository:
```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```
Create a virtual environment with Python 3.10, using conda:

```bash
conda create -y -n lerobot python=3.10
```

Then activate your conda environment, you have to do this each time you open a shell to use lerobot:

```bash
conda activate lerobot
```

When using conda, install `ffmpeg` in your environment:

```bash
conda install ffmpeg -c conda-forge
```

---

### 3. Install the XLe Robot

First, clone the repository and navigate into the directory:

```bash
git clone https://github.com/Minko82/xle-robot.git
cd xle-robot
```

Then, install the library in editable mode. This is useful if you plan to contribute to the code.

```bash
pip install -e .
```

Install these extra features: 
```bash
pip install 'lerobot[all]'   
pip install -e ".[feetech]"
```



### Troubleshooting: 

If you encounter build errors, you may need to install additional dependencies: cmake, build-essential, and ffmpeg libs. To install these for linux run:

```bash
sudo apt-get install cmake build-essential python-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev pkg-config
```

---

### 4. Activating the Environment

Every time you start the project, activate your environment:

```bash
conda activate lerobot
```

---

### 5. Powering and Connecting the Hardware

***⚠️ Important:*** If motors become unresponsive after a failure, unplug and reconnect their motor power cables to reset them.

---

### 6. Finding Your Robot Ports

Use the built-in command to locate your connected robot devices:

```bash
lerobot-find-port
```

Example output:

```
right /dev/tty.usbmodem5A680127941
left  /dev/tty.usbmodem5A680135181
```

---

### 7. Calibrating the Robot (Run Once)

You need to calibrate your robot only **once per device**.

**Follower arm calibration:**

```bash
lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem5A680127941
```

**Leader arm calibration:**

```bash
lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem5A680135181 
```

---

## 💡 Credits

This project builds on top of [🤗 LeRobot](https://github.com/huggingface/lerobot) by Hugging Face Robotics and [XLeRobot](https://github.com/Vector-Wangel/XLeRobot).
All core functionality, datasets, and models remain attributed to the original authors.

---

## 📜 License

Apache License 2.0 — see the [LICENSE](./LICENSE) file for details.

---

<p align="center">
  <sub>© 2025 XLeRobot | Built on 🤗 LeRobot</sub>
</p>
