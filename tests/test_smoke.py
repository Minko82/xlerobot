"""Hardware-free smoke tests.

These tests verify that the repository is self-consistent: every model asset
referenced by the code exists, and all scripts are syntactically valid. They
require no robot, camera, or GPU.
"""

import compileall
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CUBE_VISION = REPO_ROOT / "cube-vision"

# Model assets that the vision/IK pipeline loads at runtime.
REQUIRED_ASSETS = [
    CUBE_VISION / "assets" / "xlerobot.xml",
    CUBE_VISION / "frame_transform" / "xlerobot" / "xlerobot.xml",
    CUBE_VISION / "frame_transform" / "xlerobot" / "xlerobot.urdf",
    CUBE_VISION / "frame_transform" / "xlerobot" / "xlerobot_front.urdf",
    CUBE_VISION / "ik_solver" / "SO-ARM100" / "Simulation" / "SO101" / "so101_new_calib.urdf",
]

# Paths the project website's build guide tells users to run.
WEBSITE_REFERENCED = [
    REPO_ROOT / "set_motor_id.py",
    REPO_ROOT / "examples" / "0_so100_keyboard_joint_control.py",
    REPO_ROOT / "examples" / "provided_examples" / "1_so100_keyboard_ee_control.py",
]


def test_model_assets_exist():
    missing = [str(p) for p in REQUIRED_ASSETS if not p.exists()]
    assert not missing, f"Missing model assets: {missing}"


def test_website_referenced_scripts_exist():
    missing = [str(p) for p in WEBSITE_REFERENCED if not p.exists()]
    assert not missing, f"Missing scripts referenced by the build guide: {missing}"


def test_all_python_compiles():
    for directory in ("src", "examples", "cube-vision"):
        assert compileall.compile_dir(
            str(REPO_ROOT / directory), quiet=2, force=True
        ), f"Syntax errors under {directory}/"


def test_calibration_files_are_valid_json():
    import json

    for path in (REPO_ROOT / "calibration").glob("*.json"):
        json.loads(path.read_text())
    for path in (CUBE_VISION / "calibration").glob("*.json"):
        json.loads(path.read_text())
