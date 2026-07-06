"""Hardware-free smoke tests.

These tests verify that the repository is self-consistent: every model asset
referenced by the code exists, and all scripts are syntactically valid. They
require no robot, camera, or GPU.
"""

import compileall
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
VISION_ASSETS = REPO_ROOT / "src" / "xlerobot_vision" / "assets"

# Model assets that the vision/IK library loads at runtime.
REQUIRED_ASSETS = [
    VISION_ASSETS / "xlerobot" / "xlerobot.xml",
    VISION_ASSETS / "xlerobot" / "xlerobot.urdf",
    VISION_ASSETS / "xlerobot" / "xlerobot_front.urdf",
]

# Paths the project website's build guide tells users to run.
WEBSITE_REFERENCED = [
    REPO_ROOT / "set_motor_id.py",
    REPO_ROOT / "examples" / "0_so100_keyboard_joint_control.py",
    REPO_ROOT / "examples" / "provided_examples" / "1_so100_keyboard_ee_control.py",
]

CODE_DIRS = ("src", "examples", "diagnostics", "scripts", "tests")


def test_model_assets_exist():
    missing = [str(p) for p in REQUIRED_ASSETS if not p.exists()]
    assert not missing, f"Missing model assets: {missing}"


def test_config_paths_resolve():
    from xlerobot_vision import config

    assert config.MJCF_PATH.exists()
    assert config.FRONT_URDF_PATH.exists()
    assert config.CALIBRATION_DIR.is_dir()
    assert config.DEFAULT_CALIBRATION_FILE.exists()
    assert config.HEAD_CALIBRATION_FILE.exists()


def test_website_referenced_scripts_exist():
    missing = [str(p) for p in WEBSITE_REFERENCED if not p.exists()]
    assert not missing, f"Missing scripts referenced by the build guide: {missing}"


def test_all_python_compiles():
    for directory in CODE_DIRS:
        assert compileall.compile_dir(str(REPO_ROOT / directory), quiet=2, force=True), (
            f"Syntax errors under {directory}/"
        )


def test_calibration_files_are_valid_json():
    calibration_files = list((REPO_ROOT / "calibration").glob("*.json"))
    assert calibration_files, "No calibration files found"
    for path in calibration_files:
        json.loads(path.read_text())
