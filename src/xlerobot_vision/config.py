"""Shared configuration for the XLeRobot-Pro vision/IK stack.

All paths and hardware defaults used across the package live here so that
scripts, diagnostics, and examples stay consistent.
"""

import os
from pathlib import Path

# ---------------------------------------------------------------------------
# Hardware defaults (override per-script with CLI flags where provided)
# ---------------------------------------------------------------------------

#: Default serial port for the single-bus arm + head setup.
BUS_PORT = os.getenv("XLEROBOT_BUS_PORT", "/dev/ttyACM0")

# ---------------------------------------------------------------------------
# Model assets (shipped as package data)
# ---------------------------------------------------------------------------

ASSETS_DIR = Path(__file__).resolve().parent / "assets"

#: Canonical MJCF model of the robot (used by FK, IK, and MuJoCo tooling).
MJCF_PATH = ASSETS_DIR / "xlerobot" / "xlerobot.xml"

#: URDF variants of the same robot.
URDF_PATH = ASSETS_DIR / "xlerobot" / "xlerobot.urdf"
FRONT_URDF_PATH = ASSETS_DIR / "xlerobot" / "xlerobot_front.urdf"

# ---------------------------------------------------------------------------
# Calibration data
# ---------------------------------------------------------------------------

# This repository is used via an editable install, so the versioned
# calibration/ directory at the repo root is the default. Override with
# XLEROBOT_CALIBRATION_DIR for non-repo deployments.
_REPO_CALIBRATION = Path(__file__).resolve().parents[2] / "calibration"
CALIBRATION_DIR = Path(os.getenv("XLEROBOT_CALIBRATION_DIR", _REPO_CALIBRATION))

DEFAULT_CALIBRATION_FILE = CALIBRATION_DIR / "single_bus.json"
HEAD_CALIBRATION_FILE = CALIBRATION_DIR / "head.json"
