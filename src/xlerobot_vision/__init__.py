"""XLeRobot-Pro vision and manipulation library.

Perception (RealSense capture, color detection), camera-to-base frame
transforms, and differential IK for the SO-101 arms.

Submodules are imported lazily so that using one part of the pipeline does
not require the dependencies of another (e.g. IK works without pyrealsense2).
"""

from xlerobot_vision import config  # noqa: F401  (lightweight, no heavy deps)

# Public name -> (submodule, attribute)
_LAZY_EXPORTS = {
    "IK_SO101": ("xlerobot_vision.ik", "IK_SO101"),
    "camera_xyz_to_base_xyz": ("xlerobot_vision.frame_transform", "camera_xyz_to_base_xyz"),
    "detect_object": ("xlerobot_vision.color_detect", "detect_object"),
    "detect_color": ("xlerobot_vision.color_detect", "detect_color"),
    "detection_to_xyz": ("xlerobot_vision.color_detect", "detection_to_xyz"),
    "COLOR_RANGES": ("xlerobot_vision.color_detect", "COLOR_RANGES"),
    "capture": ("xlerobot_vision.realsense", "capture"),
    "PointCloud": ("xlerobot_vision.point_cloud", "PointCloud"),
    "MOTOR_DEFS": ("xlerobot_vision.calibration", "MOTOR_DEFS"),
    "load_or_run_calibration": ("xlerobot_vision.calibration", "load_or_run_calibration"),
    "save_ik_plot": ("xlerobot_vision.viz", "save_ik_plot"),
    "BUS_PORT": ("xlerobot_vision.config", "BUS_PORT"),
}

__all__ = ["config", *_LAZY_EXPORTS]


def __getattr__(name: str):
    try:
        module_name, attr = _LAZY_EXPORTS[name]
    except KeyError:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from None
    import importlib

    return getattr(importlib.import_module(module_name), attr)


def __dir__():
    return sorted(__all__)
