"""YAML parameter loader helper for PS11 AUV."""

import os
from pathlib import Path
from typing import Any

import yaml


def get_config_path(filename: str) -> Path:
    """Find the path to a config file in ps11_bringup/config.

    Searches:
    1. Ament package share directory (if installed and in ROS environment)
    2. PS11_ROOT or git workspace src directory (for dev/test environments)
    """
    # 1. Try ament index if available
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )

        try:
            share_dir = get_package_share_directory("ps11_bringup")
            config_path = Path(share_dir) / "config" / filename
            if config_path.is_file():
                return config_path
        except PackageNotFoundError:
            pass
    except ImportError:
        pass

    # 2. Try PS11_ROOT environment variable
    ps11_root = os.environ.get("PS11_ROOT")
    if ps11_root:
        candidate = (
            Path(ps11_root) / "ros2_ws" / "src" / "ps11_bringup" / "config" / filename
        )
        if candidate.is_file():
            return candidate

    # 3. Try relative to this file's location in workspace
    current_dir = Path(__file__).resolve()
    # Walk up to find workspace root
    for parent in current_dir.parents:
        candidate = parent / "ps11_bringup" / "config" / filename
        if candidate.is_file():
            return candidate
        candidate_src = parent / "src" / "ps11_bringup" / "config" / filename
        if candidate_src.is_file():
            return candidate_src

    raise FileNotFoundError(
        f"Config file '{filename}' not found in package share or source tree."
    )


def load_yaml(filename_or_path: str | Path) -> dict[str, Any]:
    """Load a YAML configuration file as a dict.

    If given a simple filename (e.g. 'classes.yaml'), resolves it via get_config_path.
    If given an existing path, loads directly.
    """
    path = Path(filename_or_path)
    if not path.is_file():
        path = get_config_path(str(filename_or_path))

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise TypeError(
            f"YAML file '{path}' must contain a mapping/dictionary at top level."
        )

    return data
