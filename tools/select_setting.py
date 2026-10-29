#!/usr/bin/env python3
"""Tool to switch between Setting A, B, C, and D configurations (§Q7, §14.4).

Usage:
  python3 tools/select_setting.py A   # (or 1)
  python3 tools/select_setting.py B   # (or 2)
  python3 tools/select_setting.py C   # (or 3)
  python3 tools/select_setting.py D   # (or 4)
  python3 tools/select_setting.py     # Interactive selection menu
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_YAML = (
    WORKSPACE_ROOT / "ros2_ws" / "src" / "ps11_bringup" / "config" / "perception.yaml"
)
MISSION_YAML = (
    WORKSPACE_ROOT / "ros2_ws" / "src" / "ps11_bringup" / "config" / "mission.yaml"
)

SETTINGS: dict[str, dict[str, Any]] = {
    "A": {
        "name": "Setting A (Baseline Conservative)",
        "description": "conf: 0.35, min_hits: 5, min_mean_conf: 0.40 | Alt: 2.5m, Spacing: 4.0m",
        "detector": {
            "conf_threshold": 0.35,
            "iou_threshold": 0.50,
            "nominal_rate_hz": 10.0,
            "boosted_rate_hz": 30.0,
            "boost_hold_s": 3.0,
            "max_rate_hz": 30.0,
            "camera_rate_hz": 30.0,
            "annotate_rate_hz": 10.0,
        },
        "tracker": {
            "min_hits": 5,
            "min_mean_conf": 0.40,
            "track_activation_threshold": 0.35,
            "lost_track_buffer": 30,
            "minimum_matching_threshold": 0.80,
        },
        "mission": {
            "altitude_m": 2.5,
            "leg_spacing_m": 4.0,
        },
    },
    "B": {
        "name": "Setting B (Intermediate Filtering)",
        "description": "conf: 0.25, min_hits: 3, min_mean_conf: 0.40 | Alt: 2.5m, Spacing: 4.0m",
        "detector": {
            "conf_threshold": 0.25,
            "iou_threshold": 0.50,
            "nominal_rate_hz": 10.0,
            "boosted_rate_hz": 30.0,
            "boost_hold_s": 3.0,
            "max_rate_hz": 30.0,
            "camera_rate_hz": 30.0,
            "annotate_rate_hz": 10.0,
        },
        "tracker": {
            "min_hits": 3,
            "min_mean_conf": 0.40,
            "track_activation_threshold": 0.25,
            "lost_track_buffer": 30,
            "minimum_matching_threshold": 0.80,
        },
        "mission": {
            "altitude_m": 2.5,
            "leg_spacing_m": 4.0,
        },
    },
    "C": {
        "name": "Setting C (Tuned Balanced Default)",
        "description": "conf: 0.25, min_hits: 3, min_mean_conf: 0.30 | Alt: 2.5m, Spacing: 4.0m",
        "detector": {
            "conf_threshold": 0.25,
            "iou_threshold": 0.50,
            "nominal_rate_hz": 10.0,
            "boosted_rate_hz": 30.0,
            "boost_hold_s": 3.0,
            "max_rate_hz": 30.0,
            "camera_rate_hz": 30.0,
            "annotate_rate_hz": 10.0,
        },
        "tracker": {
            "min_hits": 3,
            "min_mean_conf": 0.30,
            "track_activation_threshold": 0.25,
            "lost_track_buffer": 30,
            "minimum_matching_threshold": 0.80,
        },
        "mission": {
            "altitude_m": 2.5,
            "leg_spacing_m": 4.0,
        },
    },
    "D": {
        "name": "Setting D (Low-Altitude / High-Density Grid)",
        "description": "conf: 0.25, min_hits: 3, min_mean_conf: 0.30 | Alt: 1.8m, Spacing: 3.0m (7 survey passes)",
        "detector": {
            "conf_threshold": 0.25,
            "iou_threshold": 0.50,
            "nominal_rate_hz": 10.0,
            "boosted_rate_hz": 30.0,
            "boost_hold_s": 3.0,
            "max_rate_hz": 30.0,
            "camera_rate_hz": 30.0,
            "annotate_rate_hz": 10.0,
        },
        "tracker": {
            "min_hits": 3,
            "min_mean_conf": 0.30,
            "track_activation_threshold": 0.25,
            "lost_track_buffer": 30,
            "minimum_matching_threshold": 0.80,
        },
        "mission": {
            "altitude_m": 1.8,
            "leg_spacing_m": 3.0,
        },
    },
}

ALIAS_MAP = {
    "1": "A",
    "A": "A",
    "SECTION A": "A",
    "SETTING A": "A",
    "2": "B",
    "B": "B",
    "SECTION B": "B",
    "SETTING B": "B",
    "3": "C",
    "C": "C",
    "SECTION C": "C",
    "SETTING C": "C",
    "4": "D",
    "D": "D",
    "SECTION D": "D",
    "SETTING D": "D",
}


def apply_setting(choice_key: str, build: bool = True) -> None:
    key = choice_key.upper().strip()
    norm_key = ALIAS_MAP.get(key)
    if not norm_key or norm_key not in SETTINGS:
        print(
            f"Error: Unknown setting '{choice_key}'. Choose from A, B, C, D (or 1, 2, 3, 4)."
        )
        sys.exit(1)

    cfg = SETTINGS[norm_key]
    print("\n=======================================================")
    print(f" Applying: {cfg['name']}")
    print(f" Details:  {cfg['description']}")
    print("=======================================================\n")

    # 1. Update perception.yaml
    with open(PERCEPTION_YAML, "r", encoding="utf-8") as f:
        perception_data = yaml.safe_load(f)

    # Update detector params
    for k, v in cfg["detector"].items():
        perception_data["detector"][k] = v

    # Update tracker params
    for k, v in cfg["tracker"].items():
        clean_k = k.split(":")[0].strip()
        perception_data["tracker"][clean_k] = v

    with open(PERCEPTION_YAML, "w", encoding="utf-8") as f:
        yaml.safe_dump(perception_data, f, sort_keys=False)
    print(f" [OK] Updated {PERCEPTION_YAML.name}")

    # 2. Update mission.yaml
    with open(MISSION_YAML, "r", encoding="utf-8") as f:
        mission_data = yaml.safe_load(f)

    alt = cfg["mission"]["altitude_m"]
    spacing = cfg["mission"]["leg_spacing_m"]

    mission_data["lawnmower"]["altitude_m"] = alt
    mission_data["lawnmower"]["leg_spacing_m"] = spacing
    if "scenarios" in mission_data and "demo" in mission_data["scenarios"]:
        mission_data["scenarios"]["demo"]["altitude_m"] = alt
        mission_data["scenarios"]["demo"]["leg_spacing_m"] = spacing

    with open(MISSION_YAML, "w", encoding="utf-8") as f:
        yaml.safe_dump(mission_data, f, sort_keys=False)
    print(f" [OK] Updated {MISSION_YAML.name} (altitude: {alt}m, spacing: {spacing}m)")

    # 3. Colcon build symlink-install
    if build:
        print("\n Running colcon build for ps11_bringup and ps11_perception...")
        res = subprocess.run(
            [
                sys.executable,
                "-m",
                "colcon",
                "build",
                "--symlink-install",
                "--packages-select",
                "ps11_bringup",
                "ps11_perception",
            ],
            cwd=WORKSPACE_ROOT / "ros2_ws",
            capture_output=True,
            text=True,
            check=False,
        )
        if res.returncode != 0:
            print(" Colcon build failed:\n", res.stderr)
            sys.exit(1)
        print(" [OK] Build complete and symlinks updated.")

    print("\n-------------------------------------------------------")
    print(f" SUCCESS: {norm_key} is now ACTIVE!")
    print("-------------------------------------------------------")
    print("To run the mission now:")
    print("  bash tools/sim_cleanup.sh")
    print("  source tools/env.sh")
    print("  ros2 launch ps11_bringup demo.launch.py gui:=true\n")


def interactive_menu() -> str:
    print("\n=======================================================")
    print("        PS11 AUV Parameter Setting Selector            ")
    print("=======================================================")
    print(
        " 1) [A] Setting A: Baseline (conf: 0.35, min_hits: 5, alt: 2.5m, spacing: 4m)"
    )
    print(
        " 2) [B] Setting B: Moderate (conf: 0.25, min_hits: 3, alt: 2.5m, spacing: 4m)"
    )
    print(
        " 3) [C] Setting C: Tuned Default (conf: 0.25, min_hits: 3, alt: 2.5m, spacing: 4m)"
    )
    print(
        " 4) [D] Setting D: Low Altitude (conf: 0.25, min_hits: 3, alt: 1.8m, spacing: 3m)"
    )
    print("=======================================================")
    choice = input(
        "Enter your choice [A, B, C, D or 1, 2, 3, 4] (default: C): "
    ).strip()
    return choice if choice else "C"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Switch PS11 AUV Parameter Settings (A, B, C, D)"
    )
    parser.add_argument(
        "setting",
        nargs="?",
        default=None,
        help="Setting letter or number: A (1), B (2), C (3), D (4)",
    )
    parser.add_argument(
        "--no-build",
        action="store_true",
        help="Skip colcon build step",
    )
    args = parser.parse_args()

    choice = args.setting
    if choice is None:
        choice = interactive_menu()

    apply_setting(choice, build=not args.no_build)


if __name__ == "__main__":
    main()
