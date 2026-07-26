# Changelog

All notable changes and interface updates to the PS11-AUV project will be documented in this file.

## T0.1: repository scaffold

- Created repository directory scaffold per implementation plan §6 (`tools/`, `ml/`, `jetson/results/`, `results/`, `ros2_ws/src/`).
- Initialized empty ROS 2 packages in `ros2_ws/src/`:
  - `ament_cmake`: `ps11_interfaces`, `ps11_description`, `ps11_gazebo`
  - `ament_python`: `ps11_common`, `ps11_nav`, `ps11_perception`, `ps11_telemetry`, `ps11_eval`, `ps11_bringup`, `ps11_mission`
- Configured `ps11_gazebo` as an `ament_cmake` package with directory structure (`worlds/`, `models/`, `launch/`, `config/`) and an environment hook exporting `GZ_SIM_RESOURCE_PATH`.
- Added `[build_scripts] executable = /usr/bin/env python3` to `setup.cfg` across all Python packages.
- Configured `.gitattributes` for Git LFS and verified tracking.
- Created `docs/STATUS.md` tracking table.

## T3.1: telemetry codec interface and validation updates

- §11.1 contract update: Identifiers must not be clamped. Out-of-range `contact_id` (0–255), `class_id` (0–7), or `state` (0–7) now raise `ValueError`.
- Clamping with warning applies strictly to physical values (`x`, `y`, `depth`, `confidence`, `battery`). `pending` saturates at 63 without warning.
- Explicit round-half-away-from-zero quantization adopted for all coordinate and fraction conversions, avoiding Python banker's rounding edge cases.
- Relocated golden vectors to `ros2_ws/src/ps11_telemetry/test/golden_vectors.json` and updated test case 5 to valid identifier IDs (`contact_id=255`, `class_id=0`).
