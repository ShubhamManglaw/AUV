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
