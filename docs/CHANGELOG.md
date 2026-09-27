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

## T3.1: golden vectors approved and A4 confirmation

- Marked T3.1 done following human review and approval of golden vectors.
- Confirmed assumption A4: Water Linked M64 protocol specification confirms 8-byte payload per acoustic packet. Removed `ASSUMPTION` label from `frame_payload_bytes` in plan §11.6 and `link_profiles.yaml` (retained on `loss_prob`).
- Noted in `link_profiles.yaml` and plan §11.6 label that M64 is discontinued and used as a representative published spec.
- Added strict AGENTS.md rules requiring `python -m pytest` and terminal copy-pasting of verbatim command output.

## T3.2: link emulator

- Implemented pure-Python `LinkModel` channel emulator (`ps11_telemetry/link_model.py`) supporting acoustic airtime calculation, half-duplex channel state, propagation latency, random frame loss, all-zero sync payload discarding at receiver, queue mode buffering, and pull mode `tx_ready` repeat signals.
- Implemented `link_emulator` ROS 2 node (`ps11_telemetry/link_emulator_node.py`) wrapping `LinkModel` with sim-time support, `/link/tx` subscription, and `/link/rx`, `/link/tx_ready`, `/link/stats` publishers.
- Added unit tests with simulated clock in `ps11_telemetry/test/test_link_model.py`.

## T3.4: surface decoder and mission start time parameter

- §11.1 plan update: Surface decoder cannot access vehicle odometry per honesty rule H1. Replaced odometry-based mission start with shared parameter `mission_start_s` in `scheduler.yaml` (default 0.0 = sim start), read by both `scheduler` and `surface_decoder`.

