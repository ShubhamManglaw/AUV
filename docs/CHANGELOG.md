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

## T4.3: Foxglove layout improvements and DemoCounters interface update

- Added `uint32 correct_contacts` and `uint32 false_contacts` to `DemoCounters.msg` and updated `implementation_plan.md` §14.2.
- Updated `metrics_node.py` to publish `correct_contacts` (ground-truth matched contacts) and `false_contacts` (unmatched surface contacts) on `/eval/counters`.
- Upgraded `ps11_pitch_layout.json`:
  - Replaced unsupported `ImageView` panel with Foxglove native `Image` panel on `/vehicle/perception/image_annotated/compressed`.
  - Replaced `Indicator` panels with dedicated `RawMessages` panels for semantic bits sent, JPEG-equivalent bits, ratio vs JPEG, video airtime on 64 bps link, and correct vs false surface contacts.
  - Split bandwidth comparison plot into separate `Semantic bits sent (cumulative)` and `Video-equivalent bits (cumulative)` panels with receive-time timestamps to prevent vertical axis compression.
  - Set panel titles adhering strictly to plan §13.
  - Adjusted left 3D onboard camera to 10 m following `base_link`, and right 3D surface view to fixed `map` top-down orthographic view of 50x30 m survey area with 10 m grid.
  - Scaled up surface marker visuals in `surface_decoder_node.py` for crystal-clear readability from high altitude.

