# AGENTS.md — rules for coding agents on ps11-auv

(Read automatically by ZCode from the project root. Do not regenerate this file with `/init`.)

## Working style
- Work on exactly one task ID per session or Goal. Do not start the next task on your own.
- Read only the plan sections the task card lists, plus §1.4 and §14. Do not reload the whole plan unless asked.
- Before editing, list the files you will create or change and wait for approval if the list touches files outside the task's deliverables.

## Before you start
1. Read `docs/implementation_plan.md` §0, §1.4 (honesty rules), §14 (interfaces), and the card for your task in §15.
2. Check `docs/STATUS.md`. Do not start a task whose dependencies are not `done`.
3. Restate the deliverables and acceptance criteria before writing code.

## Environment
- Ubuntu 24.04 (Xorg session), ROS 2 Jazzy, Gazebo Harmonic, Python 3.12.
- Always `source tools/env.sh` first. Build with `colcon build --symlink-install` from `ros2_ws/`.
- Python packages need `[build_scripts] executable = /usr/bin/env python3` in `setup.cfg`.
- Do not use `cv_bridge` in nodes; use `ps11_common.image_utils`.
- Do not install system packages or change the numpy version without telling the human.

## Code rules
- Python (`rclpy`) with type hints. C++ only if a node is measured too slow, and only after the human agrees.
- Pure logic (codec, policy, geometry, fusion) goes in ROS-free modules with `pytest` tests. ROS nodes are thin wrappers.
- Always run tests with `python -m pytest` (the venv's Python), never bare `pytest`.
- No hard-coded constants: every number comes from a YAML file in `ps11_bringup/config/`. Class IDs and names come only from `classes.yaml`.
- All nodes use `use_sim_time: true`.
- Run `ruff format` and `ruff check` before finishing.

## Interfaces
- Topic names, message definitions, the 64-bit message layouts and parameter names in the plan are contracts.
- Do not change them unless the task says so. If you must, update §14 (and §11 for bit layouts) of the plan and `docs/CHANGELOG.md` in the same commit, and tell the human.
- Never edit `test/golden_vectors.json` after it has been approved.

## Honesty rules (blocking)
- `/surface/*` nodes subscribe only to `/link/rx` and `/clock`.
- Only `odom_noise`, `depth_sim` and `metrics` may read `/sim/*`.
- Seabed decals come only from images in `ml/data/test_manifest.txt`.
- Never invent, estimate or "fill in" benchmark or metric numbers. If a number isn't measured, write `NOT MEASURED`.
- Keep `ASSUMPTION` labels in config and on screen.

## Finishing a task
- Run every acceptance check in the task card. Paste the exact commands and their output in your final message.
- Paste command output by copying it from the terminal. Never retype numbers, hex values or results. If output is long, paste the relevant part verbatim and say what you cut.
- Update `docs/STATUS.md`.
- Commit with a message starting with the task ID, e.g. `T3.1: codec with golden vectors`.
- If the plan is ambiguous about an interface, a message field or an honesty rule, stop and ask instead of guessing.
