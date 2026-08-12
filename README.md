# ps11-auv

NewtonBotics — BDTS 2027 NextGen Challenge, PS11: Edge Computing for Sensors / Reducing Bandwidth for Backhaul.

An AUV edge-intelligence pipeline that turns camera frames into compact, prioritised contact reports and sends them over an emulated 64 bps acoustic link.

**Status:** simulation stage (TRL 3–4). Everything in this repository runs in simulation unless stated otherwise.

- Implementation plan: [`docs/implementation_plan.md`](docs/implementation_plan.md)
- Rules for the coding agent: [`AGENTS.md`](AGENTS.md)
- Environment setup: plan §5
- Running the demo: plan §17

## Build
Always source `tools/env.sh` and build with the virtual environment's Python from `ros2_ws/`:
```bash
source tools/env.sh
cd ros2_ws
python -m colcon build --symlink-install
```
Always build with `python -m colcon build` (never bare `colcon build`) so node executables use the venv interpreter.

