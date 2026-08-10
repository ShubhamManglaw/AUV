# PS11-AUV Task Status

**Owners:** Dev = Shubham (all code, with the ZCode agent) · Team = the two teammates (CAD, datasets, Jetson, deck).
**Status values:** todo · in progress · done · blocked · optional (M1, only if time) · deferred (moved to after the pitch).
Scope changes for the one-developer schedule are described in plan §15.1 ("Reduced M1 scope").

| Task ID | Task | Owner | Day | Status | Dependencies | Notes |
|---|---|---|---|---|---|---|
| **M1: Pitch Demo** | | | | | | |
| T0.1 | Repository scaffold | Dev | 1 | done | None | Layout, empty packages, Git LFS verified; follow-up fixes: ps11_gazebo as ament_cmake, .gitkeep, CHANGELOG |
| T0.2 | Laptop environment | Dev (+ HUMAN for driver) | 1 | done | T0.1 | Driver, Xorg and apt packages by hand; venv, env.sh, check_env.sh by agent; all checks PASS |
| T0.3 | Interfaces and common helpers | Dev | 1 | done | T0.1 | ps11_interfaces, ps11_common, config YAMLs; tests and rosidl verified |
| T1.0 | SolidWorks export [HUMAN] | Team | 1 (tonight) | todo | None | §7.1 checklist; send zipped export + assembly + mass, dimensions, thruster count and max thrust |
| T1.1 | Convert export to ROS 2 | Dev | 2 | todo | T0.3, T1.0 | convert_sw_export.py, ps11_description |
| T1.2 | Worlds, spawn, sensors and bridge | Dev | 2 | todo | T1.1 | Gazebo Harmonic worlds, sim.launch.py, bridge, env hook |
| T1.3 | Navigation sensor models | Dev | 2 | todo | T1.2 | odom_noise, depth_sim, range_adapter |
| T1.4 | Seabed decal generator | Dev | 3 | todo | T1.2, T2.1 | **demo scenario only**; eval scenario deferred |
| T1.5 | Waypoint follower | Dev | 2 | todo | T1.3 | Lawnmower, kinematic control |
| T2.1 | Datasets | Dev (script) + Team (downloads) | 1 | done | T0.2 | TrashCan & DUO mapped, video-level split, test_manifest.txt generated |
| T2.2 | Training | Dev | 1 (overnight) | done | T2.1 | 129 epochs in 2.15 h, best.pt at epoch 109, early stopping patience=20 |
| T2.3 | Evaluation and export | Dev | 2 | done | T2.2 | Test split mAP50=0.703, ONNX exported and verified with onnx.checker |
| T2.4 | Underwater effect node | Dev | 2 | optional | T1.2 | Do if time, or if sim detections are poor |
| T2.5 | Detector node | Dev | 2 | todo | T2.3 (T2.4 if done) | Rate-capped YOLO, annotated image, H3 banner |
| T2.6 | Tracker node | Dev | 3 | todo | T2.5 | ByteTrack, confirmed tracks only |
| T2.7 | Geolocator node | Dev | 3 | todo | T2.6, T1.3 | Ray-to-seabed projection, uncertainty |
| T2.8 | Contact database node | Dev | 3 | todo | T2.7 | Observation fusion |
| T2.9 | Contingency: fine-tune on sim frames | Dev | 2–3 | optional | T1.4, T2.5 | Only if sim detections are poor (decide end of Day 2) |
| T3.1 | Codec | Dev | 1 | done | T0.3 | 8-byte messages, golden vectors approved |
| T3.2 | Link emulator | Dev | 1 | done | T3.1 | 64 bps, latency, loss, pull/queue modes, tests passed |
| T3.3 | Scheduler | Dev | 3 | todo | T3.2, T2.8 | `semantic` required; `fifo_observations` optional |
| T3.4 | Surface decoder | Dev | 3 | todo | T3.1 | /link/rx only (H1) |
| T4.1 | Bringup | Dev | 4 | todo | T1.5, T2.8, T3.3, T3.4 | Top-level launch files |
| T4.2 | Metrics node | Dev | 4 | todo | T3.4 | Counters, summary.json; recall over all objects ("GT in view" cut) |
| T4.3 | Foxglove layout | Dev | 4 | todo | T4.1 | Operator layout, titles, banner |
| T4.4 | Integration and tuning | Dev | 4 | todo | All above | Tune and report on demo scenario, labelled as the tuned scenario |
| T4.5 | Experiments and charts | Dev | 4 | todo | T4.6 | H.264 comparison and charts from the recorded run; baseline run only if fifo policy exists; generic_1k deferred |
| T4.6 | Recording the demo | Dev | 4 | todo | T4.3 | MCAP bag + OBS video by mid-afternoon; copy to shared drive and USB |
| T4.7 | Deck update and rehearsal | Team (deck) + Dev (numbers) | 4 | todo | T4.5, T4.6, T5.2 | Placeholders, slide 10/11 phase status, measured numbers, rehearsal |
| T5.1 | Jetson reflash [HUMAN] | Team | 2 | todo | None | JetPack 6.x, trtexec, tegrastats, device.md |
| T5.2 | Jetson benchmark | Dev (script) + Team (run) | 3 | todo | T2.3, T5.1 | Dev writes jetson/benchmark.sh; Team runs it and commits logs |
| T5.3 | Jetson video | Team | 3 | todo | T5.2 | Not committed; keep in shared drive |
| **M2: Dynamic Vehicle & Jetson-in-the-Loop** | | | | | | |
| T6.1 | Dynamic vehicle model | Dev | — | todo | M1 | Thrusters, Buoyancy, Hydrodynamics |
| T6.2 | Thruster allocation matrix | Dev | — | todo | T6.1 | compute_allocation.py |
| T6.3 | ArduSub SITL & custom frame | Dev | — | todo | T6.2 | ArduSub + ardupilot_gazebo plugin |
| T6.4 | mav_bridge | Dev | — | todo | T6.3 | pymavlink, NED↔ENU conversion |
| T6.5 | SITL position source | Dev | — | todo | T6.4 | External position estimate (DVL-like) |
| T6.6 | Dynamic mode launch & eval | Dev | — | todo | T6.1–T6.5 | mode:=dynamic regression |
| T6.7 | Jetson-in-the-loop ROS 2 | Dev + Team | — | todo | M1, T5.1 | ROS 2 on Jetson, networked perception |
| T6.8 | Jetson INT8 quantization | Dev | — | todo | T6.7 | TensorRT INT8 calibration |
| T6.9 | Detector sim-vs-real analysis | Dev | — | todo | T2.9 | Per-class analysis |
| M1 deferred | `eval` scenario, `generic_1k` run, "GT in view" metric, FIFO baseline (if not done) | Dev | — | deferred | M1 | Pick up in the first M2 week |
| **M3: Mission Intelligence & Link Features** | | | | | | |
| T7.1 | Behaviour tree | Dev | — | todo | M2 | py_trees_ros re-inspection |
| T7.2 | Half-duplex uplink | Dev | — | todo | M2 | EXTENDED uplink commands, turn-taking |
| T7.3 | Tier-2 thumbnails | Dev | — | todo | T7.2 | Grayscale chips, reassembly |
| T7.4 | Burst-loss channel & ACKs | Dev | — | todo | M2 | Gilbert–Elliott, retransmission |
| T7.5 | Hydrodynamic parameter tuning | Dev + Team | — | todo | M2 | Coefficients from CAD geometry and literature |
| T7.6 | Detector licence migration study | Dev | — | todo | M2 | Apache-2.0/MIT alternative |
| T7.7 | Multi-scenario evaluation report | Dev | — | todo | M2, M3 | Seeds, densities, link profiles |