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
| T1.0 | SolidWorks export [HUMAN] | Team | 1 (tonight) | done | None | ROS 2 exporter export committed to tools/sw_export_raw/ps11_vehicle_export |
| T1.1 | Convert export to ROS 2 | Dev | 2 | done | T0.3, T1.0 | convert_sw_export.py, ps11_description (meshes decimated to 50k, URDF REP-103 validated) |
| T1.2 | Worlds, spawn, sensors and bridge | Dev | 2 | done | T1.1 | Gazebo Harmonic kinematic world, camera & depth_camera (640x480, 10 Hz / >=5 Hz in rclpy SensorDataQoS, 10 Hz default QoS), IMU 100 Hz, odom 50 Hz, bridge, dGPU verified |
| T1.3 | Navigation sensor models | Dev | 2 | done | T1.2 | odom_noise (seed: 0, 0.5% drift, 0.5° bias, TF map->base_link), depth_sim, range_adapter; unit tests pass, sim spawn alt/depth verified |
| T1.4 | Seabed decal generator | Dev | 3 | done | T1.2, T2.1 | **demo scenario**: 12 objects (5 debris, 7 marine life), procedural noise sand, H4 whitelist verified, camera views captured to results/bench/ |
| T1.5 | Waypoint follower | Dev | 2 | done | T1.3 | Lawnmower (LOS straight legs, axis-aligned step-overs), demo completed in 6.6 min, max cross-track 0.880 m <= 1.5 m, TRANSIT->SURVEY->RETURN->IDLE, lawnmower.png |
| T2.1 | Datasets | Dev (script) + Team (downloads) | 1 | done | T0.2 | TrashCan & DUO mapped, video-level split, test_manifest.txt generated |
| T2.2 | Training | Dev | 1 (overnight) | done | T2.1 | 129 epochs in 2.15 h, best.pt at epoch 109, early stopping patience=20 |
| T2.3 | Evaluation and export | Dev | 2 | done | T2.2 | Test split mAP50=0.703, ONNX exported and verified with onnx.checker |
| T2.4 | Underwater effect node | Dev | 2 | optional | T1.2 | Do if time, or if sim detections are poor |
| T2.5 | Detector node | Dev | 2 | done | T2.3 | Rate-capped YOLO (5.00 Hz measured), annotated image, H3 banner, detectability check across 12 objects logged to results/bench/ |
| T2.6 | Tracker node | Dev | 3 | done | T2.5 | ByteTrack, confirmed tracks (min_hits=5, mean_conf>=0.4, majority-vote class), unit tests passed |
| T2.7 | Geolocator node | Dev | 3 | done | T2.6, T1.3 | Ray-to-seabed flat projection, sigma formula, unit tests + static TF node test passed |
| T2.8 | Contact database node | Dev | 3 | done | T2.7 | Observation fusion |
| T2.9 | Contingency: fine-tune on sim frames | Dev | 2–3 | done | T1.4, T2.5 | 10 seeds (101-110, 1929 frames, 3795 labels, median offset 2.06 px); test mAP50 preserved: 0.705 vs 0.703 (+0.002, debris +0.035); best_v2.pt deployed |
| T3.1 | Codec | Dev | 1 | done | T0.3 | 8-byte messages, golden vectors approved |
| T3.2 | Link emulator | Dev | 1 | done | T3.1 | 64 bps, latency, loss, pull/queue modes, tests passed |
| T3.3 | Scheduler | Dev | 3 | done | T3.2, T2.8 | `semantic` required; policy, scheduler node, fake_vehicle, 5/5 unit tests passed, 90s e2e verified |
| T3.4 | Surface decoder | Dev | 3 | done | T3.1 | /link/rx only (H1), /surface/contacts, /surface/vehicle_track, /surface/markers published |
| T4.1 | Bringup | Dev | 4 | done | T1.5, T2.8, T3.3, T3.4 | vehicle, surface, demo launch files; all 15 nodes verified in headless 60s run |
| T4.2 | Metrics node | Dev | 4 | done | T3.4 | Counters, summary.json; recall over all objects ("GT in view" cut) |
| T4.3 | Foxglove layout | Dev | 4 | done | T4.1 | Operator layout, titles, banner |
| T4.4 | Integration and tuning | Dev | 4 | done | All above | Full-chain mission verified end-to-end; tuned on demo scenario across settings A/B/C; Setting C selected (0 false contacts, 0.245 m pos error, 1.54 s delay); MCAP recording reduced from 11.9 GB to 146.98 MB |
| T4.5 | Experiments and charts | Dev | 4 | done | T4.6 | H.264 comparison (4483.95x semantic advantage, 103.4h airtime vs 83s) and comparison charts generated in results/charts/ |
| T4.6 | Recording the demo | Dev | 4 | todo | T4.3 | MCAP bag + OBS video by mid-afternoon; copy to shared drive and USB |
| T4.7 | Deck update and rehearsal | Team (deck) + Dev (numbers) | 4 | todo | T4.5, T4.6, T5.2 | Placeholders, slide 10/11 phase status, measured numbers, rehearsal |
| T5.1 | Jetson reflash [HUMAN] | Team | 2 | todo | None | JetPack 6.x, trtexec, tegrastats, device.md |
| T5.2 | Jetson benchmark | Dev (script) + Team (run) | 3 | in progress | T2.3, T5.1 | Dev kit ready (benchmark.sh, parse_results.py, README.md; untested on device); pending team hardware run |
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