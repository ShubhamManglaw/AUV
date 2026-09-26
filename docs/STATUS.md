# PS11-AUV Task Status

| Task ID | Task Description | Owner | Status | Dependencies | Notes |
|---|---|---|---|---|---|
| **M1: Pitch Demo** | | | | | |
| T0.1 | Repository scaffold | Person A | done | None | Repository layout, empty packages, git lfs verified |
| T0.2 | Laptop environment | Person A (+ HUMAN) | todo | T0.1 | Drivers, ROS Jazzy, PyTorch, check_env.sh |
| T0.3 | Interfaces and common helpers | Person A | todo | T0.1 | ps11_interfaces, ps11_common, config YAMLs |
| T1.0 | SolidWorks export [HUMAN] | Person A [HUMAN] | todo | None | Pre-export checklist, raw export |
| T1.1 | Convert export to ROS 2 | Person A | todo | T0.3, T1.0 | convert_sw_export.py, ps11_description |
| T1.2 | Worlds, spawn, sensors and bridge | Person A | todo | T1.1 | Gazebo Harmonic worlds, sim.launch.py, bridge |
| T1.3 | Navigation sensor models | Person A | todo | T1.2 | odom_noise, depth_sim, range_adapter |
| T1.4 | Seabed decal generator | Person A | todo | T1.2, T2.1 | make_seabed.py, seabed.yaml, test split decals |
| T1.5 | Waypoint follower | Person A | todo | T1.3 | Lawnmower trajectory, kinematic control |
| T2.1 | Datasets | Person B | todo | T0.2 | TrashCan & DUO dataset prep, test manifest |
| T2.2 | Training | Person B | todo | T2.1 | YOLO nano training on CUDA |
| T2.3 | Evaluation and export | Person B | todo | T2.2 | Test mAP evaluation, ONNX export |
| T2.4 | Underwater effect node | Person B | todo | T1.2 | Synthetic attenuation and backscatter |
| T2.5 | Detector node | Person B | todo | T2.3, T2.4 | Rate-capped YOLO inference, annotated image |
| T2.6 | Tracker node | Person B | todo | T2.5 | ByteTrack track confirmation & majority voting |
| T2.7 | Geolocator node | Person B | todo | T2.6, T1.3 | Camera ray-to-seabed projection, uncertainty |
| T2.8 | Contact database node | Person B | todo | T2.7 | Multi-observation fusion, clustering |
| T2.9 | Contingency: fine-tune on sim frames | Person B | todo | T1.4, T2.5 | Only if onboard recall < 60% on Day 3 |
| T3.1 | Codec | Person B | todo | T0.3 | 8-byte bit-packing, golden vectors |
| T3.2 | Link emulator | Person A | todo | T3.1 | 64 bps channel emulation, latency, loss |
| T3.3 | Scheduler | Person A | todo | T3.2, T2.8 | Semantic prioritization vs FIFO baseline |
| T3.4 | Surface decoder | Person A | todo | T3.1 | /link/rx decoding, marker publication (H1) |
| T4.1 | Bringup | Person A | todo | T1.5, T2.8, T3.3, T3.4 | Top-level launch files |
| T4.2 | Metrics node | Person B | todo | T3.4 | Live counters, bandwidth ratios, summary.json |
| T4.3 | Foxglove layout | Person A | todo | T4.1 | Operator UI, pitch layout |
| T4.4 | Integration and tuning | Person A + B | todo | All above | End-to-end rehearsal and parameter tuning |
| T4.5 | Experiments and charts | Person B | todo | T4.4 | H.264 comparison, baseline runs, slide charts |
| T4.6 | Recording the demo | Person A | todo | T4.3 | MCAP recording & OBS screen recording |
| T4.7 | Deck update and rehearsal | Person A + B | todo | T4.5, T4.6, T5.3 | Presentation slides and timed runs |
| T5.1 | Jetson reflash [HUMAN] | Person B [HUMAN] | todo | None | JetPack 6.x SD card setup, trtexec |
| T5.2 | Jetson benchmark | Person B | todo | T2.3, T5.1 | TensorRT FP16 engine benchmark & tegrastats |
| T5.3 | Jetson video | Person B | todo | T5.2 | Benchmark recording video |
| **M2: Dynamic Vehicle & Jetson-in-the-Loop** | | | | | |
| T6.1 | Dynamic vehicle model | Person A | todo | M1 | Thrusters, Buoyancy, Hydrodynamics |
| T6.2 | Thruster allocation matrix | Person A | todo | T6.1 | compute_allocation.py |
| T6.3 | ArduSub SITL & custom frame | Person A | todo | T6.2 | ArduSub + ardupilot_gazebo plugin |
| T6.4 | mav_bridge | Person A | todo | T6.3 | pymavlink bridge, NED<->ENU conversion |
| T6.5 | SITL position source | Person A | todo | T6.4 | External position estimate for DVL-like nav |
| T6.6 | Dynamic mode launch & eval | Person A | todo | T6.1-T6.5 | mode:=dynamic evaluation |
| T6.7 | Jetson-in-the-loop ROS 2 | Person B | todo | M1, T5.1 | ROS 2 on Jetson, networked perception |
| T6.8 | Jetson INT8 quantization | Person B | todo | T6.7 | TensorRT INT8 calibration |
| T6.9 | Detector sim-vs-real analysis | Person B | todo | T2.9 | Per-class performance analysis |
| **M3: Mission Intelligence & Link Features** | | | | | |
| T7.1 | Behaviour tree | Person A | todo | M2 | py_trees_ros re-inspection behavior |
| T7.2 | Half-duplex uplink | Person A | todo | M2 | EXTENDED uplink commands & turn-taking |
| T7.3 | Tier-2 thumbnails | Person B | todo | T7.2 | Grayscale chip compression & reassembly |
| T7.4 | Burst-loss channel & ACKs | Person A | todo | M2 | Gilbert-Elliott channel & retransmission |
| T7.5 | Hydrodynamic parameter tuning | Person A | todo | M2 | Hydrodynamic coefficient estimation |
| T7.6 | Detector licence migration study | Person B | todo | M2 | Permissive license model comparison |
| T7.7 | Multi-scenario evaluation report | Person B | todo | M2, M3 | Extended evaluation report across scenarios |
