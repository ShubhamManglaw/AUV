# PS11-AUV — Technology Stack Explained

**For:** the teammate building the presentation, and anyone who needs to *understand* the tech stack well enough to explain it to BDTS judges
**Not for:** installing or coding (that is `docs/TECH_STACK.md` and `docs/implementation_plan.md`)
**Status date:** 29 September 2026
**Companion:** `docs/SYSTEM_ARCHITECTURE.md` (how the whole system works, in depth)

---

## 0. How to use this document

For every technology we use, this document answers five questions:

1. **What is it?** — in plain words
2. **What does it do in our system?**
3. **Why did we choose it?** — and what we compared it with
4. **Is it ours or off-the-shelf?** — important for indigenisation questions
5. **Status** — built and tested, or planned

Each section ends with a **slide line**: one sentence you can put on a slide or say out loud.

**Status labels**

| Label | Meaning |
|---|---|
| ✅ In use | Built, running and tested in our system |
| 🎯 Target | Chosen for the physical vehicle; used in simulation or benchmarking now |
| 🗓 Planned | Later milestone (M2/M3) |
| ⏳ Pending | A measurement not yet made — never quote a number for it |

---

## 1. The stack at a glance

```
┌──────────────────────────────────────────────────────────────────────┐
│ 6. DEVELOPMENT & QUALITY  GitHub · Git LFS · pytest · ruff · golden   │
│                           test vectors · honesty rules · AI agents    │
├──────────────────────────────────────────────────────────────────────┤
│ 5. DATA & TRAINING        TrashCan 1.0 · DUO · auto-labelled sim      │
│                           frames · Ultralytics training · PyTorch     │
├──────────────────────────────────────────────────────────────────────┤
│ 4. SOFTWARE PLATFORM      Ubuntu 24.04 · ROS 2 Jazzy · Gazebo         │
│                           Harmonic · Python 3.12 · Foxglove · MCAP    │
├──────────────────────────────────────────────────────────────────────┤
│ 3. TELEMETRY (our core)   8-byte messages · scheduler · link          │
│                           emulator (M64 profile) · surface decoder    │
├──────────────────────────────────────────────────────────────────────┤
│ 2. ONBOARD INTELLIGENCE   YOLO11n · TensorRT · ByteTrack ·            │
│                           geolocator · contact fusion                 │
├──────────────────────────────────────────────────────────────────────┤
│ 1. VEHICLE HARDWARE       Jetson Orin Nano · Pixhawk + ArduSub ·      │
│    (target)               6 vectored thrusters · acoustic modem ·     │
│                           camera, IMU, depth, altimeter · SolidWorks  │
└──────────────────────────────────────────────────────────────────────┘
```

**How to read it:** the bottom layer is the physical vehicle. Each layer above runs on or supports the one below. Layer 3 — telemetry — is where most of *our own* innovation sits.

---

## 2. Layer 1 — Vehicle hardware (target design)

### 2.1 Vehicle design — SolidWorks CAD ✅
- **What:** professional 3D mechanical design software.
- **In our system:** the team designed the vehicle in SolidWorks and exported it directly into our robotics software (as a URDF robot description), so the simulated vehicle has the real shape, mass and thruster positions.
- **Key facts:** 0.714 m long × 0.564 m wide × 0.220 m tall; 12.36 kg; cylindrical hull with front dome; 6 vectored thrusters.
- **Why:** industry-standard CAD; the direct export means one design drives both manufacturing and simulation.
- **Ours?** The design is ours; the software is a commercial tool.
- **Slide line:** *"Our own vehicle design, carried straight from CAD into simulation."*

### 2.2 Thruster layout — 6 vectored thrusters (~60 W class) 🎯
- **What:** six electric propellers mounted at angles: front pair 25° down, middle pair 45° up, back pair 10° up.
- **In our system:** in simulation M1 the vehicle is moved kinematically; thruster physics come in M2.
- **Why:** angled thrusters let six motors push and turn the vehicle in all six directions (forward/back, sideways, up/down, roll, pitch, yaw). We verified mathematically that the layout is **fully actuated** (thrust matrix of full rank 6).
- **Ours?** Layout is ours; the thrusters are **imported** components.
- **Slide line:** *"Six vectored thrusters give full control in all six directions."*

### 2.3 Edge computer — NVIDIA Jetson Orin Nano 8 GB 🎯
- **What:** a credit-card-sized computer with a built-in AI accelerator (GPU), designed for robots and drones.
- **In our system:** runs the detector and the whole onboard pipeline. In the demo, a laptop runs the software but the detector is **capped to the frame rate measured on the Jetson**, so the demo behaves like the real vehicle.
- **Why:** runs neural networks at a few watts to ~25 W (affordable on battery); NVIDIA's TensorRT software gives large speed-ups; widely used in robotics, with rugged industrial versions of the same module.
- **Alternatives considered:** Raspberry Pi + AI accelerator stick (cheaper, much less AI performance); larger Jetson modules (more power than we need).
- **Ours?** **Imported** component.
- **Status:** benchmark kit written; **Jetson speed and power figures ⏳ pending** — do not quote any.
- **Slide line:** *"AI runs on the vehicle itself, on an NVIDIA Jetson edge computer."*

### 2.4 Autopilot — Pixhawk flight controller with ArduSub firmware 🎯 / 🗓
- **What:** Pixhawk is a widely used open-hardware autopilot board; ArduSub is the free, open-source autopilot software (part of ArduPilot) for underwater vehicles.
- **In our system:** will handle low-level control — keeping depth and heading, driving the thrusters. In M2 we run the real ArduSub code against our simulator ("software-in-the-loop") with our custom 6-thruster frame.
- **Why:** mature, open-source, used on many commercial ROVs; separates safety-critical control (autopilot) from AI (Jetson).
- **Ours?** Open-source software, **imported** hardware.
- **Slide line:** *"Proven open-source autopilot for control; our AI on top."*

### 2.5 Acoustic modem — low-rate modem class (Water Linked M64 profile) 🎯
- **What:** a device that sends data through water as sound.
- **In our system:** the link between vehicle and surface. We model it with the **published specification of the Water Linked Modem-M64**: 64 bits per second, 8-byte packets, ~0.5 s latency, one direction at a time (half-duplex), up to 200 m.
- **Why:** sound is the only practical way to communicate over hundreds of metres underwater (radio dies within metres in seawater; light only reaches metres in murky water). A low-cost, very slow modem is the hardest realistic case — if it works at 64 bps, it works on faster modems too.
- **Honest note:** the M64 is **no longer on sale**; we use its published spec as a representative profile. The system works with any modem — only a configuration file changes.
- **Ours?** Off-the-shelf class of device (**imported**).
- **Slide line:** *"Designed for the hardest case: a 64 bit-per-second acoustic link."*

### 2.6 Sensors 🎯 (simulated now ✅)

| Sensor | Role | Simulated as |
|---|---|---|
| Camera (640×480, 90° view, tilted 45° down, 10 frames/s) | Sees objects on the seabed | Gazebo camera with a vehicle-mounted spotlight |
| IMU (motion sensor) | Orientation and motion | Gazebo IMU, 100 Hz |
| Depth (pressure) sensor | Vehicle depth | True depth + 2 cm noise |
| Altimeter (downward echo sounder) | Height above seabed — needed to place objects on the map | Downward laser ray |
| Navigation estimate | Where the vehicle thinks it is (no GPS underwater) | True position + drift of 0.5% of distance travelled + 0.5° heading bias |

**Slide line:** *"Standard AUV sensors; navigation errors modelled realistically."*

---

## 3. Layer 2 — Onboard intelligence

### 3.1 Detector — YOLO11 nano (Ultralytics) ✅
- **What:** YOLO ("You Only Look Once") is the most widely used family of real-time object detectors; "nano" is the smallest version (about 2.6 million parameters per Ultralytics' published figures).
- **In our system:** looks at each camera frame and draws boxes around four classes: **debris** (our defence proxy for man-made objects), starfish, sea urchin, scallop.
- **Why:** fast enough for real-time use on an edge computer; mature tools for training and export; nano size fits the Jetson's power budget.
- **Alternatives considered:** larger YOLO models (more accurate, too heavy for our power budget); two-stage detectors such as Faster R-CNN (accurate but slow).
- **Measured ✅:** on 2,229 **real** underwater test images the detector never saw during training — mAP50 **0.705** overall; debris 0.386, starfish 0.886, sea urchin 0.920, scallop 0.627.
- **Honest note:** debris is the hardest class (deep-sea debris is dark, cluttered and varied). The pipeline is detector-agnostic: a better or customer-trained model replaces one file.
- **Licence note:** Ultralytics is AGPL-3.0 — fine for research; a switch to an Apache/MIT-licensed detector is planned for a product (M3).
- **Ours?** Model architecture and tools are open-source; **our trained model (weights) is ours**.
- **Slide line:** *"A compact real-time detector, trained on real underwater imagery."*

### 3.2 PyTorch ✅
- **What:** the most widely used deep-learning framework.
- **In our system:** trains and runs the detector (version 2.11 with CUDA 12.8, needed for our laptop's new GPU).
- **Slide line:** *"Trained with PyTorch on a GPU."*

### 3.3 ONNX and TensorRT 🎯
- **What:** ONNX is a standard file format for neural networks; TensorRT is NVIDIA's optimiser that makes a network run much faster on NVIDIA hardware.
- **In our system:** the trained model is exported to ONNX ✅, then turned into a TensorRT engine (FP16, 16-bit arithmetic) **on the Jetson itself** for maximum speed.
- **Slide line:** *"Optimised for the edge with NVIDIA TensorRT."*

### 3.4 Tracker — ByteTrack ✅
- **What:** a multi-object tracking algorithm (Zhang et al., ECCV 2022), used through the MIT-licensed `supervision` library.
- **In our system:** recognises that the same object in consecutive frames is one object, and gives it an ID. A track is only trusted after **3 consistent sightings**, which filters out one-frame false alarms before they cost any bandwidth.
- **Why:** fast and accurate; also uses low-confidence detections to keep tracks alive through blur and murk — common underwater.
- **Slide line:** *"One object seen in 200 frames becomes one report, not 200."*

### 3.5 Geolocator — from pixel to seabed position ✅ (ours)
- **What:** our own geometry module.
- **In our system:** converts the box position in the image into a position on the seabed, using the camera geometry, the vehicle's estimated position and its height above the seabed. Every position gets an **uncertainty radius** that combines navigation error, pixel error and altimeter error.
- **Measured ✅:** reported contacts about **0.25 m** from the true object position (demo scenario).
- **Slide line:** *"Every report says where — and how sure we are."*

### 3.6 Contact database — fusion ✅ (ours)
- **What:** our own module that merges repeated sightings into one **contact**.
- **In our system:** averages positions, weighting precise sightings more (inverse-variance weighting), and recognises an object seen again on a later survey pass.
- **Slide line:** *"Repeated sightings make each contact more accurate, not more expensive."*

---

## 4. Layer 3 — Telemetry (our core innovation)

### 4.1 8-byte message format ✅ (ours)
- **What:** our own binary message design. Every message is exactly **64 bits** — one acoustic packet.
- **Two message types:**
  - **Heartbeat** — vehicle position, depth, heading, battery, mission state, number of pending reports (every 5–15 s)
  - **Contact** — object ID, class, confidence, position (0.5 m steps), depth, uncertainty, time
- **Why:** a JPEG photo takes ~83 minutes at 64 bps; the same fact written as text (JSON) ~15 s; our message **1 s**.
- **Quality:** frozen with 8 **golden test vectors** (exact expected bytes), checked end to end through the running link.
- **Slide line:** *"The whole finding fits in 8 bytes — one second on the link."*

### 4.2 Scheduler — value-of-information ✅ (ours)
- **What:** our own decision logic for *what to send next*.
- **In our system:** each 1-second slot carries the most valuable message: a heartbeat at least every 15 s; otherwise the highest-scoring contact (debris outranks marine life; new objects outrank small updates; waiting messages slowly gain priority so nothing waits forever). If nothing is worth sending, it sends nothing — saving energy.
- **Measured ✅:** debris reached the surface first; heartbeats never more than 6 s apart in testing.
- **Slide line:** *"The link always carries the most important information first."*

### 4.3 Link emulator ✅ (ours)
- **What:** our own software model of the acoustic channel.
- **In our system:** delays each packet by its airtime (1 s) plus latency (0.5 s), drops 5% of packets at random (an **assumption**, shown on screen), and discards all-zero packets as the real modem does.
- **Slide line:** *"A faithful model of a real 64 bps modem, including packet loss."*

### 4.4 Surface decoder ✅ (ours)
- **What:** our own receiver at the operator's end.
- **In our system:** unpacks the 8-byte messages and draws contacts and the vehicle's track on the operator's map. By design and by automated test, it can **only** see what crossed the link — so the demo cannot "cheat".
- **Slide line:** *"The operator sees only what really came through the link."*

---

## 5. Layer 4 — Software platform

### 5.1 Ubuntu 24.04 LTS ✅
- **What:** the most widely used Linux distribution for robotics.
- **Why:** the officially supported operating system for ROS 2 Jazzy; long-term support.

### 5.2 ROS 2 Jazzy ✅
- **What:** ROS 2 (Robot Operating System 2) is the standard open-source software framework for robots — not an operating system, but a way for many small programs ("nodes") to exchange data over named channels ("topics"). Jazzy is its long-term-support release (supported until 2029 per ROS's release schedule — verify).
- **In our system:** all 17 of our programs (camera bridge, detector, tracker, scheduler, link, decoder, metrics…) are ROS 2 nodes and start with one command.
- **Why:** industry standard in robotics, including defence robotics; the same code runs in simulation and on the real vehicle; huge ecosystem of tools.
- **Slide line:** *"Built on ROS 2 — the industry standard for robot software."*

### 5.3 Gazebo Harmonic ✅
- **What:** an open-source 3D robot simulator with physics and realistic sensors (long-term-support release, officially paired with ROS 2 Jazzy).
- **In our system:** simulates the ocean, the seabed with real object photos, the vehicle and its sensors, rendered on the GPU.
- **Why:** the standard ROS simulator; built-in underwater systems (buoyancy, hydrodynamics, thrusters) for M2; results transfer to the real vehicle.
- **Alternatives considered:** HoloOcean (more realistic underwater images, heavier setup, weaker ROS 2 integration); Stonefish (strong marine physics, smaller community).
- **Slide line:** *"Every scenario is repeatable, with known ground truth — so every number is measured."*

### 5.4 ros_gz bridges ✅
- **What:** the connectors between Gazebo and ROS 2.
- **Note:** camera images use the dedicated image bridge because the general one was too slow — we measured this.

### 5.5 Python 3.12 ✅
- **What:** programming language used for all our nodes.
- **Why:** fastest to develop in; excellent AI and robotics libraries. Performance-critical parts can move to C++ later if a measurement shows the need.

### 5.6 Foxglove ✅
- **What:** a modern robotics visualisation application (free tier; proprietary desktop app).
- **In our system:** the **operator screen** — the vehicle's camera view, the surface map with contacts, and live counters of bits sent vs video.
- **Why:** connects live to ROS 2 and replays recordings with the same layout (our demo fallback); presentation-quality panels.
- **Slide line:** *"Operator view: what the vehicle sees vs what the operator receives."*

### 5.7 rosbag2 with MCAP recording format ✅
- **What:** ROS 2's recorder; MCAP is an open file format for robotics data.
- **In our system:** records every mission (about 150 MB per mission after compression); recordings replay in Foxglove exactly like the live system.

### 5.8 Development machine ✅
- HP OMEN laptop, NVIDIA RTX 5060 Laptop GPU (8 GB), NVIDIA driver 580 — trains the detector and runs the simulation.

---

## 6. Layer 5 — Data and training

### 6.1 Datasets ✅

| Dataset | What | Used for | Licence / credit |
|---|---|---|---|
| **TrashCan 1.0** (Hong, Fulton & Sattar, University of Minnesota, 2020) | Deep-sea ROV images of marine debris, imagery from JAMSTEC's J-EDI archive | debris and starfish | Free for academic research; **credit authors and JAMSTEC in every presentation**; commercial use needs JAMSTEC permission |
| **DUO** (Liu et al., 2021) | Shallow-water images of marine life | starfish, sea urchin, scallop | No licence stated; credit the authors |
| **Simulator frames** (ours) | 1,929 frames, 3,795 automatically generated labels from 10 simulated seabed layouts | Teaching the detector what the simulator looks like | Ours |

- **Total real data:** 14,994 images, split by **video** so near-identical frames never appear in both training and testing (verified: zero leakage).
- **Honesty rule:** the demo seabed is never used for training, and the seabed's object photos come only from test images.

### 6.2 Training ✅
- **v1:** trained on real images (about 2.15 h on the laptop GPU, 129 epochs, stopped automatically when accuracy levelled off).
- **v2 (used in the demo):** fine-tuned on real + simulator images; kept real-image accuracy (test mAP50 0.703 → 0.705).
- **Slide line:** *"Trained on real underwater imagery, adapted to simulation without losing real-world accuracy."*

---

## 7. Layer 6 — Development and quality

| Practice / tool | What it gives us |
|---|---|
| **GitHub + Git LFS** | Version history; large files (3D meshes, model weights) stored efficiently |
| **pytest** (automated tests) | Every core module tested: message format, link timing, scheduler rules, geometry, fusion, metrics |
| **Golden test vectors** | The message format can never change by accident; any future implementation (e.g. in C++ on a microcontroller) can be checked byte for byte |
| **ruff** | Consistent, clean code |
| **Honesty rules in code** | Tests fail if the operator side reads anything but the link, or perception reads simulator truth |
| **Reproducibility** | Fixed random seeds, recorded package versions, scripted data preparation and training |
| **AI coding assistants, with verification** | Faster development; every reported number checked against real command output (we caught and corrected cases where an assistant reported values that were not real) |

**Slide line:** *"Tested, reproducible, and honest by design."*

---

## 8. How the pieces connect

```
 CAMERA ──► YOLO11n ──► ByteTrack ──► Geolocator ──► Contact DB ──► Scheduler ──► 8-byte encoder
 (Gazebo)   (PyTorch/     (supervision)  (ours)        (ours)          (ours)         (ours)
            TensorRT)                                                                   │
                                                                                        ▼
 FOXGLOVE ◄── Surface decoder ◄───────────── 64 bps acoustic link (emulated, M64 profile)
 operator map    (ours)

 All boxes are ROS 2 nodes · recorded with rosbag2/MCAP · simulated in Gazebo Harmonic
```

---

## 9. What we have measured so far

*Simulation; demo scenario (the scenario the system was tuned on).*

| Measurement | Result |
|---|---|
| Data sent for the whole survey mission | **5,312 bits** (83 s of link time) |
| Same camera stream as H.264 video over the same link | 23.8 million bits → about **4.3 days** |
| Bandwidth saving vs video | **over 4,000×** (≈4,480×; ≈37,000× vs raw JPEG frames) |
| Time from onboard detection to operator's map | **≈ 1.5 s** |
| Position accuracy of reported contacts | **≈ 0.25 m** |
| False contacts | **0** (in the tuning runs) |
| Objects reported to the operator | 1 of 12 in the tuning runs — detector-limited; being verified in the final recording |
| Detector on real test images | mAP50 **0.705** (debris 0.386) |
| Mission recording size | ≈ 150 MB |
| Jetson speed and power | ⏳ **pending** |

**Numbers that may still change** before the pitch: bits sent and number of contacts (final recording). Use the final `summary.json` values.

---

## 10. What is ours, and what is off-the-shelf

| Ours (NewtonBotics) | Open-source (free, widely used) | Imported hardware |
|---|---|---|
| Vehicle design and thruster layout | ROS 2, Gazebo, Python, PyTorch | Jetson Orin Nano compute module |
| 8-byte message format and codec | Ultralytics YOLO (AGPL), ByteTrack | Pixhawk autopilot board |
| Scheduler, link emulator, surface decoder | ArduSub autopilot software | Thrusters |
| Geolocator and contact fusion | TensorRT (NVIDIA, free), Foxglove (free tier) | Acoustic modem (class) |
| Trained detector weights, simulator dataset | | |
| System integration, tests, evaluation | | |

> **For the pitch:** "The intelligence, the message design and the integration are ours. The compute module, autopilot and thrusters are currently imported; indigenisation is on our roadmap." This matches our TRL 3–4 claim.

---

## 11. Slide-ready one-liners

| Topic | One line |
|---|---|
| Whole system | We send what the vehicle understood, not what it saw. |
| Edge AI | AI runs on the vehicle, on an NVIDIA Jetson edge computer. |
| Detector | A compact real-time detector trained on real underwater imagery. |
| Tracking | One object seen in 200 frames becomes one report. |
| Geolocation | Every report says where — and how sure we are. |
| Message format | A whole finding fits in 8 bytes — one second on a 64 bps link. |
| Scheduler | The most important information always goes first. |
| Link | Designed for the hardest case: a 64 bit-per-second acoustic modem. |
| Platform | Built on ROS 2 and Gazebo — industry standards, same code for the real vehicle. |
| Operator view | What the vehicle sees vs what the operator receives. |
| Results | Over 4,000× less data than H.264 video, with reports in about 1.5 s. |
| Quality | Tested, reproducible, and honest by design. |

---

## 12. Likely questions about the tech choices

| Question | Short answer |
|---|---|
| Why not stream video with better compression? | Even excellent video compression needs hundreds of kbps; the link has 64 bps. Changing *what* is sent closes the gap; compression cannot. |
| Why such a small detector? | Power. The vehicle runs on batteries; a nano model runs in real time at a few watts. The pipeline accepts a larger model if the platform allows. |
| Why ROS 2? | Industry standard; the same software runs in simulation and on the vehicle; huge ecosystem. |
| Why Python and not C++? | Speed of development. Nodes can be rewritten in C++ if measurements show a need; none has so far. |
| Why simulation instead of a real vehicle? | Repeatable scenarios with exact ground truth let us *measure* accuracy and bandwidth. Hardware trials follow in the next milestone. |
| Why model a discontinued modem? | Its published spec is a well-documented example of the low-cost, low-rate class. Our system is modem-agnostic — one configuration file. |
| Why not radio or Wi-Fi? | Radio is absorbed within metres in seawater. Acoustic is the only practical long-range option. |
| Is the detector good enough? | 0.705 mAP50 overall on real test images; debris (0.39) is the hardest class. Better or customer-specific training data plugs straight in. |
| What's indigenous? | Software, message design, integration, vehicle design. Compute, autopilot and thrusters are imported today. |

---

## 13. Credits to show in the deck

> Training data: **TrashCan 1.0** (J. Hong, M. Fulton, J. Sattar, University of Minnesota, 2020; imagery © JAMSTEC J-EDI) and **DUO** (C. Liu et al., 2021). Link profile based on the published specification of the Water Linked Modem-M64. Simulation: Gazebo Harmonic, ROS 2 Jazzy. Detector: Ultralytics YOLO11.
