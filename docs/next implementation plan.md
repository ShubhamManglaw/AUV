# PS11-AUV: Execution Plan — Remaining M1 Work

**Created:** 2026-09-28 21:00 IST
**Pitch date context:** Plan says Day 1 = Sep 26. Today is Sep 28 (end of Day 2). Days 3–4 remain.

---

## Where you are right now

### ✅ Done (12 tasks)

| Task | What exists on disk |
|---|---|
| T0.1–T0.2 | Scaffold, [`env.sh`](file:///home/shubham/modi/tools/env.sh), [`check_env.sh`](file:///home/shubham/modi/tools/check_env.sh), [`requirements.lock`](file:///home/shubham/modi/requirements.lock) |
| T0.3 | [`ps11_interfaces/msg/`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg) (10 .msg files), [`ps11_common/`](file:///home/shubham/modi/ros2_ws/src/ps11_common/ps11_common) ([`classes.py`](file:///home/shubham/modi/ros2_ws/src/ps11_common/ps11_common/classes.py), [`image_utils.py`](file:///home/shubham/modi/ros2_ws/src/ps11_common/ps11_common/image_utils.py), [`params.py`](file:///home/shubham/modi/ros2_ws/src/ps11_common/ps11_common/params.py)), all config YAMLs in [`config/`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config) |
| T1.0 | SolidWorks export in [`tools/sw_export_raw/`](file:///home/shubham/modi/tools/sw_export_raw), [`vehicle.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/vehicle.yaml) |
| T1.1 | [`convert_sw_export.py`](file:///home/shubham/modi/tools/convert_sw_export.py), [`ps11_description/`](file:///home/shubham/modi/ros2_ws/src/ps11_description) with [`ps11.urdf.xacro`](file:///home/shubham/modi/ros2_ws/src/ps11_description/urdf/ps11.urdf.xacro), [`ps11_core.urdf.xacro`](file:///home/shubham/modi/ros2_ws/src/ps11_description/urdf/ps11_core.urdf.xacro), [`meshes/`](file:///home/shubham/modi/ros2_ws/src/ps11_description/meshes) |
| T1.2 | [`ocean_demo_kinematic.sdf`](file:///home/shubham/modi/ros2_ws/src/ps11_gazebo/worlds/ocean_demo_kinematic.sdf), [`sim.launch.py`](file:///home/shubham/modi/ros2_ws/src/ps11_gazebo/launch/sim.launch.py), [`bridge.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_gazebo/config/bridge.yaml) |
| T2.1–T2.3 | [`prepare_dataset.py`](file:///home/shubham/modi/ml/prepare_dataset.py), [`eval.py`](file:///home/shubham/modi/ml/eval.py), [`export.py`](file:///home/shubham/modi/ml/export.py), [`DATASETS.md`](file:///home/shubham/modi/ml/DATASETS.md), [`best.pt`](file:///home/shubham/modi/ml/weights/best.pt), [`eval.md`](file:///home/shubham/modi/ml/results/eval.md) |
| T3.1 | [`codec.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/codec.py), [`golden_vectors.json`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/test/golden_vectors.json), [`test_codec.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/test/test_codec.py) |
| T3.2 | [`link_model.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/link_model.py), [`link_emulator_node.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/link_emulator_node.py), [`link_profiles.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/link_profiles.yaml), [`test_link_model.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/test/test_link_model.py) |
| T3.4 | [`surface_decoder_node.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/surface_decoder_node.py), [`surface_state.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/surface_state.py), [`test_surface_decoder.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/test/test_surface_decoder.py), [`test_surface_state.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/test/test_surface_state.py) |

### 🔴 Not done (14 M1 tasks)

| Task | What's missing | Where code goes | Critical? |
|---|---|---|---|
| **T1.3** | odom_noise, depth_sim, range_adapter — **only [`__init__.py`](file:///home/shubham/modi/ros2_ws/src/ps11_nav/ps11_nav/__init__.py) exists** | [`ps11_nav/ps11_nav/`](file:///home/shubham/modi/ros2_ws/src/ps11_nav/ps11_nav) | YES — blocks T1.5, T2.7 |
| **T1.4** | `make_seabed.py` — **doesn't exist in [`tools/`](file:///home/shubham/modi/tools)** | [`tools/make_seabed.py`](file:///home/shubham/modi/tools) | YES — camera sees blank floor |
| **T1.5** | waypoint_follower — **zero code** | [`ps11_nav/ps11_nav/`](file:///home/shubham/modi/ros2_ws/src/ps11_nav/ps11_nav) | YES — vehicle doesn't move |
| **T2.5** | detector node — **only [`__init__.py`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception/__init__.py) exists** | [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception) | YES — no detections |
| **T2.6** | tracker node | [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception) | YES — no tracks |
| **T2.7** | geolocator + geo.py | [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception) | YES — no map positions |
| **T2.8** | contact_db | [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception) | YES — no contacts |
| **T3.3** | scheduler + policy.py — **zero code** | [`ps11_telemetry/ps11_telemetry/`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry) | YES — no messages sent |
| T4.1 | Bringup launch files | [`ps11_bringup/ps11_bringup/`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/ps11_bringup) | Needed for demo |
| T4.2 | Metrics node — **only [`__init__.py`](file:///home/shubham/modi/ros2_ws/src/ps11_eval/ps11_eval) exists** | [`ps11_eval/ps11_eval/`](file:///home/shubham/modi/ros2_ws/src/ps11_eval/ps11_eval) | Needed for numbers |
| T4.3 | Foxglove layout | [`ps11_bringup/`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup) (new `foxglove/` subdir) | Needed for demo |
| T4.4 | Integration tuning | Across all config in [`config/`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config) | Final pass |
| T4.5 | Charts/experiments | [`tools/`](file:///home/shubham/modi/tools) (`video_equiv.py`, `make_charts.py`), [`results/`](file:///home/shubham/modi/results) | For the deck |
| T4.6 | Recording | [`results/`](file:///home/shubham/modi/results) (bag + video, gitignored) | For backup |

---

## The plan, in 5 parts

### Part 1: Make the vehicle fly and see objects (T1.3 → T1.5 → T1.4)

These three tasks give you a vehicle that autonomously surveys a seabed with recognizable objects. Without this, every downstream node has no input.

**Dependency chain:**
```
T1.3 (nav models) → T1.5 (waypoint follower) → T1.4 (seabed)
                                                      ↓
                                              camera sees real objects
```

#### T1.3 — Nav sensor models (~2h)

> [!IMPORTANT]
> This is the first task to start. Everything in the perception chain depends on `/vehicle/nav/odom` and TF.

**Where:** Create 3 new files in [`ps11_nav/ps11_nav/`](file:///home/shubham/modi/ros2_ws/src/ps11_nav/ps11_nav):
- `odom_noise_node.py`
- `depth_sim_node.py`
- `range_adapter_node.py`

Update [`setup.py`](file:///home/shubham/modi/ros2_ws/src/ps11_nav/setup.py) with entry points.

**Config:** Parameters from [`nav.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/nav.yaml).

**Deliverables:**
- `odom_noise`: `/sim/gt/odom` → `/vehicle/nav/odom` + TF `map→base_link` (0.5% distance random walk + 0.5° heading bias)
- `depth_sim`: ground-truth z → `/vehicle/depth` ([`Depth.msg`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg/Depth.msg)) with N(0, 0.02m) noise
- `range_adapter`: `/vehicle/altimeter/scan` → `/vehicle/altitude` (LaserScan → Range)

**Pure logic test:** error model with fixed seed; after 100m straight, horizontal error ~0.5m.

**Acceptance:** TF `map→base_link` published; `/vehicle/altitude` ≈ 2.5m at 2.5m AGL.

---

#### T1.5 — Waypoint follower (~2-3h)

**Where:** Create `waypoint_follower_node.py` in [`ps11_nav/ps11_nav/`](file:///home/shubham/modi/ros2_ws/src/ps11_nav/ps11_nav). Update [`setup.py`](file:///home/shubham/modi/ros2_ws/src/ps11_nav/setup.py).

**Config:** [`mission.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/mission.yaml) (lawnmower pattern, 4m leg spacing, 2.5m altitude, 1.0 m/s).

**Acceptance:** Demo scenario completes in ~4 min; `/vehicle/mission/state` transitions transit→survey→return→idle.

---

#### T1.4 — Seabed decal generator (~3h)

**Where:** Create [`tools/make_seabed.py`](file:///home/shubham/modi/tools).

**Config:** [`seabed.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/seabed.yaml) (demo scenario: 50m×30m, 12 objects).

**Output:** Tiles into [`ps11_gazebo/models/`](file:///home/shubham/modi/ros2_ws/src/ps11_gazebo/models), ground truth into [`config/world_objects.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config).

**Key rules:**
- Images only from [`ml/data/test_manifest.txt`](file:///home/shubham/modi/ml) (H4)
- Fixed seed = reproducible output

**Acceptance:** Camera at 2.5m altitude shows recognizable objects; script refuses non-test images.

---

### Part 2: Perception chain (T2.5 → T2.6 → T2.7 → T2.8)

These 4 tasks are serial. Each consumes the output of the previous one. All go into [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception). Update [`setup.py`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/setup.py) with entry points after each.

```
T2.5 (detector) → T2.6 (tracker) → T2.7 (geolocator) → T2.8 (contact_db)
     images           detections         tracks              observations
                                                                  ↓
                                                             /vehicle/contacts
```

#### T2.5 — Detector node (~3h)

**Where:** Create `detector_node.py` in [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception).

**Config:** [`perception.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/perception.yaml) `detector:` section. Model at [`ml/weights/best.pt`](file:///home/shubham/modi/ml/weights/best.pt).

**Classes:** Must load from [`classes.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/classes.yaml) via [`classes.py`](file:///home/shubham/modi/ros2_ws/src/ps11_common/ps11_common/classes.py). No hard-coded IDs.

**Key details:**
- Rate-capped to `max_rate_hz` (5 Hz), dropping intermediate frames
- Publishes `Detection2DArray` on `/vehicle/perception/detections`
- Annotated image with boxes + H3 banner at `annotate_rate_hz`
- Uses [`image_utils.py`](file:///home/shubham/modi/ros2_ws/src/ps11_common/ps11_common/image_utils.py) for conversion — **no `cv_bridge`**

**Acceptance:** `ros2 topic hz` shows ~5 Hz detections; annotated image has banner.

---

#### T2.6 — Tracker node (~2h)

**Where:** Create `tracker_node.py` in [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception).

**Config:** [`perception.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/perception.yaml) `tracker:` section.

**Output message:** [`TrackArray.msg`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg/TrackArray.msg) on `/vehicle/perception/tracks`.

**Key details:**
- Uses `supervision` ByteTrack
- Only publishes **confirmed** tracks (≥ `min_hits`=5 frames, mean confidence ≥ 0.4)
- Class = majority vote, confidence = mean of last 10 hits

**Acceptance:** Object over 30 frames → one track ID; < `min_hits` → not published.

---

#### T2.7 — Geolocator node (~3-4h)

**Where:** Create `geo.py` (pure maths, no ROS) and `geolocator_node.py` in [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception). Tests in a new `test/` dir.

**Config:** [`perception.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/perception.yaml) `geolocator:` section.

**Output message:** [`ObservationArray.msg`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg/ObservationArray.msg) on `/vehicle/perception/observations`.

**The maths (in `geo.py`):**
1. Ray: `r = K⁻¹ [u, v, 1]ᵀ`
2. Transform to map frame via TF
3. Intersect with seabed plane at `z_s = vehicle_z - altitude`
4. Uncertainty: `σ_xy = sqrt(σ_nav² + (s·σ_px/fx)² + (σ_alt·|d_xy|/|dz|)²)`

**Acceptance:** Unit test with synthetic 45° camera passes; sim observations land within ~1m of ground truth from [`world_objects.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config).

---

#### T2.8 — Contact database node (~2-3h)

**Where:** Create `fusion.py` (pure logic) and `contact_db_node.py` in [`ps11_perception/ps11_perception/`](file:///home/shubham/modi/ros2_ws/src/ps11_perception/ps11_perception).

**Config:** [`perception.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/perception.yaml) `contact_db:` section.

**Output message:** [`ContactArray.msg`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg/ContactArray.msg) on `/vehicle/contacts` at 2 Hz.

**Key details:**
- Associate by track ID first, then nearest same-class within `max(gate_m, 3σ)` (gate 2m)
- Inverse-variance weighted mean position; σ floor 0.3m
- Also publishes `/vehicle/markers` for Foxglove

**Acceptance:** Same object on two passes → one contact; fused σ shrinks but never below 0.3m.

---

### Part 3: Close the loop — Scheduler (T3.3)

This is the core claim of the project. With contacts flowing in and the link emulator already working, the scheduler connects them.

```
/vehicle/contacts  ──┐
/vehicle/nav/odom  ──┤
/vehicle/mission/state ─┤──→ scheduler ──→ /link/tx
/link/tx_ready  ──────┘
```

#### T3.3 — Scheduler + policy.py (~3-4h)

**Where:** Create `policy.py` (pure logic) and `scheduler_node.py` in [`ps11_telemetry/ps11_telemetry/`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry). Tests alongside existing tests in [`test/`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/test).

**Config:** [`scheduler.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/scheduler.yaml).

**Uses:** [`codec.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/codec.py) to encode messages. [`classes.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/classes.yaml) for priorities.

**Semantic policy:**
1. On `tx_ready`, fill frame slots
2. Heartbeat if ≥15s since last
3. Rank candidates: `score = priority × confidence × novelty × age_boost`
4. Track what was sent per contact ID
5. If frame empty and ≥5s since heartbeat, add heartbeat; else send nothing

**Sends frames to:** [`link_emulator_node.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/link_emulator_node.py) via `/link/tx` ([`LinkFrame.msg`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg/LinkFrame.msg)).

**Acceptance:** Debris before scallop at same confidence; heartbeat ≤15s; small moves don't trigger updates.

---

### Part 4: Bringup + Metrics + Foxglove (T4.1 → T4.2 → T4.3)

These make it a demo you can show.

#### T4.1 — Bringup launch files (~2h)

**Where:** Create launch files in [`ps11_bringup/`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup) (new `launch/` subdir):
- `demo.launch.py` — starts everything
- `vehicle.launch.py` — nav + perception + scheduler
- `surface.launch.py` — surface_decoder

Reuses [`sim.launch.py`](file:///home/shubham/modi/ros2_ws/src/ps11_gazebo/launch/sim.launch.py) from ps11_gazebo.

**Acceptance:** `ros2 launch ps11_bringup demo.launch.py` starts everything; `ros2 node list` matches §14.1.

---

#### T4.2 — Metrics node (~3h)

**Where:** Create `metrics_node.py` in [`ps11_eval/ps11_eval/`](file:///home/shubham/modi/ros2_ws/src/ps11_eval/ps11_eval). Update [`setup.py`](file:///home/shubham/modi/ros2_ws/src/ps11_eval/setup.py).

**Output:** [`DemoCounters.msg`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg/DemoCounters.msg) at 1 Hz on `/eval/counters`. Writes [`results/`](file:///home/shubham/modi/results)`run_<timestamp>/summary.json` at shutdown.

**Reads:** `/sim/gt/odom`, ground truth from [`world_objects.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config), `/vehicle/camera/image_raw`, `/vehicle/contacts`, `/surface/contacts`, `/link/stats` ([`LinkStats.msg`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg/LinkStats.msg)).

**Acceptance:** Full demo run writes `summary.json`; counters on `/eval/counters` update at 1 Hz.

---

#### T4.3 — Foxglove layout (~2h)

**Where:** Create `foxglove/ps11_pitch_layout.json` in [`ps11_bringup/`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup).

**Acceptance:** All panels populated during a demo run; titles and banner as in plan §13.

---

### Part 5: Polish and record (T4.4 → T4.5 → T4.6)

#### T4.4 — Integration tuning (~4h)
Tune thresholds across [`perception.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/perception.yaml) and [`scheduler.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/scheduler.yaml) on demo scenario. Run 3 times without crash.

#### T4.5 — Charts (~3h)
Create `tools/video_equiv.py` and `tools/make_charts.py`. Output to [`results/`](file:///home/shubham/modi/results).

#### T4.6 — Recording (~2h)
MCAP bag + OBS screen capture → [`results/`](file:///home/shubham/modi/results) (gitignored, copy to shared drive).

---

## Recommended execution order with timing

### Tonight / Day 3 morning (highest priority)

| Order | Task | Est. | Can use Flash? | Notes |
|---|---|---|---|---|
| 1 | **T1.3** | 2h | ✅ Yes — pure logic + thin ROS wrapper | Start here. Unblocks everything. Code goes in [`ps11_nav/`](file:///home/shubham/modi/ros2_ws/src/ps11_nav/ps11_nav) |
| 2 | **T1.5** | 2-3h | ✅ Yes — straightforward control loop | Verify vehicle flies the lawnmower |
| 3 | **T1.4** | 3h | ⚠️ Careful — image processing + Gazebo model gen | Creates tiles in [`ps11_gazebo/models/`](file:///home/shubham/modi/ros2_ws/src/ps11_gazebo/models) |
| 4 | **T2.5** | 3h | ⚠️ Careful — YOLO integration, rate cap logic | **Day 2 gate:** check if detector sees objects on sim camera |

### Day 3 afternoon/evening

| Order | Task | Est. | Can use Flash? |
|---|---|---|---|
| 5 | **T2.6** | 2h | ✅ Yes — thin wrapper around supervision |
| 6 | **T2.7** | 3-4h | ✅ Yes for `geo.py` — pure geometry with tests |
| 7 | **T2.8** | 2-3h | ✅ Yes — pure fusion logic + thin wrapper |
| 8 | **T3.3** | 3-4h | ✅ Yes for `policy.py` — pure scoring logic |

### Day 4

| Order | Task | Est. |
|---|---|---|
| 9 | **T4.1** | 2h |
| 10 | **T4.2** | 3h |
| 11 | **T4.3** | 2h |
| 12 | **T4.4** | 4h |
| 13 | **T4.5** | 3h |
| 14 | **T4.6** | 2h |

**Total remaining: ~36-42h of agent-assisted work across 14 tasks.**

---

## The vertical slice shortcut

If you're running behind, there's a way to prove the core claim without the full perception chain:

> **Fake-contact demo:** Write a 50-line script that publishes synthetic [`ContactArray`](file:///home/shubham/modi/ros2_ws/src/ps11_interfaces/msg/ContactArray.msg) messages (a few debris + marine life contacts, drifting slightly). Feed that into the scheduler → [`link_emulator_node.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/link_emulator_node.py) → [`surface_decoder_node.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/surface_decoder_node.py). This proves the bandwidth story end-to-end in ~1 hour.

You already have everything for this except T3.3 (scheduler). This is your fallback if perception integration fails.

---

## What to skip if behind (cut list from plan §18.2, ordered)

1. T5.3 Jetson video
2. Error ellipse markers
3. 3D props in the world
4. T4.5 charts (use live counters instead)
5. Foxglove vehicle-side 3D panel

**Never skip:** [`codec.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/codec.py), [`link_emulator_node.py`](file:///home/shubham/modi/ros2_ws/src/ps11_telemetry/ps11_telemetry/link_emulator_node.py), scheduler (semantic), surface H1 separation, live counters, H3 banner, recorded bag.

---

## [`Reference_report.md`](file:///home/shubham/modi/Reference_report.md) fixes needed

Before relying on it for the pitch:

1. **Section cross-references:** Several places say "§5" but mean "§8" (the test output section)
2. **Worked example y-field bits:** Shows `111111110100` but should be `111111111010` (-6 steps = -3.0m)
3. **Laptop timing:** "1.7 ms inference, 406.7 FPS total pipeline" — confirm these are consistent in [`eval.md`](file:///home/shubham/modi/ml/results/eval.md)
4. **§1 wording:** "currently proves" → "demonstrates in unit tests"
5. **Cheat sheet:** Cut to 7 rows: 64 bps, 8 bytes = 1s, 5% (assumption), 0.5m, 2048s, 0.703, NOT MEASURED YET

---

## Key risk: debris detection

The number everyone sees is mAP50 = 0.703. The number that matters is **debris mAP50 = 0.351** (combined) or **0.352 on TrashCan-only** ([`eval.md`](file:///home/shubham/modi/ml/results/eval.md):L17,L31). Debris has the highest scheduler priority (1.0) in [`classes.yaml`](file:///home/shubham/modi/ros2_ws/src/ps11_bringup/config/classes.yaml). Raise this yourself before a judge finds it.

> "Our strongest overall mAP50 is 0.703, but debris — our highest-priority class — is at 0.35. That's expected: underwater trash is visually diverse. Our contingency is sim-frame fine-tuning (T2.9), and the scheduler's priority weighting means even low-confidence debris reports get sent first."
