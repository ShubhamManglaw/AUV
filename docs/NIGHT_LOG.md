# PS11-AUV Overnight Autonomous Work Log

## Executive Summary
- **Overall Status:** In progress (Q0, Q1 complete; working through Q2–Q10)
- **Tasks Complete:** Q0 (T2.9 Report & Deployment), Q1 (T2.6 Tracker)
- **Tasks In Progress / Next:** Q2 (T2.7 Geolocator)
- **Tasks Blocked:** None
- **Three Most Important Items for Human Review:**
  1. **Detector Fine-Tuning (T2.9 / Q0):** Real test mAP50 improved to 0.705 (+0.002 overall, +0.035 debris); `ml/weights/best_v2.pt` deployed.
  2. *(Pending completion of overnight queue)*
  3. *(Pending completion of overnight queue)*

---

## Q0 — T2.9 Report & Model Retention Verification
- **Status:** DONE
- **Command:** `python3 tools/report_t29_results.py`
- **Output & Tables (Verbatim):**

### (a) Real Test-Set Evaluation (ml/data/ps11.yaml, 2,229 images)
```text
                 Class     Images  Instances      Box(P          R      mAP50  mAP50-95)
                   all       2229      10372       0.74      0.662      0.705      0.529

| Class | ps11_v1 mAP50 | ps11_v2 mAP50 | Δ mAP50 | ps11_v1 mAP50-95 | ps11_v2 mAP50-95 | Δ mAP50-95 |
|---|---|---|---|---|---|---|
| **overall** | **0.703** | **0.705** | **+0.002** | **0.533** | **0.529** | **-0.004** |
| debris      | 0.351 | 0.386 | +0.035 | 0.189 | 0.205 | +0.016 |
| starfish    | 0.886 | 0.886 | -0.000 | 0.708 | 0.704 | -0.004 |
| sea_urchin  | 0.926 | 0.920 | -0.006 | 0.738 | 0.730 | -0.008 |
| scallop     | 0.649 | 0.627 | -0.022 | 0.498 | 0.478 | -0.020 |

Criterion (a): Overall mAP50 drop >= -0.02: Δ = +0.0017
>>> CRITERION (a) PASSED: Real test-set mAP50 preserved! <<<
```

### (b) Simulation Validation Evaluation (ml/data/sim_val.yaml, seed 110)
```text
                 Class     Images  Instances      Box(P          R      mAP50  mAP50-95)
                   all        264        399       0.46     0.0816      0.079     0.0567

| Class | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| **all** | 0.460 | 0.082 | **0.079** | **0.057** |
| debris      |     0.624 |  0.235 | 0.235 |    0.188 |
| starfish    |     0.000 |  0.000 | 0.000 |    0.000 |
| sea_urchin  |     1.000 |  0.057 | 0.065 |    0.031 |
| scallop     |     0.215 |  0.034 | 0.016 |    0.008 |
```

### (c) 12-Object Demo Detectability Check with ps11_v2
```text
================================================================================
T2.5 DETECTABILITY RESULTS TABLE
================================================================================
| id  | class        | size_m | detected with correct class?   | best conf | box size (px)  |
|-----|--------------|--------|--------------------------------|-----------|----------------|
| 0   | debris       | 1.085  | NO                             | -         | -              |
| 1   | debris       | 0.704  | NO                             | -         | -              |
| 2   | debris       | 0.681  | NO                             | -         | -              |
| 3   | debris       | 1.083  | YES                            | 0.71      | 36x68          |
| 4   | debris       | 0.735  | YES                            | 0.72      | 77x44          |
| 5   | starfish     | 0.246  | NO                             | -         | -              |
| 6   | starfish     | 0.205  | NO                             | -         | -              |
| 7   | starfish     | 0.294  | YES                            | 0.82      | 23x23          |
| 8   | sea_urchin   | 0.100  | NO                             | -         | -              |
| 9   | sea_urchin   | 0.117  | NO                             | -         | -              |
| 10  | scallop      | 0.125  | NO                             | -         | -              |
| 11  | scallop      | 0.113  | NO                             | -         | -              |
================================================================================
Measured detector publish rate: 4.18 Hz (target: ~5 Hz)
================================================================================
```
- **Deployment Decision:** Criterion (a) passed; `ml/weights/best_v2.pt` deployed in `perception.yaml`.

---

## Q1 — T2.6 Tracker Node (ByteTrack)
- **Status:** DONE
- **Deliverables:**
  - `ros2_ws/src/ps11_perception/ps11_perception/tracker.py` (pure logic with ByteTrack, min_hits confirmation, majority-vote class, running mean confidence over last 10 hits)
  - `ros2_ws/src/ps11_perception/test/test_tracker.py` (unit tests covering acceptance checks)
  - `ros2_ws/src/ps11_perception/ps11_perception/tracker_node.py` (ROS 2 wrapper publishing `/vehicle/perception/tracks`)
  - `ros2_ws/src/ps11_perception/setup.py` (entry point `tracker`)
- **Key Commands & Output (Verbatim):**
```text
$ python -m pytest ros2_ws/src/ps11_perception/test/test_tracker.py -v
============================= test session starts ==============================
collected 4 items

ros2_ws/src/ps11_perception/test/test_tracker.py::test_fewer_than_min_hits_gives_none PASSED [ 25%]
ros2_ws/src/ps11_perception/test/test_tracker.py::test_one_object_over_30_frames_gives_1_track_id PASSED [ 50%]
ros2_ws/src/ps11_perception/test/test_tracker.py::test_majority_vote_class PASSED [ 75%]
ros2_ws/src/ps11_perception/test/test_tracker.py::test_low_confidence_filtered PASSED [100%]

========================= 4 passed, 1 warning in 0.95s =========================
```
```text
$ cd ros2_ws && python -m colcon build --symlink-install --packages-select ps11_perception
Starting >>> ps11_perception
Finished <<< ps11_perception [1.10s]

Summary: 1 package finished [1.22s]
```

---
