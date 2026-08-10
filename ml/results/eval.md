# Model Evaluation Report (T2.3)

**Model:** `ml/weights/best.pt` (YOLO11n, 2.58M parameters)  
**Input Size:** 640×640  
**Test Set:** `ml/data/test_manifest.txt` (2229 images, test split, never used for training or checkpoint selection)  
**Inference Speed (Laptop):** 1.7 ms inference (406.7 FPS total pipeline) — *RTX 5060 Laptop GPU, not Jetson*  

---

## 1. Full Test Split Evaluation (Combined)

Evaluated on all 2,229 test images (1,118 TrashCan + 1,111 DUO).

| Class | Test Instances | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| **all** | **10372** | **0.728** | **0.680** | **0.703** | **0.533** |
| debris (0) | 832 | 0.453 | 0.469 | 0.351 | 0.189 |
| starfish (1) | 2122 | 0.857 | 0.840 | 0.886 | 0.708 |
| sea_urchin (2) | 7201 | 0.895 | 0.859 | 0.926 | 0.738 |
| scallop (3) | 217 | 0.708 | 0.553 | 0.649 | 0.498 |

---

## 2. TrashCan Test Split Only (Real-World Debris Benchmark)

Evaluated on 1,118 TrashCan test images from 46 unseen videos. Debris appears exclusively in TrashCan.

| Class | Test Instances | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| **all** | **934** | **0.238** | **0.291** | **0.183** | **0.098** |
| debris (0) | 832 | 0.408 | 0.542 | 0.352 | 0.190 |
| starfish (1) | 102 | 0.067 | 0.039 | 0.013 | 0.006 |
| sea_urchin (2) | 0 | — | — | — | — |
| scallop (3) | 0 | — | — | — | — |

---

## 3. DUO Test Split Only (Marine Life Benchmark)

Evaluated on 1,111 DUO test images (official DUO test partition).

| Class | Test Instances | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| **all** | **9438** | **0.823** | **0.764** | **0.835** | **0.660** |
| debris (0) | 0 | — | — | — | — |
| starfish (1) | 2020 | 0.864 | 0.881 | 0.929 | 0.742 |
| sea_urchin (2) | 7201 | 0.896 | 0.858 | 0.926 | 0.738 |
| scallop (3) | 217 | 0.709 | 0.553 | 0.649 | 0.499 |

---

## 4. Methodological Notes

- **Split Integrity:** Evaluated strictly on the test split, never used for training or checkpoint selection.
- **Seabed Decals:** In accordance with Honesty Rule H4, only images from this test split (`ml/data/test_manifest.txt`) will be used by `make_seabed.py`.
- **Benchmark Distinction:** Laptop inference speed (1.7 ms) measured on NVIDIA GeForce RTX 5060 Laptop GPU. Jetson Orin Nano edge benchmarks will be recorded in T5.2.
