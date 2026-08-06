# Datasets Documentation (T2.1)

## 1. Overview & Sources

PS11 perception pipeline combines two underwater object detection datasets for M1 pitch demo training:

| Dataset | Official Source / Repository | Paper / Citation |
|---|---|---|
| **TrashCan 1.0** (Instance Version) | [GitHub: mwebster7/trashcan](https://github.com/mwebster7/trashcan) | Hong, J., Fulton, M., & Sattar, J. (2020). *TrashCan: A dataset for underwater trash detection and segmentation*. arXiv:2007.08097. |
| **DUO** | [GitHub: chongweiliu/DUO](https://github.com/chongweiliu/DUO) | Liu, C., Wang, H., Liu, W., et al. (2021). *DUO: A Diverse Underwater Object Detection Dataset for Robot Picking*. IEEE Robotics and Automation Letters (RA-L). |

Raw data location:
- `ml/data/raw/trashcan/dataset/instance_version/`
- `ml/data/raw/duo/DUO/`

Processed YOLO dataset location:
- `ml/data/ps11/` (`images/{train,val,test}`, `labels/{train,val,test}`)
- `ml/data/ps11.yaml`
- `ml/data/test_manifest.txt` (only test images, for honest seabed decal generation per H4)

---

## 2. Licences & Attributions

### TrashCan 1.0 Licence & JAMSTEC Attribution
Full licence text from `ml/data/raw/trashcan/LICENSE.txt`:
```text
Use of the data in this repository is free for academic teaching and research purposes. Attribution must be given to both the authors of this dataset and the original providers of the data, JAMSTEC.  Attribution shall take the form of a reference in any associated academic papers, presentations, reports, or documents, as well as similar references on any materials made available via the internet (emails, websites, derivative datasets, etc.)

For commercial use of this dataset, you must obtain permission from JAMSTEC for the commercial use of their images. Please follow the instructions at http://www.godac.jamstec.go.jp/darwin/explain/1/e#condition to obtain permission for commercial use". You must still provide attribution to the authors of this dataset after obtaining commerical use permissions for JAMSTEC.
```
*Attribution:* Derived from video collected by the Japan Agency for Marine-Earth Science and Technology (JAMSTEC) J-EDI (JAMSTEC E-Library of Deep-sea Images) and annotated by the MARS Lab (University of Minnesota).

### DUO Licence
**No licence stated** in the official repository or publication. Used strictly for evaluation and demonstration purposes.

---

## 3. Raw Categories Found

### TrashCan 1.0 (16 categories)
- Biological / non-debris: `rov` (1), `plant` (2), `animal_fish` (3), `animal_starfish` (4), `animal_shells` (5), `animal_crab` (6), `animal_eel` (7), `animal_etc` (8)
- Trash / debris: `trash_clothing` (9), `trash_pipe` (10), `trash_bottle` (11), `trash_bag` (12), `trash_snack_wrapper` (13), `trash_can` (14), `trash_cup` (15), `trash_container` (16), `trash_unknown_instance` (17), `trash_branch` (18), `trash_wreckage` (19), `trash_tarp` (20), `trash_rope` (21), `trash_net` (22)

### DUO (4 categories)
- `holothurian` (1), `echinus` (2), `scallop` (3), `starfish` (4)

---

## 4. Class Mapping (§10.2)

Target classes defined in `ps11_bringup/config/classes.yaml`:
- **0: `debris`** (priority 1.0, defense proxy)
- **1: `starfish`** (priority 0.3)
- **2: `sea_urchin`** (priority 0.3)
- **3: `scallop`** (priority 0.2)

### Mapping Rules
| Source Dataset | Raw Category | Target Class ID | Target Class Name |
|---|---|---|---|
| TrashCan | `trash_*` (all 14 trash classes) | 0 | `debris` |
| TrashCan | `animal_starfish` | 1 | `starfish` |
| TrashCan | `rov`, `plant`, other `animal_*` | — | *DROPPED* |
| DUO | `starfish` | 1 | `starfish` |
| DUO | `echinus` | 2 | `sea_urchin` |
| DUO | `scallop` | 3 | `scallop` |
| DUO | `holothurian` | — | *DROPPED* |

---

## 5. Video-ID Leakage Analysis & Re-Splitting

### TrashCan Video Extraction Rule
Filenames follow the format `vid_XXXXXX_frameXXXXXXX.jpg`.
- **Extraction Rule:** Regex `r"^(vid_\d+)_frame\d+\.jpg$"`, capturing the prefix before `_frame`.
- Example: `vid_000159_frame0000008.jpg` -> Video ID `vid_000159`.

### Data Leakage in Default Splits
Inspection of the official TrashCan `instances_train_trashcan.json` and `instances_val_trashcan.json` revealed:
- Train unique videos: 310
- Val unique videos: 115
- **Shared videos:** 113 out of 115 validation videos are present in train (98.3% video leakage).

### Re-Splitting Methodology (Seed 42)
To eliminate frame correlation and data leakage across splits:
1. **TrashCan:** Combined all 7,212 images across train and val, grouped by unique video ID (312 unique videos total).
   - Videos shuffled with fixed random seed `42`.
   - 15% of videos (~46 videos) assigned to `test`.
   - Remaining videos partitioned 90% `train` (240 videos) and 10% `val` (26 videos).
2. **DUO:** Official `test` split (1,111 images) kept intact as `test`. Official `train` split (6,671 images) partitioned 90% `train` (6,004 images) and 10% `val` (667 images) with fixed seed `42`.
3. **Leakage Verification:** Asserted that no video ID appears in multiple TrashCan splits, and no image path appears in more than one split across the entire dataset.

---

## 6. Dataset Statistics

### Image Counts per Split
| Dataset Component | Train | Val | Test | Total |
|---|---|---|---|---|
| TrashCan (images) | 5,315 | 779 | 1,118 | 7,212 |
| DUO (images) | 6,004 | 667 | 1,111 | 7,782 |
| **Total Images** | **11,319** | **1,446** | **2,229** | **14,994** |

### Object Instance Counts per Class
| Split | Images | Debris (0) | Starfish (1) | Sea Urchin (2) | Scallop (3) | Total Bboxes |
|---|---|---|---|---|---|---|
| **train** | 11,319 | 4,564 | 11,553 | 38,695 | 1,527 | 56,339 |
| **val** | 1,446 | 610 | 1,271 | 4,260 | 180 | 6,321 |
| **test** | 2,229 | 832 | 2,122 | 7,201 | 217 | 10,372 |
| **Total** | **14,994** | **6,006** | **14,946** | **50,156** | **1,924** | **73,032** |

### Acceptance Criteria Verification
- **Every class has >= 200 training instances:**
  - Debris: 4,564 instances (`>= 200` **PASS**)
  - Starfish: 11,553 instances (`>= 200` **PASS**)
  - Sea Urchin: 38,695 instances (`>= 200` **PASS**)
  - Scallop: 1,527 instances (`>= 200` **PASS**)
- **Test Manifest Generated:** `ml/data/test_manifest.txt` contains 2,229 verified test image paths for seabed decal generation (H4 compliance).
