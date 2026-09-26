#!/usr/bin/env python3
"""Evaluate best.pt on PS11 test split and dataset subsets (T2.3).

Evaluates:
1. Full test split (2,229 images) via ml/data/ps11.yaml
2. TrashCan test split only (1,118 images) via ml/data/ps11_test_trashcan.yaml
3. DUO test split only (1,111 images) via ml/data/ps11_test_duo.yaml

Generates ml/results/eval.md with per-class metrics and laptop inference speed.
"""

import sys
from collections import defaultdict
from pathlib import Path

from ultralytics import YOLO

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
ML_DIR = WORKSPACE_ROOT / "ml"
DATA_DIR = ML_DIR / "data"
RESULTS_DIR = ML_DIR / "results"
WEIGHTS_FILE = ML_DIR / "weights" / "best.pt"

CLASS_NAMES = {
    0: "debris",
    1: "starfish",
    2: "sea_urchin",
    3: "scallop",
}


def prepare_subset_files(test_img_dir: Path) -> tuple[Path, Path, Path, Path]:
    """Write test_trashcan.txt and test_duo.txt with ABSOLUTE image paths, and YAML configs."""
    all_imgs = sorted(test_img_dir.glob("*.jpg"))
    tc_imgs = [p for p in all_imgs if p.name.startswith("vid_")]
    duo_imgs = [p for p in all_imgs if not p.name.startswith("vid_")]

    print(
        f"Total test images: {len(all_imgs)} (TrashCan: {len(tc_imgs)}, DUO: {len(duo_imgs)})"
    )

    # Absolute paths in text files
    tc_txt = DATA_DIR / "test_trashcan.txt"
    duo_txt = DATA_DIR / "test_duo.txt"

    with open(tc_txt, "w", encoding="utf-8") as f:
        f.writelines(f"{p.absolute()}\n" for p in tc_imgs)

    with open(duo_txt, "w", encoding="utf-8") as f:
        f.writelines(f"{p.absolute()}\n" for p in duo_imgs)

    # Subset YAMLs
    tc_yaml = DATA_DIR / "ps11_test_trashcan.yaml"
    with open(tc_yaml, "w", encoding="utf-8") as f:
        f.write(f"""path: {DATA_DIR / "ps11"}
train: images/train
val: images/val
test: {tc_txt.absolute()}

names:
  0: debris
  1: starfish
  2: sea_urchin
  3: scallop
""")

    duo_yaml = DATA_DIR / "ps11_test_duo.yaml"
    with open(duo_yaml, "w", encoding="utf-8") as f:
        f.write(f"""path: {DATA_DIR / "ps11"}
train: images/train
val: images/val
test: {duo_txt.absolute()}

names:
  0: debris
  1: starfish
  2: sea_urchin
  3: scallop
""")

    return tc_txt, duo_txt, tc_yaml, duo_yaml


def count_instances_for_images(image_paths: list[Path]) -> dict[int, int]:
    """Count ground-truth instances per class from corresponding YOLO label files."""
    counts = defaultdict(int)
    labels_dir = DATA_DIR / "ps11" / "labels" / "test"
    for img_path in image_paths:
        stem = img_path.stem
        lbl_file = labels_dir / f"{stem}.txt"
        if lbl_file.exists():
            with open(lbl_file, "r", encoding="utf-8") as f:
                for line in f:
                    parts = line.strip().split()
                    if parts:
                        cls_id = int(parts[0])
                        counts[cls_id] += 1
    return counts


def run_evaluation(
    model: YOLO,
    yaml_path: Path,
    image_paths: list[Path],
    eval_name: str,
) -> tuple[dict, dict[int, dict]]:
    """Run model.val on a given dataset YAML and return summary and per-class metrics."""
    print(
        "\n================================================================================"
    )
    print(
        f"Running evaluation: {eval_name} on {yaml_path.name} ({len(image_paths)} images)"
    )
    print(
        "================================================================================"
    )
    results = model.val(
        data=str(yaml_path),
        split="test",
        imgsz=640,
        device=0,
        project=str(RESULTS_DIR),
        name=eval_name,
        exist_ok=True,
        verbose=True,
    )

    box = results.box
    speed = results.speed  # dict: preprocess, inference, loss, postprocess

    gt_counts = count_instances_for_images(image_paths)

    # Overall metrics
    overall = {
        "images": len(image_paths),
        "instances": sum(gt_counts.values()),
        "p": float(box.mp),
        "r": float(box.mr),
        "map50": float(box.map50),
        "map50_95": float(box.map),
        "speed": speed,
    }

    # Per-class metrics
    per_class = {}
    class_indices = (
        box.ap_class_index.tolist()
        if hasattr(box.ap_class_index, "tolist")
        else list(box.ap_class_index)
    )

    for i, cls_idx in enumerate(class_indices):
        p = float(box.p[i])
        r = float(box.r[i])
        ap50 = float(box.ap50[i])
        ap50_95 = float(box.ap[i])
        per_class[cls_idx] = {
            "name": CLASS_NAMES.get(cls_idx, f"class_{cls_idx}"),
            "instances": gt_counts[cls_idx],
            "p": p,
            "r": r,
            "map50": ap50,
            "map50_95": ap50_95,
        }

    # For classes not present in ground truth of this subset
    for cls_idx in range(4):
        if cls_idx not in per_class:
            per_class[cls_idx] = {
                "name": CLASS_NAMES[cls_idx],
                "instances": gt_counts[cls_idx],
                "p": 0.0,
                "r": 0.0,
                "map50": 0.0,
                "map50_95": 0.0,
            }

    return overall, per_class


def main():
    if not WEIGHTS_FILE.exists():
        print(f"ERROR: Model weights not found at {WEIGHTS_FILE}", file=sys.stderr)
        sys.exit(1)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    test_img_dir = DATA_DIR / "ps11" / "images" / "test"
    all_test_imgs = sorted(test_img_dir.glob("*.jpg"))
    trashcan_test_imgs = [p for p in all_test_imgs if p.name.startswith("vid_")]
    duo_test_imgs = [p for p in all_test_imgs if not p.name.startswith("vid_")]

    _, _, tc_yaml, duo_yaml = prepare_subset_files(test_img_dir)

    model = YOLO(str(WEIGHTS_FILE))

    # 1. Full test evaluation
    overall_all, per_class_all = run_evaluation(
        model,
        DATA_DIR / "ps11.yaml",
        all_test_imgs,
        "eval_test_all",
    )

    # 2. TrashCan test evaluation
    overall_tc, per_class_tc = run_evaluation(
        model,
        tc_yaml,
        trashcan_test_imgs,
        "eval_test_trashcan",
    )

    # 3. DUO test evaluation
    overall_duo, per_class_duo = run_evaluation(
        model,
        duo_yaml,
        duo_test_imgs,
        "eval_test_duo",
    )

    # Print markdown tables to terminal verbatim
    print("\n" + "=" * 80)
    print("PS11 EVALUATION SUMMARY TABLES")
    print("=" * 80)

    def print_markdown_table(title: str, overall: dict, per_class: dict):
        print(f"\n### {title}")
        print("| Class | Instances | Precision | Recall | mAP50 | mAP50-95 |")
        print("|---|---|---|---|---|---|")
        print(
            f"| **all** | {overall['instances']} | {overall['p']:.3f} | {overall['r']:.3f} | {overall['map50']:.3f} | {overall['map50_95']:.3f} |"
        )
        for cls_id in range(4):
            c = per_class[cls_id]
            if c["instances"] > 0:
                print(
                    f"| {c['name']} ({cls_id}) | {c['instances']} | {c['p']:.3f} | {c['r']:.3f} | {c['map50']:.3f} | {c['map50_95']:.3f} |"
                )
            else:
                print(f"| {c['name']} ({cls_id}) | 0 | — | — | — | — |")

    print_markdown_table("Full Test Split (Combined)", overall_all, per_class_all)
    print_markdown_table(
        "TrashCan Test Split Only (Real-World Debris Benchmark)",
        overall_tc,
        per_class_tc,
    )
    print_markdown_table(
        "DUO Test Split Only (Marine Life Benchmark)", overall_duo, per_class_duo
    )

    speed_info = overall_all["speed"]
    inf_ms = speed_info.get("inference", 0.0)
    prep_ms = speed_info.get("preprocess", 0.0)
    post_ms = speed_info.get("postprocess", 0.0)
    total_ms = prep_ms + inf_ms + post_ms
    fps = 1000.0 / total_ms if total_ms > 0 else 0.0

    print(
        f"\nInference speed: preprocess={prep_ms:.1f}ms, inference={inf_ms:.1f}ms, postprocess={post_ms:.1f}ms (Total: {total_ms:.1f}ms / {fps:.1f} FPS)"
    )
    print("Hardware label: RTX 5060 Laptop GPU, not Jetson")

    # Generate ml/results/eval.md
    eval_md_path = RESULTS_DIR / "eval.md"
    with open(eval_md_path, "w", encoding="utf-8") as f:
        f.write(f"""# Model Evaluation Report (T2.3)

**Model:** `ml/weights/best.pt` (YOLO11n, 2.58M parameters)  
**Input Size:** 640×640  
**Test Set:** `ml/data/test_manifest.txt` ({len(all_test_imgs)} images, test split, never used for training or checkpoint selection)  
**Inference Speed (Laptop):** {inf_ms:.1f} ms inference ({fps:.1f} FPS total pipeline) — *RTX 5060 Laptop GPU, not Jetson*  

---

## 1. Full Test Split Evaluation (Combined)

Evaluated on all 2,229 test images (1,118 TrashCan + 1,111 DUO).

| Class | Test Instances | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| **all** | **{overall_all["instances"]}** | **{overall_all["p"]:.3f}** | **{overall_all["r"]:.3f}** | **{overall_all["map50"]:.3f}** | **{overall_all["map50_95"]:.3f}** |
| debris (0) | {per_class_all[0]["instances"]} | {per_class_all[0]["p"]:.3f} | {per_class_all[0]["r"]:.3f} | {per_class_all[0]["map50"]:.3f} | {per_class_all[0]["map50_95"]:.3f} |
| starfish (1) | {per_class_all[1]["instances"]} | {per_class_all[1]["p"]:.3f} | {per_class_all[1]["r"]:.3f} | {per_class_all[1]["map50"]:.3f} | {per_class_all[1]["map50_95"]:.3f} |
| sea_urchin (2) | {per_class_all[2]["instances"]} | {per_class_all[2]["p"]:.3f} | {per_class_all[2]["r"]:.3f} | {per_class_all[2]["map50"]:.3f} | {per_class_all[2]["map50_95"]:.3f} |
| scallop (3) | {per_class_all[3]["instances"]} | {per_class_all[3]["p"]:.3f} | {per_class_all[3]["r"]:.3f} | {per_class_all[3]["map50"]:.3f} | {per_class_all[3]["map50_95"]:.3f} |

---

## 2. TrashCan Test Split Only (Real-World Debris Benchmark)

Evaluated on 1,118 TrashCan test images from 46 unseen videos. Debris appears exclusively in TrashCan.

| Class | Test Instances | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| **all** | **{overall_tc["instances"]}** | **{overall_tc["p"]:.3f}** | **{overall_tc["r"]:.3f}** | **{overall_tc["map50"]:.3f}** | **{overall_tc["map50_95"]:.3f}** |
| debris (0) | {per_class_tc[0]["instances"]} | {per_class_tc[0]["p"]:.3f} | {per_class_tc[0]["r"]:.3f} | {per_class_tc[0]["map50"]:.3f} | {per_class_tc[0]["map50_95"]:.3f} |
| starfish (1) | {per_class_tc[1]["instances"]} | {per_class_tc[1]["p"]:.3f} | {per_class_tc[1]["r"]:.3f} | {per_class_tc[1]["map50"]:.3f} | {per_class_tc[1]["map50_95"]:.3f} |
| sea_urchin (2) | 0 | — | — | — | — |
| scallop (3) | 0 | — | — | — | — |

---

## 3. DUO Test Split Only (Marine Life Benchmark)

Evaluated on 1,111 DUO test images (official DUO test partition).

| Class | Test Instances | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| **all** | **{overall_duo["instances"]}** | **{overall_duo["p"]:.3f}** | **{overall_duo["r"]:.3f}** | **{overall_duo["map50"]:.3f}** | **{overall_duo["map50_95"]:.3f}** |
| debris (0) | 0 | — | — | — | — |
| starfish (1) | {per_class_duo[1]["instances"]} | {per_class_duo[1]["p"]:.3f} | {per_class_duo[1]["r"]:.3f} | {per_class_duo[1]["map50"]:.3f} | {per_class_duo[1]["map50_95"]:.3f} |
| sea_urchin (2) | {per_class_duo[2]["instances"]} | {per_class_duo[2]["p"]:.3f} | {per_class_duo[2]["r"]:.3f} | {per_class_duo[2]["map50"]:.3f} | {per_class_duo[2]["map50_95"]:.3f} |
| scallop (3) | {per_class_duo[3]["instances"]} | {per_class_duo[3]["p"]:.3f} | {per_class_duo[3]["r"]:.3f} | {per_class_duo[3]["map50"]:.3f} | {per_class_duo[3]["map50_95"]:.3f} |

---

## 4. Methodological Notes

- **Split Integrity:** Evaluated strictly on the test split, never used for training or checkpoint selection.
- **Seabed Decals:** In accordance with Honesty Rule H4, only images from this test split (`ml/data/test_manifest.txt`) will be used by `make_seabed.py`.
- **Benchmark Distinction:** Laptop inference speed ({inf_ms:.1f} ms) measured on NVIDIA GeForce RTX 5060 Laptop GPU. Jetson Orin Nano edge benchmarks will be recorded in T5.2.
""")

    print(f"\nWritten evaluation report to {eval_md_path}")


if __name__ == "__main__":
    main()
