#!/usr/bin/env python3
"""Report T2.9 Fine-Tuning Results and Verification (§10.3).

Executes:
(a) Real test-set per-class table: ps11_v1 vs ps11_v2 (overall mAP50 drop <= 0.02 check).
(b) Sim validation per-class table (seed 110).
(c) 12-object demo detectability table with ps11_v2 weights.
(d) If (a) passes: copies best.pt to ml/weights/best_v2.pt and updates perception.yaml.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml
from ultralytics import YOLO

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
ML_DIR = WORKSPACE_ROOT / "ml"


def find_v2_weights() -> Path:
    candidates = [
        WORKSPACE_ROOT / "runs" / "detect" / "ml" / "runs" / "ps11_v2" / "weights" / "best.pt",
        ML_DIR / "runs" / "ps11_v2" / "weights" / "best.pt",
    ]
    for p in candidates:
        if p.exists():
            return p
    return candidates[0]


PERCEPTION_YAML = (
    WORKSPACE_ROOT / "ros2_ws" / "src" / "ps11_bringup" / "config" / "perception.yaml"
)

# Golden baseline metrics from ps11_v1 (T2.3)
V1_METRICS = {
    "all": {"map50": 0.703, "map50_95": 0.533},
    "debris": {"map50": 0.351, "map50_95": 0.189},
    "starfish": {"map50": 0.886, "map50_95": 0.708},
    "sea_urchin": {"map50": 0.926, "map50_95": 0.738},
    "scallop": {"map50": 0.649, "map50_95": 0.498},
}

CLASS_NAMES = ["debris", "starfish", "sea_urchin", "scallop"]


def evaluate_model(
    weights_path: Path, data_yaml: Path, split: str = "test"
) -> dict[str, Any]:
    """Run Ultralytics val and extract per-class metrics."""
    model = YOLO(str(weights_path))
    results = model.val(
        data=str(data_yaml),
        split=split,
        imgsz=640,
        batch=16,
        device=0,
        verbose=False,
    )
    return results


def main() -> None:
    v2_weights = find_v2_weights()
    if not v2_weights.exists():
        print(f"Error: Fine-tuned weights not found at {v2_weights}!")
        print("Ensure the training run has finished in ml/runs/ps11_v2.")
        sys.exit(1)

    print("=" * 80)
    print("T2.9 FINE-TUNING POST-TRAINING REPORT (ps11_v2)")
    print(f"Weights source: {v2_weights}")
    print("=" * 80)

    # -------------------------------------------------------------------------
    # (a) Real test-set evaluation: ps11_v1 vs ps11_v2
    # -------------------------------------------------------------------------
    print("\n--- (a) Real Test-Set Evaluation (ml/data/ps11.yaml, 2,229 images) ---")
    real_test_yaml = ML_DIR / "data" / "ps11.yaml"
    val_real = evaluate_model(v2_weights, real_test_yaml, split="test")

    overall_map50_v2 = float(val_real.box.map50)
    overall_map50_95_v2 = float(val_real.box.map)
    overall_map50_v1 = V1_METRICS["all"]["map50"]
    map50_diff_overall = overall_map50_v2 - overall_map50_v1

    print(
        "\n| Class | ps11_v1 mAP50 | ps11_v2 mAP50 | Δ mAP50 | ps11_v1 mAP50-95 | ps11_v2 mAP50-95 | Δ mAP50-95 |"
    )
    print("|---|---|---|---|---|---|---|")
    print(
        f"| **overall** | **{overall_map50_v1:.3f}** | **{overall_map50_v2:.3f}** | **{map50_diff_overall:+.3f}** | "
        f"**{V1_METRICS['all']['map50_95']:.3f}** | **{overall_map50_95_v2:.3f}** | **{overall_map50_95_v2 - V1_METRICS['all']['map50_95']:+.3f}** |"
    )

    for idx, cname in enumerate(CLASS_NAMES):
        m50_v1 = V1_METRICS[cname]["map50"]
        m_v1 = V1_METRICS[cname]["map50_95"]
        try:
            _, _, m50_v2, m_v2 = val_real.box.class_result(idx)
            m50_v2 = float(m50_v2)
            m_v2 = float(m_v2)
        except Exception:
            m50_v2 = float(val_real.box.ap50[idx]) if hasattr(val_real.box, "ap50") and idx < len(val_real.box.ap50) else 0.0
            m_v2 = float(val_real.box.ap[idx]) if hasattr(val_real.box, "ap") and idx < len(val_real.box.ap) else 0.0
        print(
            f"| {cname:<11} | {m50_v1:.3f} | {m50_v2:.3f} | {m50_v2 - m50_v1:+.3f} | "
            f"{m_v1:.3f} | {m_v2:.3f} | {m_v2 - m_v1:+.3f} |"
        )

    real_retention_pass = map50_diff_overall >= -0.02
    print(
        f"\nCriterion (a): Overall mAP50 drop >= -0.02: Δ = {map50_diff_overall:+.4f}"
    )
    if real_retention_pass:
        print(">>> CRITERION (a) PASSED: Real test-set mAP50 preserved! <<<")
    else:
        print(
            f">>> CRITERION (a) FAILED: Overall mAP50 dropped by {abs(map50_diff_overall):.4f} > 0.02 <<<"
        )

    # -------------------------------------------------------------------------
    # (b) Sim validation evaluation (seed 110)
    # -------------------------------------------------------------------------
    print(
        "\n--- (b) Simulation Validation Evaluation (ml/data/sim_val.yaml, seed 110) ---"
    )
    sim_val_yaml = ML_DIR / "data" / "sim_val.yaml"
    if sim_val_yaml.exists():
        val_sim = evaluate_model(v2_weights, sim_val_yaml, split="val")
        sim_map50 = float(val_sim.box.map50)
        sim_map = float(val_sim.box.map)
        print("\n| Class | Precision | Recall | mAP50 | mAP50-95 |")
        print("|---|---|---|---|---|")
        print(
            f"| **all** | {float(val_sim.box.mp):.3f} | {float(val_sim.box.mr):.3f} | **{sim_map50:.3f}** | **{sim_map:.3f}** |"
        )
        for idx, cname in enumerate(CLASS_NAMES):
            try:
                p_c, r_c, m50_c, m_c = val_sim.box.class_result(idx)
                p_c, r_c, m50_c, m_c = float(p_c), float(r_c), float(m50_c), float(m_c)
            except Exception:
                p_c = float(val_sim.box.p[idx]) if hasattr(val_sim.box, "p") and idx < len(val_sim.box.p) else 0.0
                r_c = float(val_sim.box.r[idx]) if hasattr(val_sim.box, "r") and idx < len(val_sim.box.r) else 0.0
                m50_c = float(val_sim.box.ap50[idx]) if hasattr(val_sim.box, "ap50") and idx < len(val_sim.box.ap50) else 0.0
                m_c = float(val_sim.box.ap[idx]) if hasattr(val_sim.box, "ap") and idx < len(val_sim.box.ap) else 0.0
            print(
                f"| {cname:<11} | {p_c:>9.3f} | {r_c:>6.3f} | {m50_c:>5.3f} | {m_c:>8.3f} |"
            )
    else:
        print("Warning: ml/data/sim_val.yaml not found, skipping sim val metrics.")

    # -------------------------------------------------------------------------
    # (c) 12-object demo detectability table
    # -------------------------------------------------------------------------
    print("\n--- (c) 12-Object Demo Detectability Check with ps11_v2 ---")
    detect_res = subprocess.run(
        [
            sys.executable,
            "tools/test_detectability.py",
            "--weights",
            str(v2_weights),
        ],
        cwd=str(WORKSPACE_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )
    print(detect_res.stdout)
    if detect_res.returncode != 0:
        print(detect_res.stderr)

    # -------------------------------------------------------------------------
    # (d) If (a) passes: copy to best_v2.pt and update perception.yaml
    # -------------------------------------------------------------------------
    if real_retention_pass:
        print("\n--- (d) Updating Deployment Weights ---")
        dest_weights = ML_DIR / "weights" / "best_v2.pt"
        shutil.copy2(v2_weights, dest_weights)
        print(f"Copied {v2_weights} -> {dest_weights}")

        import re

        content = PERCEPTION_YAML.read_text(encoding="utf-8")
        updated_content = re.sub(
            r'model_path:\s*["\']?[^"\n]+["\']?',
            'model_path: "ml/weights/best_v2.pt"',
            content,
        )
        PERCEPTION_YAML.write_text(updated_content, encoding="utf-8")

        print(f"Updated {PERCEPTION_YAML}: model_path set to ml/weights/best_v2.pt")
        print("\n>>> T2.9 CONTINGENCY COMPLETE & DEPLOYED SUCCESSFULLY! <<<")
    else:
        print("\n>>> CRITERION (a) FAILED: Keeping ps11_v1 in perception.yaml. <<<")


if __name__ == "__main__":
    main()
