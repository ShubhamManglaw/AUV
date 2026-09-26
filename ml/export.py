#!/usr/bin/env python3
"""Export PS11 trained YOLO11n model to ONNX format (T2.3).

Exports ml/weights/best.pt to ml/results/best.onnx with:
- opset 17
- static shape 640x640
- simplified via onnxslim
- validates with onnx.checker and prints input/output shapes.
"""

import shutil
import sys
from pathlib import Path

import onnx
from ultralytics import YOLO

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
ML_DIR = WORKSPACE_ROOT / "ml"
WEIGHTS_FILE = ML_DIR / "weights" / "best.pt"
RESULTS_DIR = ML_DIR / "results"
OUTPUT_ONNX = RESULTS_DIR / "best.onnx"


def main():
    if not WEIGHTS_FILE.exists():
        print(f"ERROR: Model weights not found at {WEIGHTS_FILE}", file=sys.stderr)
        sys.exit(1)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Loading PyTorch model from {WEIGHTS_FILE}...")
    model = YOLO(str(WEIGHTS_FILE))

    print("Exporting model to ONNX (opset=17, imgsz=640, static, simplify=True)...")
    exported_path = model.export(
        format="onnx",
        imgsz=640,
        opset=17,
        dynamic=False,
        simplify=True,
    )

    exported_file = Path(exported_path)
    print(f"Exported to {exported_file}")

    # Ensure destination in ml/results/best.onnx
    if exported_file.resolve() != OUTPUT_ONNX.resolve():
        shutil.copyfile(exported_file, OUTPUT_ONNX)
        print(f"Copied to {OUTPUT_ONNX}")

    # Validate with onnx.checker
    print("\nValidating ONNX model with onnx.checker...")
    onnx_model = onnx.load(str(OUTPUT_ONNX))
    onnx.checker.check_model(onnx_model)
    print("ONNX model verification PASSED!")

    # Print input and output shapes
    print("\n=== ONNX Model Signatures ===")
    print("Inputs:")
    for inp in onnx_model.graph.input:
        shape = [dim.dim_value for dim in inp.type.tensor_type.shape.dim]
        print(
            f"  Name: {inp.name}, Shape: {shape}, Type: {inp.type.tensor_type.elem_type}"
        )

    print("Outputs:")
    for out in onnx_model.graph.output:
        shape = [dim.dim_value for dim in out.type.tensor_type.shape.dim]
        print(
            f"  Name: {out.name}, Shape: {shape}, Type: {out.type.tensor_type.elem_type}"
        )

    print(f"\nModel size: {OUTPUT_ONNX.stat().st_size / (1024 * 1024):.2f} MB")
    print(f"ONNX export completed successfully -> {OUTPUT_ONNX}")


if __name__ == "__main__":
    main()
