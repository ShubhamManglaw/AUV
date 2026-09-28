#!/usr/bin/env bash
# PS11-AUV — Jetson TensorRT FP16 Benchmark Kit (§5.2, §15.2, T5.2)
#
# NOTE: Untested on device. Designed for teammates to run on NVIDIA Jetson.
# Builds FP16 TensorRT engine with trtexec, benchmarks for 60 s,
# logs power and thermals with tegrastats at 500 ms, and parses results.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
cd "$REPO_DIR"

# Locate trtexec (commonly in /usr/src/tensorrt/bin or standard PATH)
TRTEXEC_BIN="trtexec"
if ! command -v trtexec &>/dev/null; then
    if [ -x "/usr/src/tensorrt/bin/trtexec" ]; then
        TRTEXEC_BIN="/usr/src/tensorrt/bin/trtexec"
    else
        echo "ERROR: 'trtexec' not found in PATH or /usr/src/tensorrt/bin/trtexec."
        echo "Please install TensorRT or add it to PATH."
        exit 1
    fi
fi

# Locate ONNX model
ONNX_PATH="${1:-ml/weights/best.onnx}"
if [ ! -f "$ONNX_PATH" ]; then
    if [ -f "ml/weights/best_v2.onnx" ]; then
        ONNX_PATH="ml/weights/best_v2.onnx"
    elif [ -f "ml/weights/best_v2.pt" ]; then
        echo "ONNX model not found, exporting ml/weights/best_v2.pt to ONNX..."
        python3 -c "from ultralytics import YOLO; YOLO('ml/weights/best_v2.pt').export(format='onnx', imgsz=640, half=False)"
        ONNX_PATH="ml/weights/best_v2.onnx"
    else
        echo "ERROR: Cannot find ONNX model at '$ONNX_PATH'."
        exit 1
    fi
fi

# Output directory
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$REPO_DIR/jetson/results/run_${TIMESTAMP}"
mkdir -p "$OUT_DIR"
ENGINE_PATH="$OUT_DIR/model_fp16.engine"

echo "================================================================================"
echo "PS11-AUV Jetson Benchmark Run (T5.2)"
echo "Target ONNX: $ONNX_PATH"
echo "Output Directory: $OUT_DIR"
echo "trtexec: $TRTEXEC_BIN"
echo "================================================================================"

# 1. Record device metadata and power state
echo "[1/4] Recording Jetson device and clock state..."
if command -v nvpmodel &>/dev/null; then
    sudo nvpmodel -q > "$OUT_DIR/nvpmodel.txt" 2>&1 || nvpmodel -q > "$OUT_DIR/nvpmodel.txt" 2>&1 || true
else
    echo "nvpmodel not found" > "$OUT_DIR/nvpmodel.txt"
fi

if command -v jetson_clocks &>/dev/null; then
    sudo jetson_clocks --show > "$OUT_DIR/jetson_clocks.txt" 2>&1 || jetson_clocks --show > "$OUT_DIR/jetson_clocks.txt" 2>&1 || true
else
    echo "jetson_clocks not found" > "$OUT_DIR/jetson_clocks.txt"
fi

if [ -f "/etc/nv_tegra_release" ]; then
    cat /etc/nv_tegra_release > "$OUT_DIR/l4t_release.txt"
fi
uname -a > "$OUT_DIR/system_info.txt"

# 2. Build TensorRT FP16 engine on the Jetson
echo "[2/4] Building TensorRT FP16 engine on the Jetson (this may take 2-5 min)..."
"$TRTEXEC_BIN" \
    --onnx="$ONNX_PATH" \
    --saveEngine="$ENGINE_PATH" \
    --fp16 \
    --builderOptimizationLevel=3 \
    > "$OUT_DIR/trtexec_build.log" 2>&1

echo "Engine built successfully: $ENGINE_PATH"

# 3. 60-second benchmark with tegrastats
echo "[3/4] Running 60-second benchmark with tegrastats (500 ms sampling)..."
TEGRA_LOG="$OUT_DIR/tegrastats.log"

if command -v tegrastats &>/dev/null; then
    tegrastats --interval 500 --logfile "$TEGRA_LOG" &
    TEGRA_PID=$!
else
    echo "WARNING: tegrastats not found. Power/thermal logging skipped."
    TEGRA_PID=""
fi

# Run inference benchmark for 60 seconds
"$TRTEXEC_BIN" \
    --loadEngine="$ENGINE_PATH" \
    --duration=60 \
    --warmUp=2000 \
    --useCudaGraph \
    > "$OUT_DIR/trtexec_run.log" 2>&1 || true

# Stop tegrastats
if [ -n "$TEGRA_PID" ]; then
    kill "$TEGRA_PID" 2>/dev/null || true
    wait "$TEGRA_PID" 2>/dev/null || true
fi

# 4. Parse results and generate benchmark.md
echo "[4/4] Parsing benchmark results..."
python3 "$SCRIPT_DIR/parse_results.py" "$OUT_DIR"

# Link or copy latest summary
cp "$OUT_DIR/benchmark.md" "$REPO_DIR/jetson/results/benchmark.md" 2>/dev/null || true

echo "================================================================================"
echo "Benchmark Complete! Results saved to:"
echo "  $OUT_DIR/benchmark.md"
echo "  $REPO_DIR/jetson/results/benchmark.md"
echo "================================================================================"
