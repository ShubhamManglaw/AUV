# PS11-AUV — Jetson Benchmark Guide (Task T5.2)

> **Status:** Untested on device (instructions prepared for the hardware team).

This guide provides step-by-step instructions for running the YOLO11n TensorRT FP16 benchmark on the NVIDIA Jetson Orin Nano / Xavier NX.

---

## 1. Prerequisites on the Jetson
Ensure the Jetson has JetPack installed with TensorRT:
```bash
# Verify trtexec is available
which trtexec || ls /usr/src/tensorrt/bin/trtexec

# If not in PATH, add it to ~/.bashrc:
export PATH="/usr/src/tensorrt/bin:$PATH"
```

Ensure `tegrastats` and power management tools are present:
```bash
which tegrastats nvpmodel jetson_clocks
```

---

## 2. Setup Device Power Mode
Lock clocks and select maximum performance mode for reproducible benchmarking:
```bash
# Set 15W / MAXN power mode (e.g. Mode 0)
sudo nvpmodel -m 0

# Lock CPU and GPU clocks to maximum
sudo jetson_clocks
```

---

## 3. Run the Benchmark
Execute the automated benchmark script from the repository root:
```bash
cd ~/ps11-auv
chmod +x jetson/benchmark.sh jetson/parse_results.py

# Run benchmark (targets ml/weights/best.onnx or best_v2.onnx by default)
./jetson/benchmark.sh
```

### What `benchmark.sh` does automatically:
1. Records device state (`nvpmodel -q`, `jetson_clocks --show`, L4T version).
2. Builds an optimized TensorRT FP16 engine on the Jetson using `trtexec`.
3. Runs an inference throughput & latency benchmark for **60 seconds** with warmup.
4. Concurrently logs power, GPU utilization, and thermals using `tegrastats` sampled at **500 ms**.
5. Automatically parses logs into `jetson/results/run_<timestamp>/benchmark.md` and copies it to `jetson/results/benchmark.md`.

---

## 4. Video Recording for the Pitch (Task T5.3)
During the benchmark run, record a 30–60 second video showing:
1. Terminal 1: `trtexec` executing the 60-second test.
2. Terminal 2: `tegrastats` showing real-time power (mW) and GPU load.
3. Terminal 3: Output of `sudo nvpmodel -q` showing active power mode.

Save the video file to the shared drive as:
`jetson/results/jetson_benchmark.mp4` *(do NOT commit video files to Git per repository size limits)*.

---

## 5. Troubleshooting
- **Missing `trtexec`:**
  Check `/usr/src/tensorrt/bin/trtexec`. If found, symlink to `/usr/local/bin/trtexec`:
  ```bash
  sudo ln -s /usr/src/tensorrt/bin/trtexec /usr/local/bin/trtexec
  ```
- **Out of Memory during Engine Build:**
  Enable or increase swap space on the Jetson:
  ```bash
  sudo fallocate -l 4G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  ```
- **Permission Denied for tegrastats:**
  `tegrastats` may require root access on certain JetPack distributions:
  ```bash
  sudo tegrastats --interval 500
  ```
