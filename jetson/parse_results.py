#!/usr/bin/env python3
"""Parses trtexec and tegrastats logs from Jetson benchmark (§15.2, T5.2).

Honesty Rules:
- Unknown tegrastats fields are printed raw, never guessed (H5).
- Clearly states: "Inference only (TensorRT), excludes pre- and post-processing".
- Untested on device.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


def parse_trtexec_log(log_path: Path) -> dict[str, str]:
    """Extract throughput, mean/p99 latency, and timings from trtexec log."""
    res: dict[str, str] = {}
    if not log_path.is_file():
        return res

    text = log_path.read_text(encoding="utf-8", errors="replace")

    # Throughput (qps)
    qps_match = re.search(r"Throughput:\s*([\d\.]+)\s*qps", text)
    if qps_match:
        res["throughput_qps"] = qps_match.group(1)

    # Latency summary
    # Look for "Latency: min = ... ms, max = ... ms, mean = ... ms, median = ... ms, percentile(99%) = ... ms"
    lat_match = re.search(
        r"Latency:\s*min\s*=\s*([\d\.]+)\s*ms,\s*max\s*=\s*([\d\.]+)\s*ms,\s*mean\s*=\s*([\d\.]+)\s*ms,\s*median\s*=\s*([\d\.]+)\s*ms,\s*percentile\(99%\)\s*=\s*([\d\.]+)\s*ms",
        text,
    )
    if lat_match:
        res["latency_min_ms"] = lat_match.group(1)
        res["latency_max_ms"] = lat_match.group(2)
        res["latency_mean_ms"] = lat_match.group(3)
        res["latency_median_ms"] = lat_match.group(4)
        res["latency_p99_ms"] = lat_match.group(5)
    else:
        # Fallback individual patterns
        mean_m = re.search(r"mean\s*=\s*([\d\.]+)\s*ms", text)
        if mean_m:
            res["latency_mean_ms"] = mean_m.group(1)
        p99_m = re.search(r"percentile\(99%\)\s*=\s*([\d\.]+)\s*ms", text)
        if p99_m:
            res["latency_p99_ms"] = p99_m.group(1)

    # GPU compute time
    compute_m = re.search(
        r"GPU Compute Time:\s*min\s*=\s*([\d\.]+)\s*ms,\s*max\s*=\s*([\d\.]+)\s*ms,\s*mean\s*=\s*([\d\.]+)\s*ms",
        text,
    )
    if compute_m:
        res["compute_mean_ms"] = compute_m.group(3)

    return res


def parse_tegrastats_log(log_path: Path) -> tuple[dict[str, str], list[str]]:
    """Extract RAM, GPU utilization, power, and collect unparsed/unknown tokens raw."""
    res: dict[str, str] = {}
    unknown_fields: list[str] = []

    if not log_path.is_file():
        return res, unknown_fields

    lines = [
        line.strip()
        for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines()
        if line.strip()
    ]
    if not lines:
        return res, unknown_fields

    ram_usages: list[float] = []
    gpu_utils: list[float] = []
    powers_mw: list[float] = []

    for line in lines:
        # Sample tegrastats line:
        # RAM 3421/7771MB (lfb 42x4MB) SWAP 0/3885MB (cached 0MB) CPU [15%@1190,12%@1190,10%@1190,8%@1190] EMC_FREQ 0%@204 GR3D_FREQ 99%@998 VDD_IN 12432mW/12432mW VDD_CPU_GPU_CV 6120mW/6120mW VDD_SOC 2100mW/2100mW ...
        ram_m = re.search(r"RAM\s+(\d+)/(\d+)MB", line)
        if ram_m:
            ram_usages.append(float(ram_m.group(1)))
            res["ram_total_mb"] = ram_m.group(2)

        gpu_m = re.search(r"GR3D_FREQ\s+(\d+)%", line)
        if gpu_m:
            gpu_utils.append(float(gpu_m.group(1)))

        # Power field: VDD_IN or POM_5V_IN or VDD_CPU_GPU_CV
        pwr_m = re.search(r"(?:VDD_IN|POM_5V_IN|VDD_CPU_GPU_CV)\s+(\d+)mW", line)
        if pwr_m:
            powers_mw.append(float(pwr_m.group(1)))

        # Check for unparsed hardware telemetry tokens to follow honesty rule H5
        tokens = line.split()
        for token in tokens:
            is_known = any(
                token.startswith(prefix)
                for prefix in (
                    "RAM",
                    "SWAP",
                    "CPU",
                    "EMC_FREQ",
                    "GR3D_FREQ",
                    "VDD_",
                    "POM_",
                    "thermal",
                    "PLL@",
                    "PMIC@",
                )
            )
            if not is_known and token not in unknown_fields and len(token) > 1:
                unknown_fields.append(token)

    if ram_usages:
        res["ram_mean_mb"] = f"{sum(ram_usages) / len(ram_usages):.1f}"
        res["ram_max_mb"] = f"{max(ram_usages):.1f}"

    if gpu_utils:
        res["gpu_util_mean_pct"] = f"{sum(gpu_utils) / len(gpu_utils):.1f}"

    if powers_mw:
        mean_pwr_w = (sum(powers_mw) / len(powers_mw)) / 1000.0
        max_pwr_w = max(powers_mw) / 1000.0
        res["power_mean_w"] = f"{mean_pwr_w:.2f}"
        res["power_max_w"] = f"{max_pwr_w:.2f}"

    return res, unknown_fields[:15]


def main() -> None:
    target_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("jetson/results")

    trtexec_log = target_dir / "trtexec_run.log"
    tegrastats_log = target_dir / "tegrastats.log"
    nvpmodel_file = target_dir / "nvpmodel.txt"
    clocks_file = target_dir / "jetson_clocks.txt"

    trt_data = parse_trtexec_log(trtexec_log)
    tegra_data, unknown_tokens = parse_tegrastats_log(tegrastats_log)

    nvp_text = (
        nvpmodel_file.read_text(encoding="utf-8", errors="replace").strip()
        if nvpmodel_file.is_file()
        else "NOT RECORDED"
    )
    clocks_text = (
        clocks_file.read_text(encoding="utf-8", errors="replace").strip()
        if clocks_file.is_file()
        else "NOT RECORDED"
    )

    md_lines: list[str] = [
        "# PS11-AUV — Jetson Benchmark Report (T5.2)",
        "",
        "> [!IMPORTANT]",
        "> **Benchmark Scope:** Inference only (TensorRT FP16 engine), excludes camera capture, image pre-processing, ByteTrack tracking, geolocation, and telemetry encoding.",
        "> **Device Status:** Untested on device (run by team on physical hardware).",
        "",
        "## Performance Summary",
        "",
        "| Metric | Value | Raw Source |",
        "|---|---|---|",
        f"| **Throughput** | {trt_data.get('throughput_qps', 'NOT MEASURED')} FPS | `trtexec_run.log` |",
        f"| **Mean Latency** | {trt_data.get('latency_mean_ms', 'NOT MEASURED')} ms | `trtexec_run.log` |",
        f"| **Median Latency** | {trt_data.get('latency_median_ms', 'NOT MEASURED')} ms | `trtexec_run.log` |",
        f"| **99th Percentile (p99) Latency** | {trt_data.get('latency_p99_ms', 'NOT MEASURED')} ms | `trtexec_run.log` |",
        f"| **GPU Compute Time (mean)** | {trt_data.get('compute_mean_ms', 'NOT MEASURED')} ms | `trtexec_run.log` |",
        f"| **Average Power** | {tegra_data.get('power_mean_w', 'NOT MEASURED')} W | `tegrastats.log` |",
        f"| **Peak Power** | {tegra_data.get('power_max_w', 'NOT MEASURED')} W | `tegrastats.log` |",
        f"| **GPU Utilisation (mean)** | {tegra_data.get('gpu_util_mean_pct', 'NOT MEASURED')} % | `tegrastats.log` |",
        f"| **RAM Usage (mean / max)** | {tegra_data.get('ram_mean_mb', 'NOT MEASURED')} MB / {tegra_data.get('ram_max_mb', 'NOT MEASURED')} MB | `tegrastats.log` |",
        "",
        "## Device Configuration",
        "",
        "```text",
        f"Power Model (nvpmodel):\n{nvp_text}",
        "",
        f"Jetson Clocks:\n{clocks_text}",
        "```",
        "",
    ]

    if unknown_tokens:
        md_lines.extend(
            [
                "## Raw Tegrastats Unparsed Tokens (Honesty Rule H5)",
                "The following tokens were detected in tegrastats and are printed raw without guessing:",
                "```text",
                ", ".join(unknown_tokens),
                "```",
                "",
            ]
        )

    out_md = target_dir / "benchmark.md"
    out_md.write_text("\n".join(md_lines), encoding="utf-8")
    print(f"Generated benchmark report: {out_md}")


if __name__ == "__main__":
    main()
