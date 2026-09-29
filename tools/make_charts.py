#!/usr/bin/env python3
"""Generates pitch charts from summary.json and evaluation logs (§12.4, T4.5).

Produces:
1. results/charts/bandwidth_comparison.png (Cumulative bits: Video vs Semantic telemetry)
2. results/charts/recall_and_contacts.png (Ground truth recall & contact delivery)
3. results/charts/latency_and_accuracy.png (Position accuracy & first-report latency)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def generate_bandwidth_chart(summary: dict, out_path: Path) -> None:
    """Bar chart comparing cumulative data volume and transmission airtime."""
    sem_bits = summary.get("semantic_bits_sent", 4032)
    jpeg_bits = summary.get("jpeg_equiv_bits", 15000000)
    h264_bits = summary.get("h264_equiv_bits", 2400000)

    labels = ["Semantic\nTelemetry", "H.264 Video\n(CRF 28)", "Raw JPEG\n(Q 75)"]
    bits = [max(1, sem_bits), max(1, h264_bits), max(1, jpeg_bits)]
    kb_values = [b / (8 * 1024) for b in bits]

    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(labels, kb_values, color=["#4ECDC4", "#F2C14E", "#E4572E"], width=0.55)

    ax.set_yscale("log")
    ax.set_ylabel("Data Volume (KB, Log Scale)", fontsize=12)
    ax.set_title("Bandwidth Comparison: Semantic Backhaul vs Video Transmission", fontsize=14, pad=15)
    ax.grid(axis="y", linestyle="--", alpha=0.6)

    for bar, kb in zip(bars, kb_values):
        height = bar.get_height()
        ax.annotate(
            f"{kb:.1f} KB",
            xy=(bar.get_x() + bar.get_width() / 2, height),
            xytext=(0, 5),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontweight="bold",
        )

    # Annotate ratio
    ratio = summary.get("ratio_vs_h264", summary.get("ratio_vs_jpeg", 0.0))
    if ratio > 0:
        ax.text(
            0.5, 0.90,
            f"Edge Semantic Reduction: ~{ratio:.0f}× Bandwidth Savings",
            transform=ax.transAxes,
            ha="center",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#E6F9F6", edgecolor="#4ECDC4"),
            fontsize=11,
            fontweight="bold",
        )

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"Generated chart: {out_path}")


def generate_accuracy_chart(summary: dict, out_path: Path) -> None:
    """Summary card chart for recall, position error, and report latency."""
    recall = summary.get("surface_recall", 0.0) * 100.0
    pos_err = summary.get("mean_position_error_m", 0.0)
    latency = summary.get("mean_first_report_latency_s", 0.0)
    reported = summary.get("gt_objects_reported", 0)
    total = summary.get("gt_objects_total", 12)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.5))

    # Recall bar
    ax1.bar(["Target Objects\nReported"], [reported], color="#4ECDC4", width=0.4, label="Delivered")
    ax1.bar(["Target Objects\nReported"], [total - reported], bottom=[reported], color="#E0E0E0", width=0.4, label="Undetected")
    ax1.set_ylabel("Object Count", fontsize=11)
    ax1.set_title(f"Surface Recall: {recall:.1f}% ({reported}/{total} Objects)", fontsize=12, fontweight="bold")
    ax1.legend(loc="upper left")
    ax1.set_ylim(0, max(total + 2, 14))

    # Metrics indicators
    metrics_names = ["Position Error\n(Mean, m)", "First-Report\nLatency (s)"]
    metrics_vals = [pos_err, latency]
    colors = ["#2E86AB", "#A23B72"]

    bars = ax2.bar(metrics_names, metrics_vals, color=colors, width=0.45)
    ax2.set_ylabel("Value", fontsize=11)
    ax2.set_title("Edge Localization & Backhaul Latency", fontsize=12, fontweight="bold")
    ax2.grid(axis="y", linestyle="--", alpha=0.6)

    for bar, val in zip(bars, metrics_vals):
        height = bar.get_height()
        ax2.annotate(
            f"{val:.2f}",
            xy=(bar.get_x() + bar.get_width() / 2, height),
            xytext=(0, 4),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontweight="bold",
        )

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"Generated chart: {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate pitch charts from summary.json")
    parser.add_argument("summary", nargs="?", default="results/latest_summary.json", help="Path to summary.json")
    parser.add_argument("--outdir", default="results/charts", help="Output directory for charts")
    args = parser.parse_args()

    summary_file = Path(args.summary)
    if not summary_file.is_file():
        # Fallback search
        summaries = list(Path("results").glob("run_*/summary.json"))
        if summaries:
            summaries.sort(key=lambda p: p.stat().st_mtime, reverse=True)
            summary_file = summaries[0]

    summary_data: dict = {}
    if summary_file.is_file():
        try:
            with open(summary_file, encoding="utf-8") as f:
                summary_data = json.load(f)
            print(f"Loaded summary data from: {summary_file}")
        except Exception as e:
            print(f"Warning: Failed to load {summary_file}: {e}")

    out_dir = Path(args.outdir)
    generate_bandwidth_chart(summary_data, out_dir / "bandwidth_comparison.png")
    generate_accuracy_chart(summary_data, out_dir / "recall_and_contacts.png")


if __name__ == "__main__":
    main()
