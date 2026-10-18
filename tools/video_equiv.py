#!/usr/bin/env python3
"""H.264 video compression comparison for PS11 AUV backhaul (§12.3, T4.5).

Extracts /vehicle/camera/image_raw/compressed (or /vehicle/camera/image_raw) from an MCAP bag,
re-encodes to H.264 using ffmpeg (libx264, CRF 28, preset medium, 10 fps),
measures total bits, computes ratio_vs_h264, and updates summary.json.

NOTE: H.264 is re-encoded from quality-95 JPEG frames recorded at 10 Hz.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from ps11_common.image_utils import compressed_image_to_numpy, image_to_numpy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import CompressedImage, Image


def extract_and_encode_h264(bag_path: Path, output_mp4: Path) -> tuple[int, int]:
    """Read camera frames from bag and pipe raw RGB frames to ffmpeg.

    Note: H.264 is re-encoded from quality-95 JPEG frames.
    """
    storage_options = StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    ffmpeg_proc: subprocess.Popen | None = None
    frame_count = 0
    width, height = 640, 480

    while reader.has_next():
        topic_name, serialized_msg, _ = reader.read_next()
        arr = None
        if topic_name in (
            "/vehicle/camera/image_raw/compressed",
            "/vehicle/camera/compressed",
        ):
            comp_msg = deserialize_message(serialized_msg, CompressedImage)
            arr = compressed_image_to_numpy(comp_msg, target_encoding="rgb8")
            height, width = arr.shape[:2]
        elif topic_name == "/vehicle/camera/image_raw":
            img_msg = deserialize_message(serialized_msg, Image)
            width, height = img_msg.width, img_msg.height
            arr = image_to_numpy(img_msg)

        if arr is not None:


            if ffmpeg_proc is None:
                output_mp4.parent.mkdir(parents=True, exist_ok=True)
                cmd = [
                    "ffmpeg", "-y",
                    "-f", "rawvideo",
                    "-vcodec", "rawvideo",
                    "-s", f"{width}x{height}",
                    "-pix_fmt", "rgb24",
                    "-r", "10",
                    "-i", "-",
                    "-c:v", "libx264",
                    "-crf", "28",
                    "-preset", "medium",
                    str(output_mp4),
                ]
                ffmpeg_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)

            assert ffmpeg_proc.stdin is not None
            ffmpeg_proc.stdin.write(arr.tobytes())
            frame_count += 1

    if ffmpeg_proc is not None:
        if ffmpeg_proc.stdin:
            ffmpeg_proc.stdin.close()
        ffmpeg_proc.wait()

    if output_mp4.is_file() and frame_count > 0:
        file_size_bytes = output_mp4.stat().st_size
        return file_size_bytes, frame_count
    return 0, 0


def main() -> None:
    parser = argparse.ArgumentParser(description="H.264 video compression comparison")
    parser.add_argument("bag", nargs="?", default="", help="Path to bag directory or file")
    parser.add_argument("--summary", default="", help="Path to summary.json to update")
    args = parser.parse_args()

    # Locate bag
    bag_path: Path | None = None
    if args.bag:
        bag_path = Path(args.bag)
    else:
        # Auto-detect latest bag in results/
        candidates = list(Path("results").glob("run_*_bag"))
        if candidates:
            candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
            bag_path = candidates[0]

    if not bag_path or not bag_path.exists():
        print(f"ERROR: Cannot find bag at '{bag_path}'")
        sys.exit(1)

    # Locate summary.json
    summary_path: Path | None = None
    if args.summary:
        summary_path = Path(args.summary)
    else:
        # Check inside bag dir, or latest_summary.json
        if (bag_path / "summary.json").is_file():
            summary_path = bag_path / "summary.json"
        elif Path("results/latest_summary.json").is_file():
            summary_path = Path("results/latest_summary.json")

    out_mp4 = bag_path.parent / f"{bag_path.name}_h264.mp4"
    print(f"Re-encoding /vehicle/camera/image_raw from {bag_path} to H.264...")
    h264_bytes, frame_count = extract_and_encode_h264(bag_path, out_mp4)

    h264_bits = h264_bytes * 8
    print(f"H.264 Encoding Complete: {frame_count} frames, {h264_bytes / 1024:.1f} KB ({h264_bits:,} bits)")

    # Read and update summary.json
    if summary_path and summary_path.is_file():
        try:
            with open(summary_path, encoding="utf-8") as f:
                data = json.load(f)

            sem_bits = int(data.get("semantic_bits_sent", 0))
            bitrate_bps = int(data.get("link_bitrate_bps", 64))

            ratio_h264 = (float(h264_bits) / float(sem_bits)) if sem_bits > 0 else 0.0
            airtime_h264_s = (float(h264_bits) / float(bitrate_bps)) if bitrate_bps > 0 else 0.0

            data["h264_equiv_bits"] = h264_bits
            data["ratio_vs_h264"] = round(ratio_h264, 2)
            data["h264_airtime_at_link_s"] = round(airtime_h264_s, 2)
            data["h264_video_path"] = str(out_mp4)

            with open(summary_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)

            print(f"Updated {summary_path}:")
            print(f"  ratio_vs_h264: {ratio_h264:.2f}x")
            print(f"  h264_airtime: {airtime_h264_s / 3600.0:.2f} hours at {bitrate_bps} bps")
        except Exception as e:  # noqa: BLE001
            print(f"Warning: Failed to update summary.json: {e}")


if __name__ == "__main__":
    main()
