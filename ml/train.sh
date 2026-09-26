#!/usr/bin/env bash
# Training command used for T2.2 (PS11 YOLO11n)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${PROJECT_ROOT}"

yolo detect train \
  model=yolo11n.pt \
  data=ml/data/ps11.yaml \
  epochs=100 `# Ultralytics default, overridden by time=6` \
  time=6 \
  patience=20 \
  batch=-1 \
  imgsz=640 \
  device=0 \
  workers=6 \
  seed=0 \
  project="${PROJECT_ROOT}/ml/runs" \
  name=ps11_v1
