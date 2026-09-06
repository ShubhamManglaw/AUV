#!/usr/bin/env python3
"""Seabed decal generator (§8.2, Task T1.4).

Generates procedural sand texture and pastes real object cut-outs strictly
from images in ml/data/test_manifest.txt (Rule H4).
Outputs:
- Gazebo tile models in ros2_ws/src/ps11_gazebo/models/seabed_tile_<i>_<j>/
- World objects catalog in ros2_ws/src/ps11_bringup/config/world_objects.yaml
- Updates ros2_ws/src/ps11_gazebo/worlds/ocean_demo_kinematic.sdf
"""

from __future__ import annotations

import argparse
import json
import math
import random
import re
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from PIL import Image

# Workspace paths
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
MANIFEST_PATH = WORKSPACE_ROOT / "ml" / "data" / "test_manifest.txt"
RAW_DIR = WORKSPACE_ROOT / "ml" / "data" / "raw"
TRASHCAN_DIR = RAW_DIR / "trashcan" / "dataset" / "instance_version"
DUO_DIR = RAW_DIR / "duo" / "DUO"
GAZEBO_PKG = WORKSPACE_ROOT / "ros2_ws" / "src" / "ps11_gazebo"
BRINGUP_PKG = WORKSPACE_ROOT / "ros2_ws" / "src" / "ps11_bringup"
WORLD_OBJECTS_YAML = BRINGUP_PKG / "config" / "world_objects.yaml"
WORLD_SDF = GAZEBO_PKG / "worlds" / "ocean_demo_kinematic.sdf"
MODELS_DIR = GAZEBO_PKG / "models"


def load_test_manifest(manifest_path: Path = MANIFEST_PATH) -> set[str]:
    """Load normalized test manifest image paths and basenames for Rule H4 verification."""
    if not manifest_path.exists():
        raise FileNotFoundError(f"Test manifest not found at {manifest_path}")

    allowed = set()
    with open(manifest_path, "r", encoding="utf-8") as f:
        for line in f:
            clean = line.strip()
            if clean:
                allowed.add(clean)
                allowed.add(Path(clean).name)
                # Also store canonical relative path from ml/data/
                try:
                    rel_p = Path(clean)
                    allowed.add(str(rel_p))
                except (TypeError, ValueError):
                    pass
    return allowed


def verify_image_in_manifest(
    image_identifier: str | Path, manifest_whitelist: set[str]
) -> None:
    """Honesty Rule H4 guard: refuse any image not in ml/data/test_manifest.txt."""
    path_obj = Path(image_identifier)
    basename = path_obj.name
    str_path = str(image_identifier)

    # Check direct match, basename match, or relative match
    if (
        str_path in manifest_whitelist
        or basename in manifest_whitelist
        or f"ml/data/ps11/images/test/{basename}" in manifest_whitelist
    ):
        return

    raise ValueError(
        f"Rule H4 Violation: Image '{image_identifier}' is NOT in ml/data/test_manifest.txt! "
        "Images pasted into the simulated seabed must strictly originate from the test split."
    )


def extract_trashcan_cutout(
    img_bgr: np.ndarray,
    segmentation: list[list[float]],
    bbox: list[float],
) -> np.ndarray:
    """Extract object cutout using polygon segmentation mask with anti-aliasing."""
    h_img, w_img = img_bgr.shape[:2]
    mask = np.zeros((h_img, w_img), dtype=np.uint8)

    # Draw all polygon segments for this instance
    for poly in segmentation:
        if len(poly) >= 6:
            pts = np.array(poly, dtype=np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(mask, [pts], 255)

    bx, by, bw, bh = [round(v) for v in bbox]
    pad = 4
    x0 = max(0, bx - pad)
    y0 = max(0, by - pad)
    x1 = min(w_img, bx + bw + pad)
    y1 = min(h_img, by + bh + pad)

    sub_bgr = img_bgr[y0:y1, x0:x1]
    sub_mask = mask[y0:y1, x0:x1]

    # Slight blur on mask edge for smooth anti-aliased alpha blending
    sub_mask = cv2.GaussianBlur(sub_mask, (5, 5), sigmaX=1.2)

    # Convert BGR to RGB and add Alpha
    sub_rgb = cv2.cvtColor(sub_bgr, cv2.COLOR_BGR2RGB)
    sub_rgba = np.dstack([sub_rgb, sub_mask])
    return sub_rgba


def extract_duo_cutout(
    img_bgr: np.ndarray,
    bbox: list[float],
) -> np.ndarray:
    """Extract object cutout using bounding box and feathered elliptical alpha mask."""
    h_img, w_img = img_bgr.shape[:2]
    bx, by, bw, bh = [round(v) for v in bbox]

    x0 = max(0, bx)
    y0 = max(0, by)
    x1 = min(w_img, bx + bw)
    y1 = min(h_img, by + bh)

    sub_bgr = img_bgr[y0:y1, x0:x1]
    sh, sw = sub_bgr.shape[:2]
    if sh < 2 or sw < 2:
        return np.zeros((2, 2, 4), dtype=np.uint8)

    # Create elliptical feathered mask
    yy, xx = np.ogrid[:sh, :sw]
    cx = (sw - 1.0) / 2.0
    cy = (sh - 1.0) / 2.0
    rx = max(sw / 2.0, 1.0)
    ry = max(sh / 2.0, 1.0)

    # Normalized radius
    r = np.sqrt(((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2)

    # Alpha: 1.0 inside r <= 0.65, smooth cosine decay to 0 at r = 1.0
    alpha = np.zeros((sh, sw), dtype=np.float32)
    inner = r <= 0.65
    feather = (r > 0.65) & (r <= 1.0)
    alpha[inner] = 1.0
    alpha[feather] = 0.5 * (1.0 + np.cos(np.pi * (r[feather] - 0.65) / 0.35))
    alpha_u8 = (alpha * 255.0).astype(np.uint8)

    sub_rgb = cv2.cvtColor(sub_bgr, cv2.COLOR_BGR2RGB)
    sub_rgba = np.dstack([sub_rgb, alpha_u8])
    return sub_rgba


def generate_procedural_sand_tile(
    tile_x_min: float,
    tile_x_max: float,
    tile_y_min: float,
    tile_y_max: float,
    width_px: int = 3200,
    height_px: int = 3200,
    seed: int = 42,
) -> np.ndarray:
    """Generate a procedural sand texture using noise only (no external images).

    Combines:
    - Base warm sand color [188, 172, 138]
    - Directional seabed ripple waves
    - Large-scale Gaussian dunes (depth variation)
    - Fine grain noise
    """
    rng = np.random.default_rng(seed)

    # World coordinate grid
    # Row 0 corresponds to tile_y_max, Row height-1 to tile_y_min
    x_coords = np.linspace(
        tile_x_min, tile_x_max, width_px, endpoint=False, dtype=np.float32
    )
    y_coords = np.linspace(
        tile_y_max, tile_y_min, height_px, endpoint=False, dtype=np.float32
    )
    xx, yy = np.meshgrid(x_coords, y_coords)

    # 1. Sand ripple waves: wavelength ~1.1 m, angle 0.28 rad
    wavelength = 1.10
    theta = 0.28
    ripple_phase = (2.0 * math.pi / wavelength) * (
        xx * math.cos(theta) + yy * math.sin(theta)
    )
    # Asymmetric underwater sand ripple shape
    ripples = np.sin(ripple_phase) + 0.3 * np.sin(2.0 * ripple_phase + 0.4)
    ripple_intensity = ripples * 11.0  # +/- 14 intensity

    # 2. Large scale dune variation using downscaled smoothed noise
    small_size = 64
    dune_noise = rng.normal(0.0, 1.0, (small_size, small_size)).astype(np.float32)
    dunes = cv2.resize(dune_noise, (width_px, height_px), interpolation=cv2.INTER_CUBIC)
    dune_intensity = dunes * 14.0

    # 3. Fine grain noise
    grain = rng.uniform(-6.0, 6.0, (height_px, width_px)).astype(np.float32)

    # Base sand RGB color: clean sandy ocean floor with underwater visibility
    base_r = 188.0
    base_g = 172.0
    base_b = 138.0

    # Composite luminance modulation
    delta = ripple_intensity + dune_intensity + grain

    img_r = np.clip(base_r + delta * 1.0, 0, 255).astype(np.uint8)
    img_g = np.clip(base_g + delta * 0.95, 0, 255).astype(np.uint8)
    img_b = np.clip(base_b + delta * 0.85, 0, 255).astype(np.uint8)

    sand_rgb = np.dstack([img_r, img_g, img_b])
    return sand_rgb


def collect_available_test_objects(
    manifest_whitelist: set[str],
) -> dict[str, list[dict[str, Any]]]:
    """Collect test split object candidates from TrashCan and DUO adhering to H4."""
    candidates: dict[str, list[dict[str, Any]]] = {
        "debris": [],
        "starfish": [],
        "sea_urchin": [],
        "scallop": [],
    }

    # 1. Load TrashCan test instances (debris)
    tc_val_json = TRASHCAN_DIR / "instances_val_trashcan.json"
    tc_train_json = TRASHCAN_DIR / "instances_train_trashcan.json"

    for json_path, img_dir in [
        (tc_val_json, TRASHCAN_DIR / "val"),
        (tc_train_json, TRASHCAN_DIR / "train"),
    ]:
        if not json_path.exists():
            continue
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        id_to_img = {img["id"]: img for img in data["images"]}
        id_to_cat = {c["id"]: c["name"] for c in data["categories"]}

        for ann in data["annotations"]:
            img_info = id_to_img.get(ann["image_id"])
            if not img_info:
                continue
            fname = img_info["file_name"]
            # Check if this image is in the test manifest whitelist
            if (
                fname not in manifest_whitelist
                and f"ml/data/ps11/images/test/{fname}" not in manifest_whitelist
            ):
                continue

            cat_name = id_to_cat.get(ann["category_id"], "")
            if cat_name.startswith("trash_"):
                # Clean segmentation and size checks
                seg = ann.get("segmentation", [])
                bbox = ann.get("bbox", [])
                if seg and len(seg[0]) >= 6 and bbox[2] >= 30 and bbox[3] >= 30:
                    candidates["debris"].append(
                        {
                            "dataset": "trashcan",
                            "class": "debris",
                            "image_name": fname,
                            "image_path": str(img_dir / fname),
                            "segmentation": seg,
                            "bbox": bbox,
                        }
                    )

    # 2. Load DUO test instances (starfish, sea_urchin, scallop)
    duo_test_json = DUO_DIR / "annotations" / "instances_test.json"
    duo_img_dir = DUO_DIR / "images" / "test"

    if duo_test_json.exists():
        with open(duo_test_json, "r", encoding="utf-8") as f:
            duo_data = json.load(f)

        duo_id_to_img = {img["id"]: img for img in duo_data["images"]}
        duo_cat_map = {
            1: None,  # holothurian (dropped)
            2: "sea_urchin",  # echinus
            3: "scallop",  # scallop
            4: "starfish",  # starfish
        }

        for ann in duo_data["annotations"]:
            img_info = duo_id_to_img.get(ann["image_id"])
            if not img_info:
                continue
            fname = img_info["file_name"]
            if (
                fname not in manifest_whitelist
                and f"ml/data/ps11/images/test/{fname}" not in manifest_whitelist
            ):
                continue

            target_cls = duo_cat_map.get(ann["category_id"])
            if target_cls:
                bbox = ann.get("bbox", [])
                if bbox and bbox[2] >= 20 and bbox[3] >= 20:
                    candidates[target_cls].append(
                        {
                            "dataset": "duo",
                            "class": target_cls,
                            "image_name": fname,
                            "image_path": str(duo_img_dir / fname),
                            "bbox": bbox,
                        }
                    )

    return candidates


def generate_seabed(
    config_path: Path,
    scenario_name: str = "demo",
    manifest_path: Path = MANIFEST_PATH,
) -> list[dict[str, Any]]:
    """Execute complete seabed generation pipeline for the given scenario."""
    # 1. Load configs
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    classes_yaml = config_path.parent / "classes.yaml"
    if not classes_yaml.exists():
        classes_yaml = BRINGUP_PKG / "config" / "classes.yaml"
    with open(classes_yaml, "r", encoding="utf-8") as f:
        classes_data = yaml.safe_load(f)
    id_to_class = {c["id"]: c["name"] for c in classes_data.get("classes", [])}

    mission_yaml = config_path.parent / "mission.yaml"
    if not mission_yaml.exists():
        mission_yaml = BRINGUP_PKG / "config" / "mission.yaml"
    with open(mission_yaml, "r", encoding="utf-8") as f:
        mission_data = yaml.safe_load(f)
    scen_mission = mission_data["scenarios"][scenario_name]
    survey_ox = float(scen_mission["survey_origin_x_m"])
    survey_oy = float(scen_mission["survey_origin_y_m"])
    survey_w = float(scen_mission["area_x_m"])
    survey_h = float(scen_mission["area_y_m"])
    leg_spacing = float(scen_mission["leg_spacing_m"])
    seabed_z = float(scen_mission["seabed_z_m"])

    seed = int(config.get("seed", 42))
    px_per_m = int(config.get("pixel_density_px_per_m", 160))
    tile_size_m = float(config.get("tile_size_m", 20.0))
    min_spacing = float(config.get("min_object_spacing_m", 2.0))
    size_ranges = config.get("object_size_ranges_m", {})

    scenario_cfg = config["scenarios"][scenario_name]
    num_objects = int(scenario_cfg.get("num_objects", 12))
    num_debris = int(scenario_cfg.get("num_debris", 5))
    num_marine = int(scenario_cfg.get("num_marine_life", 7))

    # Fixed seed RNG
    rng = random.Random(seed)

    # 2. Rule H4 Whitelist validation
    manifest_whitelist = load_test_manifest(manifest_path)
    all_candidates = collect_available_test_objects(manifest_whitelist)

    for cls_name, items in all_candidates.items():
        if not items:
            raise RuntimeError(f"No valid test objects found for class '{cls_name}'!")

    # 3. Select 12 objects: 5 debris, 3 starfish, 2 sea_urchin, 2 scallop
    target_counts = {
        id_to_class[0]: num_debris,
        id_to_class[1]: 3,
        id_to_class[2]: 2,
        id_to_class[3]: 2,
    }
    assert sum(target_counts.values()) == num_objects
    assert sum(v for k, v in target_counts.items() if k != id_to_class[0]) == num_marine

    selected_objects: list[dict[str, Any]] = []
    obj_id = 0
    for cls_name, count in target_counts.items():
        pool = list(all_candidates[cls_name])
        # Sort for deterministic shuffle
        pool.sort(key=lambda x: (x["image_name"], str(x["bbox"])))
        rng.shuffle(pool)
        chosen = pool[:count]
        for c in chosen:
            # Enforce H4 verification guard
            verify_image_in_manifest(c["image_name"], manifest_whitelist)

            # Sample physical size
            s_min, s_max = size_ranges[cls_name]
            size_m = round(rng.uniform(s_min, s_max), 3)
            yaw_deg = round(rng.uniform(0.0, 360.0), 1)

            selected_objects.append(
                {
                    "id": obj_id,
                    "class": cls_name,
                    "size_m": size_m,
                    "yaw_deg": yaw_deg,
                    "candidate": c,
                    "source_image": c["image_path"],
                    "source_split": "test",
                }
            )
            obj_id += 1

    # Shuffle objects to distribute classes randomly across positions
    rng.shuffle(selected_objects)

    # 4. Object placement along lawnmower survey legs
    # Compute leg positions dynamically from mission.yaml
    num_legs = round(survey_h / leg_spacing) + 1
    leg_y_coords = [survey_oy + i * leg_spacing for i in range(num_legs)]
    placed_coords: list[tuple[float, float]] = []

    # Place 2 objects per leg within survey swath
    x_leg_start = survey_ox
    x_leg_end = survey_ox + survey_w
    span = x_leg_end - x_leg_start

    for i, leg_y in enumerate(leg_y_coords):
        # Object 1 in first third of leg
        x1 = round(rng.uniform(x_leg_start + 0.20 * span, x_leg_start + 0.35 * span), 2)
        y1 = round(leg_y + rng.uniform(-0.25, 0.25), 2)

        # Object 2 in final third of leg
        x2 = round(rng.uniform(x_leg_start + 0.65 * span, x_leg_start + 0.80 * span), 2)
        y2 = round(leg_y + rng.uniform(-0.25, 0.25), 2)

        placed_coords.extend([(x1, y1), (x2, y2)])

    # Verify spacing >= min_spacing (2.0 m)
    for i in range(len(placed_coords)):
        for j in range(i + 1, len(placed_coords)):
            dist = math.hypot(
                placed_coords[i][0] - placed_coords[j][0],
                placed_coords[i][1] - placed_coords[j][1],
            )
            assert dist >= min_spacing, f"Spacing {dist:.2f} m < {min_spacing} m!"

    # Assign coordinates to selected objects
    for i, obj in enumerate(selected_objects):
        x, y = placed_coords[i]
        obj["x"] = float(x)
        obj["y"] = float(y)
        obj["z"] = float(seabed_z)

    # 5. Extract and scale cutouts
    extracted_cutouts = []
    for obj in selected_objects:
        cand = obj["candidate"]
        img_bgr = cv2.imread(cand["image_path"])
        if img_bgr is None:
            raise FileNotFoundError(f"Cannot read image: {cand['image_path']}")

        if cand["dataset"] == "trashcan":
            cutout_rgba = extract_trashcan_cutout(
                img_bgr, cand["segmentation"], cand["bbox"]
            )
        else:
            cutout_rgba = extract_duo_cutout(img_bgr, cand["bbox"])

        # Target size in pixels
        target_max_px = max(16, round(obj["size_m"] * px_per_m))
        sh, sw = cutout_rgba.shape[:2]
        scale = target_max_px / max(sh, sw)
        new_w = max(4, round(sw * scale))
        new_h = max(4, round(sh * scale))

        resized = cv2.resize(
            cutout_rgba,
            (new_w, new_h),
            interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
        )

        # Rotate cutout
        pil_cutout = Image.fromarray(resized).rotate(
            obj["yaw_deg"], resample=Image.Resampling.BILINEAR, expand=True
        )
        final_cutout = np.array(pil_cutout)
        extracted_cutouts.append(final_cutout)

    # 6. Seabed Tiling
    # Demo scenario: 50 m x 30 m -> covered by 3 x 2 grid of 20 m tiles:
    # X in [0, 60] (tiles 0..2: [0, 20], [20, 40], [40, 60])
    # Y in [-20, 20] (tiles 0..1: [-20, 0], [0, 20])
    tile_grid_x = [0.0, 20.0, 40.0]
    tile_grid_y = [-20.0, 0.0]

    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    generated_tiles = []

    for ix, x_start in enumerate(tile_grid_x):
        x_end = x_start + tile_size_m
        for iy, y_start in enumerate(tile_grid_y):
            y_end = y_start + tile_size_m
            tile_name = f"seabed_tile_{ix}_{iy}"
            tile_center_x = (x_start + x_end) / 2.0
            tile_center_y = (y_start + y_end) / 2.0

            tile_seed = seed + ix * 10 + iy
            tile_rgb = generate_procedural_sand_tile(
                tile_x_min=x_start,
                tile_x_max=x_end,
                tile_y_min=y_start,
                tile_y_max=y_end,
                width_px=3200,
                height_px=3200,
                seed=tile_seed,
            )

            # Paste overlapping objects onto this tile
            for obj_idx, obj in enumerate(selected_objects):
                ox = obj["x"]
                oy = obj["y"]
                cutout = extracted_cutouts[obj_idx]
                ch, cw = cutout.shape[:2]

                # Center pixel in tile coordinates:
                # Column = (ox - x_start) * px_per_m
                # Row = (y_end - oy) * px_per_m (row 0 is y_end)
                c_px = (ox - x_start) * px_per_m
                r_px = (y_end - oy) * px_per_m

                px0 = round(c_px - cw / 2.0)
                py0 = round(r_px - ch / 2.0)
                px1 = px0 + cw
                py1 = py0 + ch

                # Check overlap with [0, 3200]
                if px1 <= 0 or px0 >= 3200 or py1 <= 0 or py0 >= 3200:
                    continue

                # Intersection bounds
                src_x0 = max(0, -px0)
                src_y0 = max(0, -py0)
                src_x1 = cw - max(0, px1 - 3200)
                src_y1 = ch - max(0, py1 - 3200)

                dst_x0 = max(0, px0)
                dst_y0 = max(0, py0)
                dst_x1 = min(3200, px1)
                dst_y1 = min(3200, py1)

                if dst_x1 <= dst_x0 or dst_y1 <= dst_y0:
                    continue

                cut_slice = cutout[src_y0:src_y1, src_x0:src_x1]
                alpha = cut_slice[:, :, 3:4].astype(np.float32) / 255.0
                fg_rgb = cut_slice[:, :, :3].astype(np.float32)
                bg_rgb = tile_rgb[dst_y0:dst_y1, dst_x0:dst_x1].astype(np.float32)

                blended = fg_rgb * alpha + bg_rgb * (1.0 - alpha)
                tile_rgb[dst_y0:dst_y1, dst_x0:dst_x1] = np.clip(
                    blended, 0, 255
                ).astype(np.uint8)

            # Write tile model
            tile_model_dir = MODELS_DIR / tile_name
            tile_textures_dir = tile_model_dir / "materials" / "textures"
            tile_textures_dir.mkdir(parents=True, exist_ok=True)

            texture_path = tile_textures_dir / f"{tile_name}.png"
            cv2.imwrite(str(texture_path), cv2.cvtColor(tile_rgb, cv2.COLOR_RGB2BGR))

            # Write model.config
            config_xml = f"""<?xml version="1.0"?>
<model>
  <name>{tile_name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>make_seabed.py</name>
  </author>
  <description>Seabed tile {tile_name}</description>
</model>
"""
            with open(tile_model_dir / "model.config", "w", encoding="utf-8") as f:
                f.write(config_xml)

            # Write model.sdf
            sdf_xml = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{tile_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{tile_size_m} {tile_size_m}</size>
          </plane>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{tile_size_m} {tile_size_m}</size>
          </plane>
        </geometry>
        <material>
          <ambient>0.85 0.85 0.85 1.0</ambient>
          <diffuse>0.85 0.85 0.85 1.0</diffuse>
          <specular>0.02 0.02 0.02 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://{tile_name}/materials/textures/{tile_name}.png</albedo_map>
              <roughness>0.9</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
            with open(tile_model_dir / "model.sdf", "w", encoding="utf-8") as f:
                f.write(sdf_xml)

            generated_tiles.append((tile_name, tile_center_x, tile_center_y))

    # 7. Write world_objects.yaml
    output_objects = []
    # Sort by ID
    selected_objects.sort(key=lambda o: o["id"])
    for obj in selected_objects:
        output_objects.append(
            {
                "id": int(obj["id"]),
                "class": str(obj["class"]),
                "x": float(obj["x"]),
                "y": float(obj["y"]),
                "z": float(obj["z"]),
                "size_m": float(obj["size_m"]),
                "source_image": str(
                    Path(obj["source_image"]).relative_to(WORKSPACE_ROOT)
                ),
                "source_split": "test",
            }
        )

    WORLD_OBJECTS_YAML.parent.mkdir(parents=True, exist_ok=True)
    yaml_dict = {
        "metadata": {
            "scenario": scenario_name,
            "seed": seed,
            "num_objects": len(output_objects),
            "generated_by": "tools/make_seabed.py",
        },
        "objects": output_objects,
    }
    with open(WORLD_OBJECTS_YAML, "w", encoding="utf-8") as f:
        yaml.dump(yaml_dict, f, sort_keys=False, default_flow_style=False)

    print(
        f"[make_seabed] Written {len(output_objects)} objects to {WORLD_OBJECTS_YAML}"
    )

    # 8. Update ocean_demo_kinematic.sdf to include tiles
    update_world_sdf(generated_tiles)
    return output_objects


def update_world_sdf(tiles: list[tuple[str, float, float]]) -> None:
    """Include tile models in ocean_demo_kinematic.sdf at z = -15, replacing placeholder sand."""
    if not WORLD_SDF.exists():
        raise FileNotFoundError(f"World SDF not found: {WORLD_SDF}")

    with open(WORLD_SDF, "r", encoding="utf-8") as f:
        sdf_content = f.read()

    # Generate tile include XML with clear delimiters
    start_delim = "    <!-- SEABED_TILES_START -->"
    end_delim = "    <!-- SEABED_TILES_END -->"
    tile_includes = [
        start_delim,
        "    <!-- Procedural seabed tiles generated by make_seabed.py -->",
    ]
    for tile_name, cx, cy in tiles:
        inc = f"""    <include>
      <name>{tile_name}</name>
      <uri>model://{tile_name}</uri>
      <pose>{cx:.1f} {cy:.1f} -15.0 0 0 0</pose>
    </include>"""
        tile_includes.append(inc)
    tile_includes.append(end_delim)
    tiles_block = "\n".join(tile_includes)

    if start_delim in sdf_content and end_delim in sdf_content:
        idx_start = sdf_content.find(start_delim)
        idx_end = sdf_content.find(end_delim) + len(end_delim)
        updated_sdf = sdf_content[:idx_start] + tiles_block + sdf_content[idx_end:]
    else:
        # Check if legacy comment exists
        legacy_comment = (
            "    <!-- Procedural seabed tiles generated by make_seabed.py -->"
        )
        placeholder_pattern = r"    <!-- Placeholder seabed plane.*?<\/model>"
        if legacy_comment in sdf_content:
            idx = sdf_content.find(legacy_comment)
            world_end = sdf_content.rfind("  </world>")
            updated_sdf = (
                sdf_content[:idx] + tiles_block + "\n" + sdf_content[world_end:]
            )
        elif re.search(placeholder_pattern, sdf_content, re.DOTALL):
            updated_sdf = re.sub(
                placeholder_pattern, tiles_block, sdf_content, flags=re.DOTALL
            )
        else:
            updated_sdf = sdf_content.replace(
                "  </world>", f"{tiles_block}\n  </world>"
            )

    with open(WORLD_SDF, "w", encoding="utf-8") as f:
        f.write(updated_sdf)
    print(
        f"[make_seabed] Updated {WORLD_SDF} with {len(tiles)} seabed tiles at z = -15.0"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="PS11 AUV Seabed Decal Generator")
    parser.add_argument(
        "--config",
        type=str,
        default="ps11_bringup/config/seabed.yaml",
        help="Path to seabed.yaml configuration file",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default="demo",
        choices=["demo"],
        help="Scenario to generate (demo)",
    )
    parser.add_argument(
        "--manifest",
        type=str,
        default=str(MANIFEST_PATH),
        help="Path to test_manifest.txt for Rule H4 verification",
    )
    args = parser.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        # Try relative to workspace root or current directory
        if (WORKSPACE_ROOT / cfg_path).exists():
            cfg_path = WORKSPACE_ROOT / cfg_path
        elif (WORKSPACE_ROOT / "ros2_ws" / "src" / cfg_path).exists():
            cfg_path = WORKSPACE_ROOT / "ros2_ws" / "src" / cfg_path
        else:
            cfg_path = Path.cwd() / cfg_path

    manifest_path = Path(args.manifest)
    if not manifest_path.is_absolute():
        manifest_path = WORKSPACE_ROOT / manifest_path

    print(
        f"[make_seabed] Starting generation: config={cfg_path}, scenario={args.scenario}"
    )
    generate_seabed(cfg_path, scenario_name=args.scenario, manifest_path=manifest_path)
    print("[make_seabed] Done successfully.")


if __name__ == "__main__":
    main()
