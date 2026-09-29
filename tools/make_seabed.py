#!/usr/bin/env python3
"""Seabed decal/object generator for PS11 AUV (T1.4, plan §8.2).

Generates, for the configured scenario (demo only for M1):
- Gazebo seabed tile models (procedural sand texture + real-object decals)
  under ps11_gazebo/models/seabed_tile_<i>_<j>/
- Ground-truth object list ros2_ws/src/ps11_bringup/config/world_objects.yaml

Honesty rule H4 (hard requirement): every source image MUST be a member of
ml/data/test_manifest.txt. Any requested or selected image outside the
manifest is rejected with ValueError. Test-split labels (ml/data/ps11/labels)
and the TrashCan COCO annotations are used only to identify instances inside
those test images.

Determinism: the same manifest + config seed + scenario produce identical
object selection, placement and textures (fixed-seed RNGs only).
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MANIFEST = REPO_ROOT / "ml" / "data" / "test_manifest.txt"
DEFAULT_LABELS = REPO_ROOT / "ml" / "data" / "ps11" / "labels" / "test"
DEFAULT_MODELS = REPO_ROOT / "ros2_ws" / "src" / "ps11_gazebo" / "models"
DEFAULT_GT = (
    REPO_ROOT / "ros2_ws" / "src" / "ps11_bringup" / "config" / "world_objects.yaml"
)
DEFAULT_TRASHCAN_JSON = (
    REPO_ROOT / "ml" / "data" / "raw" / "trashcan" / "dataset" / "instance_version"
)


def load_manifest(path: Path) -> list[str]:
    """Load the test manifest as a sorted list of repo-relative image paths."""
    lines = [l.strip() for l in Path(path).read_text(encoding="utf-8").splitlines()]
    manifest = sorted(l for l in lines if l)
    if not manifest:
        raise ValueError(f"manifest {path} is empty")
    return manifest


def repo_relative(path: str | Path, repo_root: Path = REPO_ROOT) -> str:
    """Normalize an image path to a repo-relative string for manifest checks.

    Purely lexical (os.path.normpath): manifest entries are symlinks into
    ml/data/raw/, so they must NOT be resolved to their physical targets.
    """
    import os

    p = Path(path)
    abs_lex = Path(os.path.normpath(p if p.is_absolute() else repo_root / p))
    return os.path.relpath(abs_lex, repo_root)


def assert_in_manifest(
    path: str | Path, manifest: list[str] | set[str], repo_root: Path = REPO_ROOT
) -> str:
    """H4 guard: raise ValueError unless `path` is a manifest member.

    Returns the normalized repo-relative path on success.
    """
    rel = repo_relative(path, repo_root)
    members = set(manifest)
    if rel not in members:
        raise ValueError(
            f"H4 violation: {rel} is not in test_manifest.txt — seabed images may "
            "only come from the verified test manifest"
        )
    return rel


def choose_marine_composition(marine_class_ids: list[int], n_marine: int) -> list[int]:
    """Deterministic round-robin over class-id-sorted marine classes."""
    ordered = sorted(marine_class_ids)
    return [ordered[i % len(ordered)] for i in range(n_marine)]


def load_test_labels(labels_dir: Path) -> dict[str, set[int]]:
    """Map label stem -> set of YOLO class ids present in that test image."""
    out: dict[str, set[int]] = {}
    for lbl in Path(labels_dir).glob("*.txt"):
        ids = set()
        for line in lbl.read_text(encoding="utf-8").splitlines():
            if line.strip():
                ids.add(int(line.split()[0]))
        out[lbl.stem] = ids
    return out


def select_sources(
    counts_by_class: dict[int, int],
    manifest: list[str],
    labels: dict[str, set[int]],
    repo_root: Path = REPO_ROOT,
    seed: int = 42,
) -> dict[int, list[str]]:
    """Deterministically pick source test images per class.

    Candidates are manifest members whose test label contains the class and
    whose file exists on disk. H4 guard applies to every pick; images are not
    reused across objects.
    """
    members = set(manifest)
    candidates: dict[int, list[str]] = defaultdict(list)
    for entry in manifest:
        stem = Path(entry).stem
        for cid in labels.get(stem, set()):
            candidates[cid].append(entry)
    for cid, entries in candidates.items():
        entries.sort()

    rng = random.Random(seed)
    picked: dict[int, list[str]] = {}
    used: set[str] = set()
    for cid, n in sorted(counts_by_class.items()):
        pool = candidates.get(cid, [])
        chosen = []
        for _ in range(n):
            available = [p for p in pool if p not in used]
            if not available:
                raise ValueError(f"no unused manifest image with class id {cid}")
            pick = rng.choice(available)
            rel = assert_in_manifest(pick, members, repo_root)
            if not (repo_root / rel).exists():
                raise ValueError(f"selected source image does not exist: {rel}")
            used.add(pick)
            chosen.append(rel)
        picked[cid] = chosen
    return picked


def place_objects(
    rng: random.Random,
    area_x_m: float,
    area_y_m: float,
    n: int,
    min_spacing_m: float,
    max_attempts: int = 100_000,
) -> list[tuple[float, float]]:
    """Uniform placement with rejection sampling on the minimum spacing."""
    placed: list[tuple[float, float]] = []
    attempts = 0
    while len(placed) < n:
        attempts += 1
        if attempts > max_attempts:
            raise ValueError("object placement failed: spacing infeasible")
        x = rng.uniform(-area_x_m / 2.0, area_x_m / 2.0)
        y = rng.uniform(-area_y_m / 2.0, area_y_m / 2.0)
        if all(math.hypot(x - px, y - py) >= min_spacing_m for px, py in placed):
            placed.append((x, y))
    return placed


def sample_sizes(rng: random.Random, ranges: dict, class_name: str) -> float:
    lo, hi = ranges[class_name]
    return rng.uniform(lo, hi)


def trashcan_ann_index(json_dir: Path) -> dict[str, list[dict]]:
    """file_name -> list of TrashCan COCO annotations, from official train+val."""
    import pycocotools.mask as mask_utils  # noqa: F401  (ensures availability)
    from pycocotools.coco import COCO  # noqa: F401

    index: dict[str, list[dict]] = defaultdict(list)
    for json_path in sorted(Path(json_dir).glob("instances_*_trashcan.json")):
        data = json_load(json_path)
        imgs = {img["id"]: img["file_name"] for img in data["images"]}
        for ann in data["annotations"]:
            fname = imgs[ann["image_id"]]
            index[fname].append(ann)
    return index


def json_load(path: Path) -> dict:
    import json

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def trashcan_class_id(ann: dict, cat_map: dict[int, int | None]) -> int | None:
    return cat_map.get(ann["category_id"])


def largest_ann(
    anns: list[dict], want_cls: int, cat_map: dict[int, int | None]
) -> dict | None:
    """Deterministically pick the largest-area annotation of the wanted class."""
    best = None
    for ann in anns:
        if trashcan_class_id(ann, cat_map) != want_cls:
            continue
        if best is None or ann["area"] > best["area"]:
            best = ann
    return best


def scale_to_mask_extent(
    crop: np.ndarray, alpha: np.ndarray, size_m: float, px_per_m: int
) -> tuple[np.ndarray, np.ndarray]:
    """Rescale a decal so the MASK's own extent (largest side, alpha > 0.5)
    spans size_m — size_m is the physical object size, not the crop size."""
    ys, xs = np.where(alpha > 0.5)
    if len(ys) == 0:
        return crop, alpha
    extent_px = float(max(ys.max() - ys.min() + 1, xs.max() - xs.min() + 1))
    scale = (size_m * px_per_m) / extent_px
    new_w = max(round(crop.shape[1] * scale), 1)
    new_h = max(round(crop.shape[0] * scale), 1)
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return (
        cv2.resize(crop, (new_w, new_h), interpolation=interp),
        cv2.resize(alpha, (new_w, new_h), interpolation=interp),
    )


def extract_trashcan_decal(
    img_path: Path, ann: dict, cat_id_to_name: dict[int, str]
) -> tuple[np.ndarray, np.ndarray]:
    """RGB crop + feathered instance-mask alpha for a TrashCan annotation."""
    from pycocotools.coco import COCO

    img = cv2.cvtColor(cv2.imread(str(img_path), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    coco = COCO.__new__(COCO)  # annToMask without loading the full dataset
    coco.anns = {}
    x, y, w, h = (float(v) for v in ann["bbox"])
    x0, y0 = max(int(x), 0), max(int(y), 0)
    x1 = min(math.ceil(x + w), img.shape[1])
    y1 = min(math.ceil(y + h), img.shape[0])
    mask = _ann_to_mask(ann, img.shape[1], img.shape[0])
    crop = img[y0:y1, x0:x1]
    alpha = mask[y0:y1, x0:x1].astype(np.float32)
    if crop.size == 0:
        raise ValueError(f"empty TrashCan crop for {img_path}")
    alpha = cv2.GaussianBlur(alpha, (9, 9), 0)
    alpha = np.clip(alpha, 0.0, 1.0)
    return crop, alpha


def _ann_to_mask(ann: dict, width: int, height: int) -> np.ndarray:
    """Rasterize a COCO annotation segmentation without a full COCO object."""
    import pycocotools.mask as mask_utils

    seg = ann["segmentation"]
    if isinstance(seg, list):
        rles = mask_utils.frPyObjects(seg, height, width)
        rle = mask_utils.merge(rles)
    elif isinstance(seg["counts"], list):
        rle = mask_utils.frPyObjects(seg, height, width)
    else:
        rle = seg
    return mask_utils.decode(rle)


def extract_duo_decal(
    img_path: Path, bbox_norm: tuple[float, float, float, float]
) -> tuple[np.ndarray, np.ndarray]:
    """RGB crop + elliptical feathered alpha for a DUO bbox (plan §8.2)."""
    img = cv2.cvtColor(cv2.imread(str(img_path), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    ih, iw = img.shape[:2]
    xc, yc, nw, nh = bbox_norm
    w, h = int(nw * iw), int(nh * ih)
    cx, cy = int(xc * iw), int(yc * ih)
    x0, y0 = max(cx - w // 2, 0), max(cy - h // 2, 0)
    x1, y1 = min(cx + w // 2 + 1, iw), min(cy + h // 2 + 1, ih)
    crop = img[y0:y1, x0:x1]
    if crop.size == 0:
        raise ValueError(f"empty DUO crop for {img_path}")
    alpha = np.zeros(crop.shape[:2], dtype=np.float32)
    cv2.ellipse(
        alpha,
        (crop.shape[1] // 2, crop.shape[0] // 2),
        (max(crop.shape[1] // 2 - 2, 1), max(crop.shape[0] // 2 - 2, 1)),
        0,
        0,
        360,
        1.0,
        -1,
    )
    alpha = cv2.GaussianBlur(alpha, (15, 15), 0)
    return crop, alpha


def rotate_patch(
    crop: np.ndarray, alpha: np.ndarray, yaw_rad: float
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate an RGB+alpha decal patch by yaw around its centre."""
    h, w = crop.shape[:2]
    diag = math.ceil(math.hypot(h, w)) + 2
    canvas_c = np.zeros((diag, diag, 3), dtype=crop.dtype)
    canvas_a = np.zeros((diag, diag), dtype=np.float32)
    oy, ox = (diag - h) // 2, (diag - w) // 2
    canvas_c[oy : oy + h, ox : ox + w] = crop
    canvas_a[oy : oy + h, ox : ox + w] = alpha
    M = cv2.getRotationMatrix2D((diag / 2.0, diag / 2.0), math.degrees(yaw_rad), 1.0)
    rot_c = cv2.warpAffine(canvas_c, M, (diag, diag), flags=cv2.INTER_LINEAR)
    rot_a = cv2.warpAffine(canvas_a, M, (diag, diag), flags=cv2.INTER_LINEAR)
    return rot_c, rot_a


def make_sand_texture(height: int, width: int, seed: int) -> np.ndarray:
    """Procedural sand: layered seeded noise, no external images (plan §8.2)."""
    rng = np.random.default_rng(seed)
    base = np.array([194.0, 178.0, 127.0])  # warm sand, matches world ambient
    texture = np.ones((height, width, 3), dtype=np.float32) * base
    for octave, (grid_h, grid_w, amp) in enumerate(
        [(6, 10, 14.0), (24, 40, 9.0), (96, 160, 5.0), (height // 8, width // 8, 3.0)]
    ):
        noise = rng.standard_normal((grid_h, grid_w, 1)).astype(np.float32)
        noise = cv2.resize(noise, (width, height), interpolation=cv2.INTER_LINEAR)[
            :, :, None
        ]
        texture += noise * amp
    texture += rng.standard_normal((height, width, 1)).astype(np.float32) * 4.0
    return np.clip(texture, 0, 255).astype(np.uint8)


def tile_grid(
    area_x_m: float, area_y_m: float, tile_m: float
) -> list[tuple[int, int, float, float]]:
    """Tile (i, j, x_min, y_min) covering the area, aligned so the area centre
    sits at the centre of the tile grid."""
    nx = math.ceil(area_x_m / tile_m)
    ny = math.ceil(area_y_m / tile_m)
    x_origin = -(nx * tile_m) / 2.0
    y_origin = -(ny * tile_m) / 2.0
    return [
        (i, j, x_origin + i * tile_m, y_origin + j * tile_m)
        for j in range(ny)
        for i in range(nx)
    ]


def render_tiles(
    tiles: list[tuple[int, int, float, float]],
    objects: list[dict],
    decals: dict[int, tuple[np.ndarray, np.ndarray]],
    tile_m: float,
    px_per_m: int,
    out_dir: Path,
    seed: int,
) -> list[Path]:
    """Render each tile PNG (sand + rotated decal patches) and write the
    Gazebo model files (plan §8.2 tile models; pattern proven in T1.2/T1.4)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    px = int(tile_m * px_per_m)
    written: list[Path] = []
    for i, j, x_min, y_min in tiles:
        name = f"seabed_tile_{i}_{j}"
        texture = make_sand_texture(px, px, seed * 1000 + i * 17 + j)
        for obj in objects:
            ox, oy = obj["x"], obj["y"]
            if not (x_min <= ox < x_min + tile_m and y_min <= oy < y_min + tile_m):
                continue
            crop_r, alpha_r = rotate_patch(*decals[obj["id"]], obj["yaw_rad"])
            new_h, new_w = crop_r.shape[:2]

            col = int((ox - x_min) * px_per_m)
            row = int((y_min + tile_m - oy) * px_per_m)  # image row 0 = max y
            r0, c0 = row - new_h // 2, col - new_w // 2
            r1, c1 = r0 + new_h, c0 + new_w
            tr0, tc0 = max(r0, 0), max(c0, 0)
            tr1, tc1 = min(r1, px), min(c1, px)
            if tr1 <= tr0 or tc1 <= tc0:
                continue
            sr0, sc0 = tr0 - r0, tc0 - c0
            a = alpha_r[sr0 : sr0 + (tr1 - tr0), sc0 : sc0 + (tc1 - tc0)].astype(
                np.float32
            )
            region = texture[tr0:tr1, tc0:tc1].astype(np.float32)
            decal = crop_r[sr0 : sr0 + (tr1 - tr0), sc0 : sc0 + (tc1 - tc0)].astype(
                np.float32
            )
            texture[tr0:tr1, tc0:tc1] = (
                region * (1.0 - a[..., None]) + decal * a[..., None]
            ).astype(np.uint8)

        tile_dir = out_dir / name
        tex_dir = tile_dir / "materials" / "textures"
        tex_dir.mkdir(parents=True, exist_ok=True)
        png = tex_dir / f"{name}.png"
        cv2.imwrite(str(png), cv2.cvtColor(texture, cv2.COLOR_RGB2BGR))
        (tile_dir / "model.config").write_text(
            '<?xml version="1.0"?>\n<model>\n  <name>' + name + "</name>\n"
            '  <version>1.0</version>\n  <sdf version="1.8">model.sdf</sdf>\n'
            "  <author>\n    <name>make_seabed.py</name>\n  </author>\n"
            "  <description>Seabed tile "
            + name
            + " (generated, deterministic)</description>\n</model>\n",
            encoding="utf-8",
        )
        (tile_dir / "model.sdf").write_text(
            '<?xml version="1.0" ?>\n<sdf version="1.8">\n'
            f'  <model name="{name}">\n    <static>true</static>\n'
            '    <link name="link">\n      <collision name="collision">\n'
            "        <geometry>\n          <plane>\n            <normal>0 0 1</normal>\n"
            f"            <size>{tile_m} {tile_m}</size>\n          </plane>\n"
            "        </geometry>\n      </collision>\n"
            '      <visual name="visual">\n        <geometry>\n          <plane>\n'
            "            <normal>0 0 1</normal>\n"
            f"            <size>{tile_m} {tile_m}</size>\n          </plane>\n"
            "        </geometry>\n        <material>\n"
            "          <ambient>0.85 0.85 0.85 1.0</ambient>\n"
            "          <diffuse>0.85 0.85 0.85 1.0</diffuse>\n"
            "          <specular>0.02 0.02 0.02 1.0</specular>\n"
            "          <pbr>\n            <metal>\n"
            f"              <albedo_map>model://{name}/materials/textures/{name}.png</albedo_map>\n"
            "              <roughness>0.9</roughness>\n              <metalness>0.0</metalness>\n"
            "            </metal>\n          </pbr>\n        </material>\n      </visual>\n"
            "    </link>\n  </model>\n</sdf>\n",
            encoding="utf-8",
        )
        written.append(png)
    return written


def write_world_objects(
    gt_out: Path, scenario: str, seed: int, objects: list[dict]
) -> None:
    gt_out.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# Ground-truth seabed objects — GENERATED by tools/make_seabed.py (T1.4, plan §8.2).\n"
        f"# scenario: {scenario} · seed: {seed} · H4: every source_image is a member of\n"
        "# ml/data/test_manifest.txt (sha256 5d22b285b68051f6f21c77e43adb987c61b75897e88fc1db23d4dd473040bbf3).\n"
        "# Do not hand-edit; regenerate instead.\n"
    )
    body = yaml.safe_dump(
        {"scenario": scenario, "seed": seed, "objects": objects},
        sort_keys=False,
        default_flow_style=False,
    )
    gt_out.write_text(header + body, encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(REPO_ROOT / "ps11_bringup" / "config" / "seabed.yaml")
        if (REPO_ROOT / "ps11_bringup").exists()
        else str(
            REPO_ROOT / "ros2_ws" / "src" / "ps11_bringup" / "config" / "seabed.yaml"
        ),
    )
    parser.add_argument("--scenario", default="demo")
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--labels-dir", default=str(DEFAULT_LABELS))
    parser.add_argument("--models-dir", default=str(DEFAULT_MODELS))
    parser.add_argument("--gt-out", default=str(DEFAULT_GT))
    parser.add_argument("--trashcan-json-dir", default=str(DEFAULT_TRASHCAN_JSON))
    args = parser.parse_args(argv)

    from ps11_common.classes import ClassDatabase

    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    scenario_cfg = cfg["scenarios"][args.scenario]
    seed = int(cfg["seed"])
    classes = ClassDatabase(
        REPO_ROOT / "ros2_ws" / "src" / "ps11_bringup" / "config" / "classes.yaml"
    )

    manifest = load_manifest(Path(args.manifest))
    print(f"Loaded manifest: {len(manifest)} test images (H4 source of truth)")

    debris_id = classes.get_by_name("debris").id
    marine_ids = sorted(c.id for c in classes.all_classes() if c.id != debris_id)
    composition = [debris_id] * int(
        scenario_cfg["num_debris"]
    ) + choose_marine_composition(marine_ids, int(scenario_cfg["num_marine_life"]))
    composition.sort()
    print(f"Object composition (class ids): {composition}")

    picked = select_sources(
        {cid: composition.count(cid) for cid in set(composition)},
        manifest,
        load_test_labels(Path(args.labels_dir)),
        REPO_ROOT,
        seed,
    )
    for cid, paths in sorted(picked.items()):
        for p in paths:
            print(f"  class {cid} ({classes.get_by_id(cid).name}): {p}")

    rng = random.Random(seed)
    placements = place_objects(
        rng,
        scenario_cfg["area_x_m"],
        scenario_cfg["area_y_m"],
        len(composition),
        float(cfg["min_object_spacing_m"]),
    )

    objects: list[dict] = []
    for k, cid in enumerate(composition):
        name = classes.get_by_id(cid).name
        x, y = placements[k]
        objects.append(
            {
                "id": k,
                "class_id": cid,
                "class_name": name,
                "x": round(x, 3),
                "y": round(y, 3),
                "z": -15.0,
                "size_m": round(
                    sample_sizes(rng, cfg["object_size_ranges_m"], name), 3
                ),
                "yaw_rad": round(rng.uniform(0.0, 2.0 * math.pi), 3),
                "source_image": picked[cid].pop(0),
                "source_split": "test",
            }
        )
    for obj in objects:
        assert_in_manifest(obj["source_image"], manifest)  # final H4 sweep

    tiles = tile_grid(
        scenario_cfg["area_x_m"], scenario_cfg["area_y_m"], float(cfg["tile_size_m"])
    )
    print(
        f"Tile grid: {len(tiles)} tiles of {cfg['tile_size_m']} m at {cfg['pixel_density_px_per_m']} px/m"
    )

    # Decal extraction (H4: images already manifest-verified)
    tc_anns = trashcan_ann_index(Path(args.trashcan_json_dir))
    cat_maps = {}
    for json_path in sorted(
        Path(args.trashcan_json_dir).glob("instances_*_trashcan.json")
    ):
        data = json_load(json_path)
        for cat in data["categories"]:
            mapped = (
                0
                if cat["name"].startswith("trash_")
                else (1 if cat["name"] == "animal_starfish" else None)
            )
            cat_maps[cat["id"]] = mapped
    decals: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for obj in objects:
        img_path = REPO_ROOT / obj["source_image"]
        if "vid_" in obj["source_image"]:
            ann = largest_ann(tc_anns[Path(img_path).name], obj["class_id"], cat_maps)
            if ann is None:
                raise ValueError(
                    f"no TrashCan instance of class {obj['class_id']} in {img_path.name}"
                )
            decals[obj["id"]] = extract_trashcan_decal(img_path, ann, {})
        else:
            stem = Path(img_path).stem
            lbl = Path(args.labels_dir) / f"{stem}.txt"
            best, best_area = None, -1.0
            for line in lbl.read_text().splitlines():
                if not line.strip():
                    continue
                parts = line.split()
                if int(parts[0]) != obj["class_id"]:
                    continue
                nw, nh = float(parts[3]), float(parts[4])
                if nw * nh > best_area:
                    best, best_area = tuple(float(v) for v in parts[1:5]), nw * nh
            if best is None:
                raise ValueError(
                    f"no DUO instance of class {obj['class_id']} in {lbl.name}"
                )
            decals[obj["id"]] = extract_duo_decal(img_path, best)

    pngs = render_tiles(
        tiles,
        objects,
        decals,
        float(cfg["tile_size_m"]),
        int(cfg["pixel_density_px_per_m"]),
        Path(args.models_dir),
        seed,
    )
    print(f"Wrote {len(pngs)} tile textures + model files under {args.models_dir}")
    write_world_objects(Path(args.gt_out), args.scenario, seed, objects)
    print(f"Wrote ground truth: {args.gt_out} ({len(objects)} objects)")

    print(
        "\nWorld include block (paste into the scenario world, replacing the placeholder):"
    )
    for i, j, x_min, y_min in tiles:
        cx, cy = x_min + cfg["tile_size_m"] / 2.0, y_min + cfg["tile_size_m"] / 2.0
        print(
            f"    <include><name>seabed_tile_{i}_{j}</name>"
            f"<uri>model://seabed_tile_{i}_{j}</uri>"
            f"<pose>{cx} {cy} -15.0 0 0 0</pose></include>"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
