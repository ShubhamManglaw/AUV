#!/usr/bin/env python3
"""Dataset preparation for PS11 AUV perception pipeline (T2.1).

Prepares and maps raw annotations from TrashCan and DUO into YOLO format under ml/data/ps11.
Enforces zero data leakage (video-level split for TrashCan, seed 42).
Generates ml/data/ps11.yaml and ml/data/test_manifest.txt.
"""

import json
import os
import random
import re
import sys
from collections import defaultdict
from pathlib import Path

# Fixed random seed for reproducible splits
SPLIT_SEED = 42

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
ML_DIR = WORKSPACE_ROOT / "ml"
DATA_DIR = ML_DIR / "data"
RAW_DIR = DATA_DIR / "raw"
OUTPUT_DIR = DATA_DIR / "ps11"

# Raw annotation files
TRASHCAN_TRAIN_JSON = (
    RAW_DIR
    / "trashcan"
    / "dataset"
    / "instance_version"
    / "instances_train_trashcan.json"
)
TRASHCAN_VAL_JSON = (
    RAW_DIR
    / "trashcan"
    / "dataset"
    / "instance_version"
    / "instances_val_trashcan.json"
)
TRASHCAN_TRAIN_IMG_DIR = RAW_DIR / "trashcan" / "dataset" / "instance_version" / "train"
TRASHCAN_VAL_IMG_DIR = RAW_DIR / "trashcan" / "dataset" / "instance_version" / "val"

DUO_TRAIN_JSON = RAW_DIR / "duo" / "DUO" / "annotations" / "instances_train.json"
DUO_TEST_JSON = RAW_DIR / "duo" / "DUO" / "annotations" / "instances_test.json"
DUO_TRAIN_IMG_DIR = RAW_DIR / "duo" / "DUO" / "images" / "train"
DUO_TEST_IMG_DIR = RAW_DIR / "duo" / "DUO" / "images" / "test"

# Class definitions (classes.yaml)
TARGET_CLASSES = {
    0: "debris",
    1: "starfish",
    2: "sea_urchin",
    3: "scallop",
}


def load_json(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def print_raw_categories(
    dataset_name: str, split_name: str, coco_data: dict
) -> dict[int, str]:
    print(f"=== Raw Categories: {dataset_name} ({split_name}) ===")
    id_to_name = {}
    for cat in sorted(coco_data["categories"], key=lambda c: c["id"]):
        id_to_name[cat["id"]] = cat["name"]
        print(f"  ID {cat['id']:2d}: {cat['name']}")
    return id_to_name


def extract_trashcan_video_id(filename: str) -> str:
    """Extract video ID prefix from TrashCan image filename.

    Example: vid_000001_frame0000010.jpg -> vid_000001
    """
    match = re.match(r"^(vid_\d+)_frame\d+\.jpg$", filename)
    if match:
        return match.group(1)
    # Fallback to splitting on _frame
    if "_frame" in filename:
        return filename.split("_frame")[0]
    return filename


def map_trashcan_category(cat_name: str) -> int | None:
    """Plan §10.2: all trash_* -> debris (0); animal_starfish -> starfish (1); drop rest."""
    if cat_name.startswith("trash_"):
        return 0
    if cat_name == "animal_starfish":
        return 1
    return None


def map_duo_category(cat_name: str) -> int | None:
    """Plan §10.2: starfish -> starfish (1); echinus -> sea_urchin (2); scallop -> scallop (3); drop holothurian."""
    if cat_name == "starfish":
        return 1
    if cat_name == "echinus":
        return 2
    if cat_name == "scallop":
        return 3
    return None


def convert_bbox_coco_to_yolo(
    bbox: list[float], img_w: int, img_h: int
) -> tuple[float, float, float, float]:
    """Convert COCO [x_min, y_min, w, h] to YOLO [x_center, y_center, width, height] normalized."""
    x_min, y_min, w, h = bbox
    xc = (x_min + w / 2.0) / img_w
    yc = (y_min + h / 2.0) / img_h
    nw = w / img_w
    nh = h / img_h
    # Clamp to [0, 1]
    xc = max(0.0, min(1.0, xc))
    yc = max(0.0, min(1.0, yc))
    nw = max(0.0, min(1.0, nw))
    nh = max(0.0, min(1.0, nh))
    return xc, yc, nw, nh


def main():
    print("=" * 70)
    print("PS11 AUV — Dataset Preparation Pipeline (T2.1)")
    print("=" * 70)

    # 1. Print real category names from all 4 annotation files BEFORE mapping
    tc_train_data = load_json(TRASHCAN_TRAIN_JSON)
    tc_val_data = load_json(TRASHCAN_VAL_JSON)
    duo_train_data = load_json(DUO_TRAIN_JSON)
    duo_test_data = load_json(DUO_TEST_JSON)

    tc_train_cats = print_raw_categories("TrashCan", "train", tc_train_data)
    tc_val_cats = print_raw_categories("TrashCan", "val", tc_val_data)
    duo_train_cats = print_raw_categories("DUO", "train", duo_train_data)
    duo_test_cats = print_raw_categories("DUO", "test", duo_test_data)

    print("\n=== Category Mapping (§10.2) ===")
    print("TrashCan mapping:")
    for cid, name in sorted(tc_train_cats.items()):
        target = map_trashcan_category(name)
        target_name = TARGET_CLASSES[target] if target is not None else "DROPPED"
        print(f"  {name:25s} -> {target_name} ({target})")

    print("\nDUO mapping:")
    for cid, name in sorted(duo_train_cats.items()):
        target = map_duo_category(name)
        target_name = TARGET_CLASSES[target] if target is not None else "DROPPED"
        print(f"  {name:25s} -> {target_name} ({target})")

    # 2. Detect video ID encoding in TrashCan & check train/val video overlap
    print("\n=== TrashCan Video-ID Analysis ===")
    sample_files = [img["file_name"] for img in tc_train_data["images"][:5]]
    print("5 Example filenames from TrashCan:")
    for fn in sample_files:
        vid_id = extract_trashcan_video_id(fn)
        print(f"  Filename: {fn} -> Video ID: {vid_id}")
    print(
        "Video ID Extraction Rule: Prefix matching r'^(vid_\\d+)_frame\\d+\\.jpg$' (part before '_frame')"
    )

    tc_train_vids = {
        extract_trashcan_video_id(img["file_name"]) for img in tc_train_data["images"]
    }
    tc_val_vids = {
        extract_trashcan_video_id(img["file_name"]) for img in tc_val_data["images"]
    }
    tc_overlap = tc_train_vids.intersection(tc_val_vids)
    print(f"Official TrashCan train unique videos: {len(tc_train_vids)}")
    print(f"Official TrashCan val unique videos:   {len(tc_val_vids)}")
    print(
        f"Official TrashCan shared videos:        {len(tc_overlap)} (LEAKAGE DETECTED: {len(tc_overlap)}/{len(tc_val_vids)} val videos exist in train)"
    )

    # 3. Combine TrashCan images/annotations and re-split by video ID to eliminate leakage
    print("\n=== Re-splitting TrashCan by Video ID (Seed 42) ===")
    # Collect all TrashCan images with full image path
    all_tc_images = {}  # image_id_key -> image_info dict
    all_tc_annotations = defaultdict(list)  # image_id_key -> list of target annotations

    # Track category maps
    tc_cat_map_train = {
        cid: map_trashcan_category(name) for cid, name in tc_train_cats.items()
    }
    tc_cat_map_val = {
        cid: map_trashcan_category(name) for cid, name in tc_val_cats.items()
    }

    for img in tc_train_data["images"]:
        key = f"tc_train_{img['id']}"
        img_copy = dict(img)
        img_copy["source_path"] = TRASHCAN_TRAIN_IMG_DIR / img["file_name"]
        img_copy["video_id"] = extract_trashcan_video_id(img["file_name"])
        all_tc_images[key] = img_copy

    for ann in tc_train_data["annotations"]:
        key = f"tc_train_{ann['image_id']}"
        mapped_cls = tc_cat_map_train[ann["category_id"]]
        if mapped_cls is not None:
            all_tc_annotations[key].append((mapped_cls, ann["bbox"]))

    for img in tc_val_data["images"]:
        key = f"tc_val_{img['id']}"
        img_copy = dict(img)
        img_copy["source_path"] = TRASHCAN_VAL_IMG_DIR / img["file_name"]
        img_copy["video_id"] = extract_trashcan_video_id(img["file_name"])
        all_tc_images[key] = img_copy

    for ann in tc_val_data["annotations"]:
        key = f"tc_val_{ann['image_id']}"
        mapped_cls = tc_cat_map_val[ann["category_id"]]
        if mapped_cls is not None:
            all_tc_annotations[key].append((mapped_cls, ann["bbox"]))

    # Group images by video_id
    video_to_img_keys = defaultdict(list)
    for key, img in all_tc_images.items():
        video_to_img_keys[img["video_id"]].append(key)

    unique_tc_videos = sorted(video_to_img_keys.keys())
    rng_tc = random.Random(SPLIT_SEED)
    shuffled_tc_videos = list(unique_tc_videos)
    rng_tc.shuffle(shuffled_tc_videos)

    # 15% test, 90/10 train/val from remainder
    n_tc_test = int(len(shuffled_tc_videos) * 0.15)
    tc_test_vids = set(shuffled_tc_videos[:n_tc_test])
    tc_rem_vids = shuffled_tc_videos[n_tc_test:]
    n_tc_val = int(len(tc_rem_vids) * 0.10)
    tc_val_vids = set(tc_rem_vids[:n_tc_val])
    tc_train_vids = set(tc_rem_vids[n_tc_val:])

    assert len(tc_test_vids.intersection(tc_train_vids)) == 0, (
        "TrashCan test/train video overlap!"
    )
    assert len(tc_test_vids.intersection(tc_val_vids)) == 0, (
        "TrashCan test/val video overlap!"
    )
    assert len(tc_val_vids.intersection(tc_train_vids)) == 0, (
        "TrashCan val/train video overlap!"
    )

    print(f"TrashCan unique videos: {len(unique_tc_videos)}")
    print(f"  Train videos: {len(tc_train_vids)}")
    print(f"  Val videos:   {len(tc_val_vids)}")
    print(f"  Test videos:  {len(tc_test_vids)}")

    # 4. DUO Split: official test = test, train split 90/10 with seed 42
    print("\n=== DUO Split (Seed 42) ===")
    duo_cat_map_train = {
        cid: map_duo_category(name) for cid, name in duo_train_cats.items()
    }
    duo_cat_map_test = {
        cid: map_duo_category(name) for cid, name in duo_test_cats.items()
    }

    all_duo_train_images = {}
    all_duo_train_annotations = defaultdict(list)
    for img in duo_train_data["images"]:
        key = f"duo_train_{img['id']}"
        img_copy = dict(img)
        img_copy["source_path"] = DUO_TRAIN_IMG_DIR / img["file_name"]
        all_duo_train_images[key] = img_copy

    for ann in duo_train_data["annotations"]:
        key = f"duo_train_{ann['image_id']}"
        mapped_cls = duo_cat_map_train[ann["category_id"]]
        if mapped_cls is not None:
            all_duo_train_annotations[key].append((mapped_cls, ann["bbox"]))

    duo_train_keys = sorted(all_duo_train_images.keys())
    rng_duo = random.Random(SPLIT_SEED)
    shuffled_duo_train = list(duo_train_keys)
    rng_duo.shuffle(shuffled_duo_train)

    n_duo_val = int(len(shuffled_duo_train) * 0.10)
    duo_val_keys = set(shuffled_duo_train[:n_duo_val])
    duo_train_split_keys = set(shuffled_duo_train[n_duo_val:])

    # DUO test
    all_duo_test_images = {}
    all_duo_test_annotations = defaultdict(list)
    for img in duo_test_data["images"]:
        key = f"duo_test_{img['id']}"
        img_copy = dict(img)
        img_copy["source_path"] = DUO_TEST_IMG_DIR / img["file_name"]
        all_duo_test_images[key] = img_copy

    for ann in duo_test_data["annotations"]:
        key = f"duo_test_{ann['image_id']}"
        mapped_cls = duo_cat_map_test[ann["category_id"]]
        if mapped_cls is not None:
            all_duo_test_annotations[key].append((mapped_cls, ann["bbox"]))

    print(f"DUO train images: {len(duo_train_split_keys)}")
    print(f"DUO val images:   {len(duo_val_keys)}")
    print(f"DUO test images:  {len(all_duo_test_images)}")

    # 5. Partition all samples into train / val / test splits
    # Each item: (dest_filename, source_path, width, height, annotations_list)
    splits = {
        "train": [],
        "val": [],
        "test": [],
    }

    # Add TrashCan items
    for key, img in all_tc_images.items():
        vid = img["video_id"]
        if vid in tc_train_vids:
            split = "train"
        elif vid in tc_val_vids:
            split = "val"
        else:
            split = "test"
        splits[split].append(
            (
                img["file_name"],
                img["source_path"],
                img["width"],
                img["height"],
                all_tc_annotations[key],
            )
        )

    # Add DUO train/val items
    for key in duo_train_split_keys:
        img = all_duo_train_images[key]
        splits["train"].append(
            (
                img["file_name"],
                img["source_path"],
                img["width"],
                img["height"],
                all_duo_train_annotations[key],
            )
        )

    for key in duo_val_keys:
        img = all_duo_train_images[key]
        splits["val"].append(
            (
                img["file_name"],
                img["source_path"],
                img["width"],
                img["height"],
                all_duo_train_annotations[key],
            )
        )

    for key, img in all_duo_test_images.items():
        splits["test"].append(
            (
                img["file_name"],
                img["source_path"],
                img["width"],
                img["height"],
                all_duo_test_annotations[key],
            )
        )

    # Check for image and video leakage across splits
    print("\n=== Verifying Leakage Prevention ===")
    split_image_sources = {
        s: {item[1] for item in items} for s, items in splits.items()
    }
    assert (
        len(split_image_sources["train"].intersection(split_image_sources["val"])) == 0
    ), "Train and Val share images!"
    assert (
        len(split_image_sources["train"].intersection(split_image_sources["test"])) == 0
    ), "Train and Test share images!"
    assert (
        len(split_image_sources["val"].intersection(split_image_sources["test"])) == 0
    ), "Val and Test share images!"
    print("Verification PASSED: No image path appears in more than one split.")

    # 6. Generate YOLO format dataset
    print("\n=== Generating YOLO Dataset Directory ===")
    for split in ["train", "val", "test"]:
        img_dir = OUTPUT_DIR / "images" / split
        lbl_dir = OUTPUT_DIR / "labels" / split
        img_dir.mkdir(parents=True, exist_ok=True)
        lbl_dir.mkdir(parents=True, exist_ok=True)

    test_manifest_paths = []
    class_counts = {
        "train": defaultdict(int),
        "val": defaultdict(int),
        "test": defaultdict(int),
    }

    for split, items in splits.items():
        img_dir = OUTPUT_DIR / "images" / split
        lbl_dir = OUTPUT_DIR / "labels" / split

        for filename, source_path, width, height, annotations in items:
            dest_img = img_dir / filename
            dest_lbl = lbl_dir / f"{Path(filename).stem}.txt"

            # Create relative symlink
            if dest_img.is_symlink() or dest_img.exists():
                dest_img.unlink()
            rel_source = os.path.relpath(source_path, start=img_dir)
            dest_img.symlink_to(rel_source)

            # Write YOLO label file
            lines = []
            for cls_id, bbox in annotations:
                class_counts[split][cls_id] += 1
                xc, yc, w, h = convert_bbox_coco_to_yolo(bbox, width, height)
                lines.append(f"{cls_id} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")

            with open(dest_lbl, "w", encoding="utf-8") as f:
                f.writelines(lines)

            if split == "test":
                manifest_entry = str(dest_img.relative_to(WORKSPACE_ROOT))
                test_manifest_paths.append(manifest_entry)

    # 7. Write ml/data/test_manifest.txt
    test_manifest_file = DATA_DIR / "test_manifest.txt"
    test_manifest_paths.sort()
    with open(test_manifest_file, "w", encoding="utf-8") as f:
        f.writelines(f"{p}\n" for p in test_manifest_paths)
    print(
        f"Written test manifest: {test_manifest_file} ({len(test_manifest_paths)} test images)"
    )

    # 8. Write ml/data/ps11.yaml
    yaml_content = f"""# PS11 AUV Perception Dataset configuration
path: {OUTPUT_DIR.resolve()}
train: images/train
val: images/val
test: images/test

names:
  0: debris
  1: starfish
  2: sea_urchin
  3: scallop
"""
    yaml_file = DATA_DIR / "ps11.yaml"
    with open(yaml_file, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    print(f"Written YOLO dataset YAML: {yaml_file}")

    # 9. Print summary statistics & verify acceptance criteria
    print("\n" + "=" * 70)
    print("DATASET PREPARATION SUMMARY & STATISTICS")
    print("=" * 70)
    print(
        f"{'Split':<10} | {'Images':<8} | {'Debris (0)':<12} | {'Starfish (1)':<12} | {'Sea Urchin (2)':<14} | {'Scallop (3)':<12}"
    )
    print("-" * 78)
    for split in ["train", "val", "test"]:
        n_imgs = len(splits[split])
        c0 = class_counts[split][0]
        c1 = class_counts[split][1]
        c2 = class_counts[split][2]
        c3 = class_counts[split][3]
        print(f"{split:<10} | {n_imgs:<8} | {c0:<12} | {c1:<12} | {c2:<14} | {c3:<12}")

    print("\n=== Acceptance Criteria Check ===")
    for cls_id, cls_name in TARGET_CLASSES.items():
        cnt = class_counts["train"][cls_id]
        status = "PASS" if cnt >= 200 else "FAIL"
        print(
            f"Class {cls_id} ({cls_name}): {cnt} training instances (>= 200 required) -> {status}"
        )
        if cnt < 200:
            print(
                f"ERROR: Class {cls_name} has only {cnt} training instances (< 200)!",
                file=sys.stderr,
            )
            sys.exit(1)

    print("\nDataset preparation completed successfully!")


if __name__ == "__main__":
    main()
