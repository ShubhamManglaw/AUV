"""Focused unit tests for the T1.4 seabed generator (plan §8.2, H4).

ROS-free. Uses tiny synthetic fixtures (temp dirs) — the real dataset is not
needed to test manifest enforcement, determinism, bounds and schema.
"""

from __future__ import annotations

import random
import sys
from pathlib import Path

import pytest
import yaml

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

from make_seabed import (
    assert_in_manifest,
    choose_marine_composition,
    load_manifest,
    place_objects,
    select_sources,
    tile_grid,
)

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture()
def manifest_file(tmp_path: Path) -> Path:
    entries = sorted(f"ml/data/ps11/images/test/img_{i}.jpg" for i in range(6))
    p = tmp_path / "test_manifest.txt"
    p.write_text("\n".join(entries) + "\n", encoding="utf-8")
    return p


def make_fake_sources(repo: Path, manifest: list[str]) -> None:
    """Create the actual image files the manifest refers to."""
    for rel in manifest:
        f = repo / rel
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_bytes(b"\xff\xd8fake")


def test_every_selected_image_belongs_to_manifest(
    manifest_file: Path, tmp_path: Path
) -> None:
    manifest = load_manifest(manifest_file)
    make_fake_sources(tmp_path, manifest)
    repo = tmp_path
    labels = {"img_0": {0}, "img_1": {0}, "img_2": {1}}
    picked = select_sources({0: 1, 1: 1}, manifest, labels, repo_root=repo, seed=1)
    for paths in picked.values():
        for p in paths:
            assert p in set(manifest)


def test_non_manifest_image_is_rejected(manifest_file: Path, tmp_path: Path) -> None:
    manifest = load_manifest(manifest_file)
    with pytest.raises(ValueError, match="H4 violation"):
        assert_in_manifest(
            "ml/data/ps11/images/test/not_in_manifest.jpg", manifest, repo_root=tmp_path
        )
    with pytest.raises(ValueError, match="H4 violation"):
        assert_in_manifest(
            str(tmp_path / "ml/data/train/images/x.jpg"), manifest, repo_root=tmp_path
        )


def test_deterministic_output_with_fixed_seed(
    manifest_file: Path, tmp_path: Path
) -> None:
    manifest = load_manifest(manifest_file)
    make_fake_sources(tmp_path, manifest)
    labels = {
        "img_0": {0},
        "img_1": {0},
        "img_2": {0},
        "img_3": {1},
        "img_4": {1},
        "img_5": {2},
    }
    a = select_sources({0: 2, 1: 1}, manifest, labels, repo_root=tmp_path, seed=42)
    b = select_sources({0: 2, 1: 1}, manifest, labels, repo_root=tmp_path, seed=42)
    c = select_sources({0: 2, 1: 1}, manifest, labels, repo_root=tmp_path, seed=43)
    assert a == b
    assert a != c
    ra = place_objects(random.Random(42), 50.0, 30.0, 12, 2.0)
    rb = place_objects(random.Random(42), 50.0, 30.0, 12, 2.0)
    assert ra == rb


def test_object_ids_unique_and_complete(manifest_file: Path, tmp_path: Path) -> None:
    manifest = load_manifest(manifest_file)
    make_fake_sources(tmp_path, manifest)
    labels = {
        "img_0": {0},
        "img_1": {0},
        "img_2": {0},
        "img_3": {0},
        "img_4": {2},
        "img_5": {1},
    }
    with pytest.raises(ValueError):
        # more objects requested than unused candidate images allow (no reuse)
        select_sources(
            {0: 5, 1: 2, 2: 1}, manifest, labels, repo_root=tmp_path, seed=42
        )
    picked = select_sources(
        {0: 4, 1: 1, 2: 1}, manifest, labels, repo_root=tmp_path, seed=42
    )
    ids = [cid for cid, paths in picked.items() for _ in paths]
    assert sorted(ids) == [0, 0, 0, 0, 1, 2]
    used = [p for paths in picked.values() for p in paths]
    assert len(used) == len(set(used))


def test_class_ids_loaded_from_classes_yaml() -> None:
    from ps11_common.classes import ClassDatabase

    db = ClassDatabase(
        REPO / "ros2_ws" / "src" / "ps11_bringup" / "config" / "classes.yaml"
    )
    names = {c.id: c.name for c in db.all_classes()}
    assert names == {0: "debris", 1: "starfish", 2: "sea_urchin", 3: "scallop"}
    marine = choose_marine_composition([c.id for c in db.all_classes() if c.id != 0], 7)
    assert sorted(marine) == [1, 1, 1, 2, 2, 3, 3]


def test_placements_lie_within_demo_bounds() -> None:
    pts = place_objects(random.Random(42), 50.0, 30.0, 12, 2.0)
    assert len(pts) == 12
    for x, y in pts:
        assert -25.0 <= x <= 25.0
        assert -15.0 <= y <= 15.0
    for i, (x0, y0) in enumerate(pts):
        for x1, y1 in pts[i + 1 :]:
            assert ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5 >= 2.0


def test_generated_schema_loads(tmp_path: Path) -> None:
    objects = [
        {
            "id": 0,
            "class_id": 0,
            "class_name": "debris",
            "x": -1.5,
            "y": 2.0,
            "z": -15.0,
            "size_m": 0.8,
            "yaw_rad": 0.5,
            "source_image": "ml/data/ps11/images/test/img_0.jpg",
            "source_split": "test",
        }
    ]
    from make_seabed import write_world_objects

    out = tmp_path / "world_objects.yaml"
    write_world_objects(out, "demo", 42, objects)
    doc = yaml.safe_load(out.read_text(encoding="utf-8"))
    assert doc["scenario"] == "demo" and doc["seed"] == 42
    assert len(doc["objects"]) == 1
    obj = doc["objects"][0]
    for key in (
        "id",
        "class_id",
        "class_name",
        "x",
        "y",
        "z",
        "size_m",
        "source_image",
        "source_split",
    ):
        assert key in obj


def test_selected_source_files_must_exist(manifest_file: Path, tmp_path: Path) -> None:
    manifest = load_manifest(manifest_file)
    labels = {"img_0": {0}}  # img_0.jpg NOT created on disk
    with pytest.raises(ValueError, match="does not exist"):
        select_sources({0: 1}, manifest, labels, repo_root=tmp_path, seed=42)


def test_tile_grid_covers_demo_area() -> None:
    tiles = tile_grid(50.0, 30.0, 20.0)
    assert len(tiles) == 6
    xs = {x0 for _i, _j, x0, _y0 in tiles}
    ys = {y0 for _i, _j, _x0, y0 in tiles}
    assert xs == {-30.0, -10.0, 10.0}
    assert ys == {-20.0, 0.0}
    # Survey area (±25, ±15) fully covered by the grid (±30, ±20)
    covered = all(
        any(x0 <= x <= x0 + 20 and y0 <= y <= y0 + 20 for _i, _j, x0, y0 in tiles)
        for x, y in [(-25, -15), (25, -15), (-25, 15), (25, 15), (0, 0)]
    )
    assert covered
