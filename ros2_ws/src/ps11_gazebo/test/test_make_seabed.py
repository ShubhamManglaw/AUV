"""Unit tests for seabed decal generator and Honesty Rule H4 guard (§1.4, §8.2, Task T1.4)."""

import sys
from pathlib import Path

import pytest
import yaml

# Add workspace root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parents[4]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.make_seabed import (
    BRINGUP_PKG,
    MANIFEST_PATH,
    collect_available_test_objects,
    generate_seabed,
    load_test_manifest,
    verify_image_in_manifest,
)


def test_h4_guard_manifest_whitelist():
    """Rule H4 guard unit test: refuse any image not in ml/data/test_manifest.txt."""
    whitelist = load_test_manifest(MANIFEST_PATH)
    assert len(whitelist) > 0, "Whitelist must not be empty"

    # 1. A valid test image from manifest MUST pass
    sample_valid = next(iter(whitelist))
    # Should not raise
    verify_image_in_manifest(sample_valid, whitelist)

    # 2. An image NOT in manifest MUST raise ValueError (blocking Honesty Rule H4)
    invalid_samples = [
        "fake_leak_image.jpg",
        "ml/data/ps11/images/train/vid_000001_frame0000010.jpg",
        "vid_999999_frame9999999.jpg",
        "/tmp/forbidden_image.png",
    ]
    for bad in invalid_samples:
        with pytest.raises(ValueError, match="Rule H4 Violation"):
            verify_image_in_manifest(bad, whitelist)


def test_collected_objects_satisfy_h4():
    """Verify that all collected candidate objects are in the test manifest."""
    whitelist = load_test_manifest(MANIFEST_PATH)
    candidates = collect_available_test_objects(whitelist)

    for cls_name, items in candidates.items():
        assert len(items) > 0, f"Expected candidates for class {cls_name}"
        for item in items:
            # Must pass H4 check
            verify_image_in_manifest(item["image_name"], whitelist)


def test_demo_scenario_counts_and_determinism():
    """Verify object counts match seabed.yaml and running twice with fixed seed gives identical outputs."""
    config_path = BRINGUP_PKG / "config" / "seabed.yaml"
    assert config_path.exists(), f"Config not found at {config_path}"

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    demo_cfg = cfg["scenarios"]["demo"]

    # Run generation
    objs_1 = generate_seabed(config_path, scenario_name="demo")

    # Verify counts
    assert len(objs_1) == demo_cfg["num_objects"] == 12
    debris_count = sum(1 for o in objs_1 if o["class"] == "debris")
    marine_count = sum(1 for o in objs_1 if o["class"] != "debris")
    assert debris_count == demo_cfg["num_debris"] == 5
    assert marine_count == demo_cfg["num_marine_life"] == 7

    # Verify spacing >= min_object_spacing_m (2.0 m)
    for i in range(len(objs_1)):
        for j in range(i + 1, len(objs_1)):
            dx = objs_1[i]["x"] - objs_1[j]["x"]
            dy = objs_1[i]["y"] - objs_1[j]["y"]
            dist = (dx**2 + dy**2) ** 0.5
            assert dist >= 2.0, (
                f"Objects {i} and {j} are too close: {dist:.2f} m < 2.0 m"
            )

    # Verify every object is inside the survey area coverage
    for o in objs_1:
        assert 5.0 <= o["x"] <= 45.0, (
            f"Object {o['id']} X={o['x']} out of survey X [5, 45]"
        )
        assert -11.0 <= o["y"] <= 11.0, (
            f"Object {o['id']} Y={o['y']} out of survey Y [-10, 10]"
        )
        assert o["z"] == -15.0

    # Run generation a second time with same seed
    objs_2 = generate_seabed(config_path, scenario_name="demo")

    # Must be bitwise identical
    assert objs_1 == objs_2, "Outputs with same seed must be identical"


def test_world_objects_yaml_fields():
    """Verify world_objects.yaml schema matches plan §8.2."""
    world_obj_file = BRINGUP_PKG / "config" / "world_objects.yaml"
    assert world_obj_file.exists(), "world_objects.yaml must exist after generation"

    with open(world_obj_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    assert "objects" in data
    objs = data["objects"]
    assert len(objs) == 12

    required_keys = {
        "id",
        "class",
        "x",
        "y",
        "z",
        "size_m",
        "source_image",
        "source_split",
    }
    for o in objs:
        assert required_keys.issubset(o.keys())
        assert o["source_split"] == "test"
