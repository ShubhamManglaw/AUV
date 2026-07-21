"""Tests for YAML params loading helper."""

import pytest
from ps11_common.params import get_config_path, load_yaml


def test_load_yaml_classes() -> None:
    data = load_yaml("classes.yaml")
    assert "classes" in data
    assert len(data["classes"]) == 4


def test_load_all_configs() -> None:
    expected_configs = [
        "classes.yaml",
        "vehicle.yaml",
        "seabed.yaml",
        "mission.yaml",
        "nav.yaml",
        "perception.yaml",
        "link_profiles.yaml",
        "scheduler.yaml",
    ]
    for cfg in expected_configs:
        data = load_yaml(cfg)
        assert isinstance(data, dict), f"Failed to load {cfg} as dictionary"


def test_nonexistent_config_raises_error() -> None:
    with pytest.raises(FileNotFoundError):
        get_config_path("completely_missing_file_123.yaml")
