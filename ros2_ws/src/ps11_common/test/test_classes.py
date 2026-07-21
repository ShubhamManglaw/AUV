"""Tests for perception classes database (§10.1)."""

import pytest
from ps11_common.classes import (
    ClassDatabase,
    ClassInfo,
    get_class_by_id,
    get_class_by_name,
)


def test_classes_lookup() -> None:
    db = ClassDatabase()
    assert len(db) == 4

    debris = db.get_by_id(0)
    assert isinstance(debris, ClassInfo)
    assert debris.name == "debris"
    assert debris.priority == 1.0
    assert debris.color == "#E4572E"

    starfish = db.get_by_name("starfish")
    assert starfish.id == 1
    assert starfish.priority == 0.3
    assert starfish.color == "#F2C14E"

    sea_urchin = db.get_by_id(2)
    assert sea_urchin.name == "sea_urchin"

    scallop = db.get_by_name("scallop")
    assert scallop.id == 3


def test_unknown_id_raises_error() -> None:
    db = ClassDatabase()
    with pytest.raises(KeyError, match="Unknown class id"):
        db.get_by_id(99)

    with pytest.raises(KeyError, match="Unknown class name"):
        db.get_by_name("nonexistent_creature")


def test_convenience_functions() -> None:
    c0 = get_class_by_id(0)
    assert c0.name == "debris"

    c1 = get_class_by_name("starfish")
    assert c1.id == 1
