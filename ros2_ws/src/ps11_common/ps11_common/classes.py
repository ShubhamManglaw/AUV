"""Single source of truth for perception classes in PS11 AUV (§10.1)."""

from dataclasses import dataclass
from pathlib import Path

from ps11_common.params import load_yaml


@dataclass(frozen=True)
class ClassInfo:
    """Class metadata entry."""

    id: int
    name: str
    priority: float
    color: str


class ClassDatabase:
    """Manages class definitions loaded from classes.yaml."""

    def __init__(self, config_path: str | Path | None = None) -> None:
        raw = load_yaml(config_path or "classes.yaml")
        entries = raw.get("classes", [])

        self._by_id: dict[int, ClassInfo] = {}
        self._by_name: dict[str, ClassInfo] = {}

        for item in entries:
            c = ClassInfo(
                id=int(item["id"]),
                name=str(item["name"]),
                priority=float(item["priority"]),
                color=str(item["color"]),
            )
            if c.id in self._by_id:
                raise ValueError(f"Duplicate class id: {c.id}")
            if c.name in self._by_name:
                raise ValueError(f"Duplicate class name: {c.name}")
            self._by_id[c.id] = c
            self._by_name[c.name] = c

    def get_by_id(self, class_id: int) -> ClassInfo:
        """Look up class by integer ID. Raises KeyError if unknown."""
        if class_id not in self._by_id:
            raise KeyError(
                f"Unknown class id: {class_id}. Known IDs: {list(self._by_id.keys())}"
            )
        return self._by_id[class_id]

    def get_by_name(self, name: str) -> ClassInfo:
        """Look up class by string name. Raises KeyError if unknown."""
        if name not in self._by_name:
            raise KeyError(
                f"Unknown class name: '{name}'. Known names: {list(self._by_name.keys())}"
            )
        return self._by_name[name]

    def all_classes(self) -> list[ClassInfo]:
        """Return all classes ordered by ID."""
        return [self._by_id[k] for k in sorted(self._by_id.keys())]

    def __len__(self) -> int:
        return len(self._by_id)

    def __contains__(self, item: int | str) -> bool:
        if isinstance(item, int):
            return item in self._by_id
        return item in self._by_name


_DEFAULT_DB: ClassDatabase | None = None


def get_class_db(config_path: str | Path | None = None) -> ClassDatabase:
    """Return the singleton or custom ClassDatabase."""
    global _DEFAULT_DB
    if config_path is not None:
        return ClassDatabase(config_path)
    if _DEFAULT_DB is None:
        _DEFAULT_DB = ClassDatabase()
    return _DEFAULT_DB


def get_class_by_id(class_id: int) -> ClassInfo:
    """Convenience helper to look up a class by ID."""
    return get_class_db().get_by_id(class_id)


def get_class_by_name(name: str) -> ClassInfo:
    """Convenience helper to look up a class by name."""
    return get_class_db().get_by_name(name)
