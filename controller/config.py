"""Load and merge the config files described in config/README.md.

The seven YAML files are merged into one flat mapping keyed by their top-level
sections (``rover``, ``bed``, ``safety``, ...).  Nothing here validates the
values: that is ``controller.startup_checks``, which runs against the merged
mapping this module produces.

Values are addressed by dotted path (``"rover.drive.omega_max_deg_s"``) so that
an override in a test or a ``*.local.yaml`` reads the same as the invariant
message that rejects it.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"

#: Every file config/README.md documents.  Loading is all-or-nothing — a
#: missing file is a broken checkout, not a reason to fall back to defaults.
CONFIG_FILES: tuple[str, ...] = (
    "rover.yaml",
    "safety.yaml",
    "perception.yaml",
    "control.yaml",
    "simulation.yaml",
    "camera.yaml",
    "development.yaml",
)

_MISSING = object()


class ConfigError(Exception):
    """The config files could not be read or merged at all."""


def load_config(config_dir: Path | str | None = None) -> dict[str, Any]:
    """Read every file in :data:`CONFIG_FILES` and merge their top-level keys."""
    directory = Path(config_dir) if config_dir is not None else CONFIG_DIR
    merged: dict[str, Any] = {}
    origin: dict[str, str] = {}

    for name in CONFIG_FILES:
        path = directory / name
        if not path.is_file():
            raise ConfigError(f"missing config file: {path}")

        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ConfigError(f"{path} must contain a mapping at the top level")

        for key, value in loaded.items():
            if key in merged:
                raise ConfigError(
                    f"top-level key {key!r} is declared in both "
                    f"{origin[key]} and {name} — each section has exactly one owner"
                )
            merged[key] = value
            origin[key] = name

    return merged


def get(config: dict[str, Any], dotted: str, default: Any = _MISSING) -> Any:
    """Read ``config`` by dotted path, e.g. ``"rover.drive.omega_max_deg_s"``."""
    node: Any = config
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            if default is not _MISSING:
                return default
            raise KeyError(f"no config value at {dotted!r}")
        node = node[part]
    return node


def with_overrides(config: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    """Return a deep copy of ``config`` with the dotted paths in ``overrides`` replaced.

    Every path must already exist.  An override that creates a key is almost
    always a typo, and a typo that silently does nothing turns an invariant test
    into a test that proves nothing.
    """
    patched = copy.deepcopy(config)

    for dotted, value in overrides.items():
        *parents, leaf = dotted.split(".")
        node: Any = patched
        for part in parents:
            if not isinstance(node, dict) or part not in node:
                raise KeyError(f"no config value at {dotted!r}")
            node = node[part]
        if not isinstance(node, dict) or leaf not in node:
            raise KeyError(f"no config value at {dotted!r}")
        node[leaf] = value

    return patched
