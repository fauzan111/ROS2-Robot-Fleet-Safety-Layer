"""Build a Nav2 params file from upstream defaults plus a small overrides file."""

from __future__ import annotations

import copy
import tempfile
from pathlib import Path

import yaml


def deep_merge(base: dict, overrides: dict) -> dict:
    """Recursively merge overrides into a copy of base (overrides win)."""
    out = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def write_merged_params(upstream: str | Path, overrides: str | Path) -> str:
    merged = deep_merge(yaml.safe_load(Path(upstream).read_text()),
                        yaml.safe_load(Path(overrides).read_text()) or {})
    with tempfile.NamedTemporaryFile("w", prefix="fleet_nav2_", suffix=".yaml",
                                     delete=False) as f:
        yaml.safe_dump(merged, f, sort_keys=False)
        return f.name
