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


def _write_temp_yaml(data: dict) -> str:
    with tempfile.NamedTemporaryFile("w", prefix="fleet_nav2_", suffix=".yaml",
                                     delete=False) as f:
        yaml.safe_dump(data, f, sort_keys=False)
        return f.name


def write_merged_params(upstream: str | Path, overrides: str | Path) -> str:
    merged = deep_merge(yaml.safe_load(Path(upstream).read_text()),
                        yaml.safe_load(Path(overrides).read_text()) or {})
    return _write_temp_yaml(merged)


# Absolute topic prefixes in the upstream multi-robot file that must follow the namespace.
_NAMESPACED_PREFIXES = ("/local_costmap/", "/global_costmap/")


def _relativize_topics(node: object) -> object:
    if isinstance(node, dict):
        return {k: _relativize_topics(v) for k, v in node.items()}
    if isinstance(node, list):
        return [_relativize_topics(v) for v in node]
    if isinstance(node, str) and node.startswith(_NAMESPACED_PREFIXES):
        return node[1:]
    return node


def fleet_params(multi: dict, single: dict, overrides: dict, spawn: dict) -> dict:
    """Nav2 params for one namespaced robot.

    multi: upstream nav2_multirobot_params_all.yaml (DWB controller, <robot_namespace> topics).
    single: upstream nav2_params.yaml, used only for nodes the bringup starts but the
            multi-robot file omits (route_server in Jazzy), so lifecycle startup won't abort.
    spawn: {x, y, yaw}; AMCL starts there so the global costmap gets map->base_link.
    """
    out = copy.deepcopy(multi)
    for node in single.keys() - multi.keys():
        out[node] = copy.deepcopy(single[node])
    out = _relativize_topics(out)
    out = deep_merge(out, overrides or {})
    pose = {"x": float(spawn["x"]), "y": float(spawn["y"]), "z": 0.0,
            "yaw": float(spawn.get("yaw", 0.0))}
    return deep_merge(out, {"amcl": {"ros__parameters": {"set_initial_pose": True,
                                                         "initial_pose": pose}}})


def write_fleet_params(multi: str | Path, single: str | Path, overrides: str | Path,
                       spawn: dict) -> str:
    def load(p: str | Path) -> dict:
        return yaml.safe_load(Path(p).read_text()) or {}

    return _write_temp_yaml(fleet_params(load(multi), load(single), load(overrides), spawn))
