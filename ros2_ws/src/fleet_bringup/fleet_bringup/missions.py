"""Mission files, single-robot and fleet formats (no ROS dependency).

Single robot:  {initial_pose: {x, y, yaw}, goals: [...], timeout_s}
Fleet:         {robots: {robot1: {spawn: {x, y, yaw}, goals: [...]}, ...}, timeout_s}
Each goal:     {name, x, y, yaw}
"""

from __future__ import annotations


def mission_for(cfg: dict, robot: str = "") -> tuple[dict, list[dict]]:
    """Return (initial_pose, goals) for one robot, or for the single-robot file if robot is ''."""
    if robot:
        robots = cfg.get("robots", {})
        if robot not in robots:
            raise KeyError(f"robot '{robot}' not in mission file (have: {sorted(robots)})")
        entry = robots[robot]
        return entry["spawn"], entry["goals"]
    return cfg["initial_pose"], cfg["goals"]
