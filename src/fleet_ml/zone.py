"""Protective-zone logic: is a person inside the zone, and what speed is allowed?

The state machine is deliberately conservative:
- a person in the zone means STOP immediately
- an out-of-distribution camera frame means SLOW (the detector may be blind)
- leaving STOP needs the zone to be clear for `clear_for_s` seconds (hysteresis), so a
  flickering detection cannot make the robot lurch back and forth
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import StrEnum

Point = tuple[float, float]


def point_in_polygon(p: Point, polygon: Sequence[Point]) -> bool:
    """Ray-casting test. Points exactly on an edge count as inside (safer)."""
    x, y = p
    n = len(polygon)
    if n < 3:
        raise ValueError("polygon needs at least 3 vertices")
    inside = False
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        # on-edge check
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        on_segment = min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)
        if abs(cross) < 1e-12 and on_segment:
            return True
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


class SafetyState(StrEnum):
    NORMAL = "normal"
    SLOW = "slow"
    STOP = "stop"


SPEED_SCALE = {SafetyState.NORMAL: 1.0, SafetyState.SLOW: 0.3, SafetyState.STOP: 0.0}


@dataclass
class SafetyStateMachine:
    clear_for_s: float = 1.0
    state: SafetyState = SafetyState.NORMAL
    _clear_since: float | None = field(default=None, repr=False)

    def update(self, t: float, person_in_zone: bool, ood: bool) -> float:
        """Advance to time t (seconds) and return the allowed speed scale in [0, 1]."""
        if person_in_zone:
            self.state = SafetyState.STOP
            self._clear_since = None
        elif self.state is SafetyState.STOP:
            if self._clear_since is None:
                self._clear_since = t
            if t - self._clear_since >= self.clear_for_s:
                self.state = SafetyState.SLOW if ood else SafetyState.NORMAL
                self._clear_since = None
        else:
            self.state = SafetyState.SLOW if ood else SafetyState.NORMAL
        return SPEED_SCALE[self.state]
