import numpy as np
import pytest

from fleet_ml.ood import energy_score
from fleet_ml.zone import SafetyState, SafetyStateMachine, point_in_polygon

SQUARE = [(0, 0), (2, 0), (2, 2), (0, 2)]


def test_point_in_polygon():
    assert point_in_polygon((1, 1), SQUARE)
    assert not point_in_polygon((3, 1), SQUARE)
    assert point_in_polygon((2, 1), SQUARE)  # on edge counts as inside
    with pytest.raises(ValueError):
        point_in_polygon((0, 0), [(0, 0), (1, 1)])


def test_person_stops_immediately():
    sm = SafetyStateMachine()
    assert sm.update(0.0, person_in_zone=False, ood=False) == 1.0
    assert sm.update(0.1, person_in_zone=True, ood=False) == 0.0
    assert sm.state is SafetyState.STOP


def test_resume_needs_clear_zone_for_hysteresis_time():
    sm = SafetyStateMachine(clear_for_s=1.0)
    sm.update(0.0, True, False)
    assert sm.update(0.5, False, False) == 0.0   # clear, but not long enough
    assert sm.update(0.9, True, False) == 0.0    # person flickers back: timer resets
    assert sm.update(1.5, False, False) == 0.0
    assert sm.update(2.6, False, False) == 1.0   # clear for >= 1 s


def test_ood_slows_down():
    sm = SafetyStateMachine()
    assert sm.update(0.0, False, True) == pytest.approx(0.3)


def test_energy_score_higher_for_uncertain_logits():
    assert energy_score(np.array([[10.0, 0, 0]]))[0] < energy_score(np.array([[0.1, 0, 0.05]]))[0]
