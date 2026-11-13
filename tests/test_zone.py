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
    assert sm.update(0.0, person_in_zone=False, slow=False) == 1.0
    assert sm.update(0.1, person_in_zone=True, slow=False) == 0.0
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


def test_slow_holds_through_flicker_then_releases():
    sm = SafetyStateMachine(slow_hold_s=0.5)
    assert sm.update(0.0, False, True) == pytest.approx(0.3)
    assert sm.update(0.1, False, False) == pytest.approx(0.3)   # detection dropped for a frame
    assert sm.update(0.2, False, True) == pytest.approx(0.3)
    assert sm.update(0.6, False, False) == pytest.approx(0.3)   # 0.4 s since last slow reason
    assert sm.update(0.75, False, False) == 1.0                 # 0.55 s clear -> normal


def test_leaving_stop_goes_to_slow_if_slow_reason_is_recent():
    sm = SafetyStateMachine(clear_for_s=1.0, slow_hold_s=0.5)
    assert sm.update(0.0, True, False) == 0.0
    assert sm.update(0.5, False, False) == 0.0                  # stop zone clear since 0.5 s
    assert sm.update(1.2, False, True) == 0.0                   # person now in warning zone
    assert sm.update(1.5, False, False) == pytest.approx(0.3)   # clear 1.0 s, slow 0.3 s ago


def test_energy_score_higher_for_uncertain_logits():
    assert energy_score(np.array([[10.0, 0, 0]]))[0] < energy_score(np.array([[0.1, 0, 0.05]]))[0]


def test_auroc_extremes_and_ties():
    from fleet_ml.ood import auroc

    assert auroc([0, 1, 2], [3, 4]) == 1.0
    assert auroc([3, 4], [0, 1]) == 0.0
    assert auroc([1, 1], [1, 1]) == 0.5


def test_mahalanobis_flags_shifted_samples_and_roundtrips():
    from fleet_ml.ood import Mahalanobis, auroc

    rng = np.random.default_rng(0)
    train = rng.normal(0, 1, (60, 100))          # fewer samples than dims: needs PCA+shrinkage
    m = Mahalanobis(n_components=10).fit(train)
    ind, ood = rng.normal(0, 1, (40, 100)), rng.normal(0, 1, (40, 100)) + 1.5
    assert auroc(m.score(ind), m.score(ood)) > 0.95
    m2 = Mahalanobis.from_dict(m.to_dict())
    assert np.allclose(m.score(ind), m2.score(ind))


def test_corruptions_change_image_statistics():
    from fleet_ml.ood import CORRUPTIONS, corrupt, image_stats

    rng = np.random.default_rng(1)
    img = rng.integers(0, 256, (60, 80, 3), dtype=np.uint8)
    base = image_stats(img)
    for kind in CORRUPTIONS:
        out = corrupt(img, kind)
        assert out.shape == img.shape and out.dtype == np.uint8
        assert not np.allclose(image_stats(out), base), kind
    with pytest.raises(ValueError):
        corrupt(img, "rain")
