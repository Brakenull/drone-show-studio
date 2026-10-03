import numpy as np
import pytest

from stage1_designer.config import LEG_MODE_AUTO, LEG_MODE_TARGET
from stage1_designer.core.holding_area import compute_holding_positions
from stage1_designer.core.kinematic_validator import compute_min_safe_duration
from stage1_designer.core.show_legs import (
    estimate_leg,
    estimate_max_travel,
    launch_wave_span,
    target_duration,
)

CENTER = (0.0, -30.0, 5.0)
SIZE = (40.0, 10.0)


def test_max_travel_uses_the_best_matching_not_the_index_order():
    a = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    b = np.array([[10.0, 0.0, 1.0], [0.0, 0.0, 1.0]])  # same points, swapped order
    assert estimate_max_travel(a, b) == pytest.approx(1.0)


def test_max_travel_rejects_mismatched_sets():
    with pytest.raises(ValueError):
        estimate_max_travel(np.zeros((2, 3)), np.zeros((3, 3)))


def test_estimate_uses_the_auto_fix_duration_rule():
    slots = compute_holding_positions(4, CENTER, SIZE, 15.0, 2.0)
    formation = slots + np.array([0.0, 50.0, 20.0])  # every drone flies the same 53.9 m
    est = estimate_leg(slots, formation, v_max=6.0)
    assert est.d_max_m == pytest.approx(np.hypot(50.0, 20.0))
    assert est.min_duration_sec == pytest.approx(compute_min_safe_duration(est.d_max_m, 6.0))
    assert est.wave_span_sec == 0.0 and est.total_sec == est.min_duration_sec


def test_wave_span_counts_rows_within_a_layer():
    # 40 x 10 m at 2 m -> 21 columns x 6 rows = 126 slots per layer.
    assert launch_wave_span(21, CENTER, SIZE, 15.0, 2.0, wave_delay_s=1.2) == 0.0  # one row
    assert launch_wave_span(22, CENTER, SIZE, 15.0, 2.0, wave_delay_s=1.2) == pytest.approx(1.2)
    assert launch_wave_span(300, CENTER, SIZE, 15.0, 2.0, wave_delay_s=1.2) == pytest.approx(5 * 1.2)  # full layers: rows 0..5
    assert launch_wave_span(300, CENTER, SIZE, 15.0, 2.0, wave_delay_s=0.0) == 0.0


def test_target_duration_export_values():
    assert target_duration(LEG_MODE_AUTO, 30.0) is None
    assert target_duration(LEG_MODE_TARGET, 42) == 42.0


def test_launch_wave_span_staggered_layers():
    # Odd layers have 5 rows instead of 6: the deepest row is still row 5.
    on_ground = (0.0, -30.0, 0.0)
    assert launch_wave_span(126, on_ground, SIZE, 15.0, 2.0, 4.0, True, wave_delay_s=1.0) == pytest.approx(5.0)
    assert launch_wave_span(30, on_ground, SIZE, 15.0, 2.0, 4.0, True, wave_delay_s=1.0) == pytest.approx(1.0)
