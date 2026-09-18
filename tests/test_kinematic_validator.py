import numpy as np
import pytest

from stage1_designer.core.kinematic_validator import (
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARNING,
    auto_fix_times,
    classify_status,
    compute_min_safe_duration,
    evaluate_transitions,
    has_error,
    required_velocity,
)

V_MAX = 6.0


def test_required_velocity_basic():
    assert required_velocity(d_max=12.0, delta_t=4.0) == pytest.approx(3.0)


def test_required_velocity_zero_delta_t_with_movement_is_infinite():
    assert required_velocity(d_max=5.0, delta_t=0.0) == float("inf")


def test_required_velocity_zero_delta_t_no_movement_is_zero():
    assert required_velocity(d_max=0.0, delta_t=0.0) == 0.0


@pytest.mark.parametrize(
    "v_req,expected",
    # Thresholds are inclusive on the safer side (doc section 3.5: OK is
    # v_req <= 0.75*v_max, WARNING is up to and including v_req == v_max).
    [(3.0, STATUS_OK), (4.5, STATUS_OK), (4.501, STATUS_WARNING), (6.0, STATUS_WARNING), (6.001, STATUS_ERROR), (10.0, STATUS_ERROR)],
)
def test_classify_status_thresholds(v_req, expected):
    assert classify_status(v_req, V_MAX) == expected


def test_evaluate_transitions_matches_doc_example():
    # docs/1-phase_1.md section 3.5's own example: 73 m in 2 s.
    times = [0.0, 2.0]
    p0 = np.array([[0.0, 0.0, 0.0]])
    p1 = np.array([[73.0, 0.0, 0.0]])
    result = evaluate_transitions(times, [p0, p1], V_MAX)
    assert len(result) == 1
    t = result[0]
    assert t.d_max == pytest.approx(73.0)
    assert t.v_req == pytest.approx(36.5)
    assert t.status == STATUS_ERROR
    assert has_error(result)


def test_evaluate_transitions_uses_max_not_mean_displacement():
    times = [0.0, 10.0]
    p0 = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    p1 = np.array([[1.0, 0.0, 0.0], [50.0, 0.0, 0.0]])  # one point barely moves, one moves far
    result = evaluate_transitions(times, [p0, p1], V_MAX)
    assert result[0].d_max == pytest.approx(50.0)


def test_evaluate_transitions_mismatched_point_counts_raises():
    with pytest.raises(ValueError):
        evaluate_transitions([0.0, 1.0], [np.zeros((2, 3)), np.zeros((3, 3))], V_MAX)


def test_compute_min_safe_duration_zero_distance_is_zero():
    assert compute_min_safe_duration(0.0, V_MAX) == 0.0


def test_compute_min_safe_duration_matches_formula():
    d_max = 39.05
    expected = 1.875 * d_max / V_MAX * 1.25
    assert compute_min_safe_duration(d_max, V_MAX, slack_fraction=0.25) == pytest.approx(expected)


def test_auto_fix_times_stretches_unsafe_transition():
    times = [0.0, 2.0]
    p0 = np.array([[0.0, 0.0, 0.0]])
    p1 = np.array([[73.0, 0.0, 0.0]])
    fixed = auto_fix_times(times, [p0, p1], V_MAX)
    assert fixed[0] == 0.0
    safe_delta = compute_min_safe_duration(73.0, V_MAX)
    assert fixed[1] == pytest.approx(safe_delta)
    # The fixed timeline must no longer trigger any ERROR status.
    assert not has_error(evaluate_transitions(fixed, [p0, p1], V_MAX))


def test_auto_fix_times_never_compresses_already_safe_transitions():
    times = [0.0, 100.0]  # already generously slow
    p0 = np.array([[0.0, 0.0, 0.0]])
    p1 = np.array([[1.0, 0.0, 0.0]])
    fixed = auto_fix_times(times, [p0, p1], V_MAX)
    assert fixed[1] == pytest.approx(100.0)


def test_auto_fix_times_cascades_offset_to_later_keyframes():
    # First transition is unsafe and gets stretched; the second transition's
    # nominal spacing (5s) must be preserved relative to the *fixed* time of
    # keyframe 1, not silently swallowed.
    times = [0.0, 2.0, 7.0]
    p0 = np.array([[0.0, 0.0, 0.0]])
    p1 = np.array([[73.0, 0.0, 0.0]])
    p2 = np.array([[74.0, 0.0, 0.0]])  # second transition is trivially safe
    fixed = auto_fix_times(times, [p0, p1, p2], V_MAX)
    assert fixed[2] == pytest.approx(fixed[1] + 5.0)


def test_auto_fix_times_empty_input():
    assert auto_fix_times([], [], V_MAX) == []
