import numpy as np
import pytest

from stage1_designer.core.ground import check_formations_ground, lowest_point_below


def test_counts_points_below_and_reports_depth():
    formations = [
        ("high", np.array([[0.0, 0.0, 10.0], [1.0, 0.0, 12.0]])),
        ("dips", np.array([[0.0, 0.0, -1.5], [1.0, 0.0, 3.0], [2.0, 0.0, -0.5]])),
        ("all parked", np.zeros((0, 3))),
    ]
    high, dips = check_formations_ground(formations, ground_z=0.0)
    assert (high.below, high.is_warning) == (0, False)
    assert high.depth_m == pytest.approx(-10.0)
    assert (dips.keyframe_index, dips.below, dips.is_warning) == (1, 2, True)
    assert dips.lowest_z == pytest.approx(-1.5) and dips.depth_m == pytest.approx(1.5)


def test_on_the_ground_is_not_below():
    (r,) = check_formations_ground([("landed", np.array([[0.0, 0.0, 2.0]]))], ground_z=2.0)
    assert not r.is_warning
    assert lowest_point_below(np.array([[0.0, 0.0, 2.0]]), 2.0) == 0.0


def test_raised_ground_flags_low_points():
    pts = np.array([[0.0, 0.0, 3.0], [0.0, 0.0, 6.0]])
    (r,) = check_formations_ground([("low", pts)], ground_z=4.0)
    assert r.below == 1 and r.depth_m == pytest.approx(1.0)
    assert lowest_point_below(pts, 4.0) == pytest.approx(1.0)
