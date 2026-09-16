import math

import numpy as np
import pytest

from stage1_designer.core.timeline_sampler import enu_transform


def test_zero_heading_is_identity():
    points = np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0]])
    result = enu_transform(points, 0.0)
    np.testing.assert_allclose(result, points, atol=1e-9)


def test_single_point_shape_preserved():
    point = np.array([1.0, 0.0, 0.0])
    result = enu_transform(point, 45.0)
    assert result.shape == (1, 3)


@pytest.mark.parametrize("heading_deg", [10.0, 45.0, 90.0, 180.0, 270.0, 359.0])
def test_rotation_matches_manual_formula(heading_deg):
    points = np.array([[3.0, -2.0, 1.5], [0.0, 0.0, 5.0]])
    result = enu_transform(points, heading_deg)

    theta = math.radians(heading_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    expected = np.array(
        [
            [
                p[0] * cos_t + p[1] * sin_t,
                -p[0] * sin_t + p[1] * cos_t,
                p[2],
            ]
            for p in points
        ]
    )
    np.testing.assert_allclose(result, expected, atol=1e-5)


def test_rotation_preserves_vector_norm():
    points = np.array([[3.0, 4.0, 0.0]])  # norm 5 in XY
    result = enu_transform(points, 37.0)
    xy_norm = math.hypot(result[0, 0], result[0, 1])
    assert xy_norm == pytest.approx(5.0, abs=1e-6)
    assert result[0, 2] == pytest.approx(0.0, abs=1e-9)


def test_z_axis_untouched():
    points = np.array([[1.0, 2.0, 42.0]])
    result = enu_transform(points, 123.0)
    assert result[0, 2] == pytest.approx(42.0, abs=1e-9)
