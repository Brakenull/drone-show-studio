import pytest

from stage1_designer.core.color_extractor import linear_to_srgb8


def test_black_and_white():
    assert linear_to_srgb8((0.0, 0.0, 0.0)) == (0, 0, 0)
    assert linear_to_srgb8((1.0, 1.0, 1.0)) == (255, 255, 255)


def test_mid_gray_linear_is_brighter_in_srgb():
    r, g, b = linear_to_srgb8((0.5, 0.5, 0.5))
    # Linear 0.5 should map above the naive 128 midpoint in sRGB gamma space.
    assert r == g == b
    assert 180 <= r <= 190


def test_clamps_out_of_range_inputs():
    assert linear_to_srgb8((-1.0, 2.0, 0.5)) == (0, 255, linear_to_srgb8((0.0, 0.0, 0.5))[2])


@pytest.mark.parametrize("channel", [0.0, 0.001, 0.0031308, 0.1, 0.5, 0.9, 1.0])
def test_monotonic_increasing(channel):
    lower = linear_to_srgb8((channel, 0.0, 0.0))[0]
    higher = linear_to_srgb8((min(channel + 0.05, 1.0), 0.0, 0.0))[0]
    assert higher >= lower
