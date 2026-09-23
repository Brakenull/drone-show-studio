import numpy as np
import pytest

from stage3_helpers import make_contract, make_segment
from stage3_simulation_packer.warp_sim.loaders.arrow_loader import parse_contract_dict
from stage3_simulation_packer.warp_sim.loaders.spline_evaluator import (
    bspline_to_power_spans,
    build_piecewise,
    evaluate_colors_numpy,
    evaluate_numpy,
)


def cox_de_boor(knots, cps, degree, u):
    """Independent textbook reference (recursive basis), not the code under test."""
    def basis(i, p, x):
        if p == 0:
            last = knots[i + 1] == knots[-1] and x == knots[-1]
            return 1.0 if (knots[i] <= x < knots[i + 1]) or (last and knots[i] < knots[i + 1]) else 0.0
        out = 0.0
        if knots[i + p] > knots[i]:
            out += (x - knots[i]) / (knots[i + p] - knots[i]) * basis(i, p - 1, x)
        if knots[i + p + 1] > knots[i + 1]:
            out += (knots[i + p + 1] - x) / (knots[i + p + 1] - knots[i + 1]) * basis(i + 1, p - 1, x)
        return out
    return sum(basis(i, degree, u) * cps[i] for i in range(len(cps)))


def random_segment(rng, start, end, n_cp=9, degree=5):
    cps = rng.uniform(-20, 20, (n_cp, 3))
    duration = end - start
    interior = np.sort(rng.uniform(0.05, 0.95, n_cp - degree - 1)) * duration  # non-uniform knots
    seg = make_segment(0, start, end, cps, degree=degree)
    seg["knot_vector"] = [0.0] * (degree + 1) + interior.tolist() + [duration] * (degree + 1)
    return seg


def test_power_basis_matches_cox_de_boor_on_nonuniform_knots():
    rng = np.random.default_rng(3)
    seg = random_segment(rng, 0.0, 7.0)
    knots, cps = np.asarray(seg["knot_vector"]), np.asarray(seg["control_points"])
    show = parse_contract_dict(make_contract([[seg]]))
    pw = build_piecewise(show)
    times = np.linspace(0.0, 7.0, 57)
    pos, vel, acc = evaluate_numpy(pw, times)
    for k, u in enumerate(times):
        np.testing.assert_allclose(pos[0, k], cox_de_boor(knots, cps, 5, u), atol=1e-9)
    # derivatives vs central finite differences of the reference
    h = 1e-5
    for u in (0.5, 3.3, 6.2):
        ref_v = (cox_de_boor(knots, cps, 5, u + h) - cox_de_boor(knots, cps, 5, u - h)) / (2 * h)
        ref_a = (cox_de_boor(knots, cps, 5, u + h) - 2 * cox_de_boor(knots, cps, 5, u)
                 + cox_de_boor(knots, cps, 5, u - h)) / h**2
        k = int(np.argmin(np.abs(times - u)))
        v, a, _ = (evaluate_numpy(pw, np.array([u]))[i][0, 0] for i in (1, 2, 0))
        np.testing.assert_allclose(v, ref_v, atol=1e-4)
        np.testing.assert_allclose(a, ref_a, atol=2e-2)


def test_repeated_interior_knots_are_skipped():
    cps = np.arange(27, dtype=float).reshape(9, 3)
    knots = np.array([0.0] * 6 + [1.0, 1.0, 2.0] + [3.0] * 6)
    spans = bspline_to_power_spans(knots, cps)
    assert [(a, b) for a, b, _ in spans] == [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0)]


def test_timeline_holds_before_in_gaps_and_after():
    a = make_segment(0, 2.0, 4.0, [[0, 0, 0]] * 3 + [[4, 0, 0]] * 3)
    b = make_segment(1, 6.0, 8.0, [[10, 0, 0]] * 3 + [[10, 5, 0]] * 3)
    pw = build_piecewise(parse_contract_dict(make_contract([[a, b]])))
    pos, vel, _ = evaluate_numpy(pw, np.array([0.0, 3.0, 5.0, 7.0, 99.0]))
    np.testing.assert_allclose(pos[0, 0], [0, 0, 0])
    np.testing.assert_allclose(vel[0, 0], 0.0)
    assert vel[0, 1, 0] > 0
    np.testing.assert_allclose(pos[0, 2], [4, 0, 0])      # gap: hold end of segment a
    np.testing.assert_allclose(vel[0, 2], 0.0)
    np.testing.assert_allclose(pos[0, 4], [10, 5, 0])     # after the show
    np.testing.assert_allclose(vel[0, 4], 0.0)


def test_color_rules_lerp_round_step_and_clamp():
    seg = make_segment(0, 0.0, 10.0, [[0, 0, 0]] * 6, colors=[
        (2.0, [0, 0, 0]), (4.0, [255, 0, 10]), (4.0, [0, 255, 0]), (8.0, [0, 255, 200])])
    pw = build_piecewise(parse_contract_dict(make_contract([[seg]])))
    c = evaluate_colors_numpy(pw, np.array([0.0, 3.0, 3.99, 4.0, 6.0, 10.0]))[0]
    assert c[0].tolist() == [0, 0, 0]              # clamp-to-edge before the first keyframe
    assert c[1].tolist() == [128, 0, 5]            # 127.5 -> 128, 5.0 -> 5
    assert c[2].tolist() == [254, 0, 10]
    assert c[3].tolist() == [0, 255, 0]            # same-instant keyframes: step, later wins
    assert c[4].tolist() == [0, 255, 100]
    assert c[5].tolist() == [0, 255, 200]          # clamp-to-edge after the last keyframe


def test_color_keyframes_merge_across_segments():
    a = make_segment(0, 0.0, 5.0, [[0, 0, 0]] * 6, colors=[(0.0, [0, 0, 0]), (5.0, [100, 0, 0])])
    b = make_segment(1, 5.0, 10.0, [[0, 0, 0]] * 6, colors=[(5.0, [0, 0, 100]), (10.0, [0, 0, 200])])
    pw = build_piecewise(parse_contract_dict(make_contract([[a, b]])))
    c = evaluate_colors_numpy(pw, np.array([2.5, 5.0, 7.5]))[0]
    assert c.tolist() == [[50, 0, 0], [0, 0, 100], [0, 0, 150]]


def test_warp_kernels_match_numpy_reference():
    wp = pytest.importorskip("warp")
    from stage3_simulation_packer.warp_sim.loaders.spline_evaluator import WarpTrajectoryBuffers

    rng = np.random.default_rng(11)
    drones = [[random_segment(rng, 0.0, 5.0), random_segment(rng, 5.0, 9.0) | {"segment_index": 1}]
              for _ in range(6)]
    pw = build_piecewise(parse_contract_dict(make_contract(drones)))
    times = np.linspace(-1.0, 10.0, 45)
    ref_p, ref_v, _ = evaluate_numpy(pw, times)
    buffers = WarpTrajectoryBuffers(pw, device="cpu")
    p, v = buffers.sample_grid(times)
    np.testing.assert_allclose(p, ref_p, atol=2e-4)   # float32 on the device
    np.testing.assert_allclose(v, ref_v, atol=2e-3)

    n = pw.fleet_size
    arrays = [wp.zeros(n, dtype=wp.vec3, device="cpu") for _ in range(3)] + [wp.zeros(n, dtype=float, device="cpu")]
    buffers.launch_reference(2.5, *arrays)
    colors = evaluate_colors_numpy(pw, np.array([2.5]))[:, 0].astype(float)
    np.testing.assert_allclose(arrays[3].numpy(), colors.sum(axis=1) / 765.0, atol=3e-3)
