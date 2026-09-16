import itertools

import numpy as np
import pytest

scipy = pytest.importorskip("scipy", reason="scipy is required for sampler algorithms (see README/CLAUDE notes)")

from stage1_designer.core.sampler import poisson_disk_surface_sample, volumetric_sample


def _min_pairwise_distance(points: np.ndarray) -> float:
    dists = [
        np.linalg.norm(a - b) for a, b in itertools.combinations(points, 2)
    ]
    return min(dists) if dists else float("inf")


def _unit_cube_mesh():
    vertices = np.array(
        [
            [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
            [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
        ],
        dtype=float,
    ) * 10.0  # scale to a 10m cube so min_dist=1.5 has room
    faces = np.array(
        [
            [0, 1, 2], [0, 2, 3],  # bottom
            [4, 5, 6], [4, 6, 7],  # top
            [0, 1, 5], [0, 5, 4],  # front
            [2, 3, 7], [2, 7, 6],  # back
            [1, 2, 6], [1, 6, 5],  # right
            [0, 3, 7], [0, 7, 4],  # left
        ]
    )
    return vertices, faces


def test_surface_sample_respects_min_distance():
    vertices, faces = _unit_cube_mesh()
    points = poisson_disk_surface_sample(vertices, faces, min_dist=1.5, seed=42)
    assert len(points) > 0
    assert _min_pairwise_distance(points) >= 1.5 - 1e-9


def test_surface_sample_respects_max_points():
    vertices, faces = _unit_cube_mesh()
    points = poisson_disk_surface_sample(vertices, faces, min_dist=0.5, max_points=10, seed=1)
    assert len(points) <= 10


def test_surface_sample_empty_mesh_returns_empty():
    points = poisson_disk_surface_sample(np.zeros((0, 3)), np.zeros((0, 3), dtype=int), min_dist=1.0)
    assert points.shape == (0, 3)


def test_volumetric_sample_all_inside_respects_min_distance():
    def always_inside(_p):
        return True

    points = volumetric_sample(
        bounds_min=(0.0, 0.0, 0.0),
        bounds_max=(10.0, 10.0, 10.0),
        min_dist=1.5,
        inside_test_fn=always_inside,
        seed=7,
    )
    assert len(points) > 0
    assert _min_pairwise_distance(points) >= 1.5 - 1e-9


def test_volumetric_sample_all_outside_returns_empty():
    def never_inside(_p):
        return False

    points = volumetric_sample(
        bounds_min=(0.0, 0.0, 0.0),
        bounds_max=(5.0, 5.0, 5.0),
        min_dist=1.5,
        inside_test_fn=never_inside,
        seed=3,
    )
    assert points.shape == (0, 3)


def test_volumetric_sample_half_space_filter():
    def right_half_only(p):
        return p[0] >= 5.0

    points = volumetric_sample(
        bounds_min=(0.0, 0.0, 0.0),
        bounds_max=(10.0, 5.0, 5.0),
        min_dist=1.5,
        inside_test_fn=right_half_only,
        seed=11,
    )
    assert len(points) > 0
    assert np.all(points[:, 0] >= 5.0 - 1e-6)
