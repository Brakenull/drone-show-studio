"""Point-cloud sampling algorithms (spec section 3.1).

The geometry-only algorithms (`poisson_disk_surface_sample`, `volumetric_sample`)
take plain NumPy arrays / callables and have no `bpy` dependency, so they are
directly unit-testable. The `sample_object_*` wrappers adapt a live Blender
object to those pure functions and are only importable from inside Blender
(the `bpy`/`bmesh`/`mathutils` imports are local to those functions).
"""

from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

import numpy as np


def _ckdtree_cls():
    """Lazily import scipy.spatial.cKDTree with an actionable error message.

    Kept as a local import (rather than a module-level one) so this module can
    still be imported - e.g. for docs/tooling - in environments without scipy;
    the actual sampling functions below fail fast only when invoked.
    """
    try:
        from scipy.spatial import cKDTree
    except ImportError as exc:  # pragma: no cover - environment guard
        raise ImportError(
            "scipy is required for Poisson-disk / volumetric sampling "
            "(scipy.spatial.cKDTree). Install it into Blender's bundled Python, e.g.\n"
            '  "<Blender install dir>/<version>/python/bin/python.exe" -m pip install scipy'
        ) from exc
    return cKDTree


def _triangle_areas(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    tri = vertices[faces]
    v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
    cross = np.cross(v1 - v0, v2 - v0)
    return 0.5 * np.linalg.norm(cross, axis=1)


def poisson_disk_surface_sample(
    vertices: np.ndarray,
    faces: np.ndarray,
    min_dist: float,
    max_points: Optional[int] = None,
    seed: Optional[int] = None,
    max_consecutive_failures: int = 30,
) -> np.ndarray:
    """Bridson-style dart-throwing Poisson-disk sampling over a triangle mesh surface.

    Candidate points are drawn area-weighted across triangles (uniform barycentric
    sampling) and accepted only if their nearest neighbour among already-accepted
    points is >= `min_dist` (checked via `scipy.spatial.cKDTree`). Sampling stops
    once `max_points` is reached or after `max_consecutive_failures` rejected darts
    in a row (standard Bridson termination heuristic).
    """
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=int)
    if len(faces) == 0 or len(vertices) == 0:
        return np.zeros((0, 3), dtype=float)

    rng = np.random.default_rng(seed)
    tri = vertices[faces]
    v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
    areas = _triangle_areas(vertices, faces)
    total_area = float(areas.sum())
    if total_area <= 0.0:
        return np.zeros((0, 3), dtype=float)
    cum_area = np.cumsum(areas)

    def random_surface_point() -> np.ndarray:
        r = rng.random() * total_area
        idx = int(np.searchsorted(cum_area, r))
        idx = min(idx, len(areas) - 1)
        r1, r2 = rng.random(), rng.random()
        sqrt_r1 = math.sqrt(r1)
        u = 1.0 - sqrt_r1
        v = r2 * sqrt_r1
        w = 1.0 - u - v
        return u * v0[idx] + v * v1[idx] + w * v2[idx]

    cKDTree = _ckdtree_cls()
    points: list[np.ndarray] = []
    tree = None
    failures = 0
    rebuild_every = 25

    while max_points is None or len(points) < max_points:
        candidate = random_surface_point()
        if points:
            if tree is None or len(points) % rebuild_every == 0:
                tree = cKDTree(np.asarray(points))
            dist, _ = tree.query(candidate, k=1)
            if dist < min_dist:
                failures += 1
                if failures >= max_consecutive_failures:
                    break
                continue
        points.append(candidate)
        tree = None  # force rebuild on next query for correctness
        failures = 0

    return np.asarray(points, dtype=float) if points else np.zeros((0, 3), dtype=float)


def volumetric_sample(
    bounds_min: Tuple[float, float, float],
    bounds_max: Tuple[float, float, float],
    min_dist: float,
    inside_test_fn: Callable[[np.ndarray], bool],
    seed: Optional[int] = None,
    jitter_factor: float = 0.2,
) -> np.ndarray:
    """Jittered-grid + ray-cast parity volumetric sampling (spec section 3.1).

    1. Lay a 3D grid over the bounding box with cell size s = min_dist / sqrt(3).
    2. Jitter each cell center by delta in [-jitter_factor*s, jitter_factor*s].
    3. Keep samples for which `inside_test_fn` reports "inside" (the caller is
       expected to implement the ray-cast parity test against the real mesh).
    4. Greedily filter with a KD-tree so no pair violates `min_dist`.
    """
    rng = np.random.default_rng(seed)
    cell = min_dist / math.sqrt(3.0)

    bmin = np.asarray(bounds_min, dtype=float)
    bmax = np.asarray(bounds_max, dtype=float)
    dims = bmax - bmin
    counts = np.maximum(np.floor(dims / cell).astype(int), 1)

    candidates = []
    for i in range(counts[0]):
        for j in range(counts[1]):
            for k in range(counts[2]):
                center = bmin + (np.array([i, j, k], dtype=float) + 0.5) * cell
                jitter = rng.uniform(-jitter_factor * cell, jitter_factor * cell, size=3)
                candidates.append(center + jitter)

    if not candidates:
        return np.zeros((0, 3), dtype=float)

    candidates = np.asarray(candidates, dtype=float)
    inside_mask = np.fromiter(
        (bool(inside_test_fn(p)) for p in candidates), dtype=bool, count=len(candidates)
    )
    inside_points = candidates[inside_mask]
    if len(inside_points) == 0:
        return inside_points

    cKDTree = _ckdtree_cls()
    kept: list[np.ndarray] = []
    for p in inside_points:
        if kept:
            tree = cKDTree(np.asarray(kept))
            dist, _ = tree.query(p, k=1)
            if dist < min_dist:
                continue
        kept.append(p)

    return np.asarray(kept, dtype=float)


# --------------------------------------------------------------------------
# bpy-dependent adapters (only usable inside Blender)
# --------------------------------------------------------------------------

def sample_object_surface(obj, min_dist: float, max_points: Optional[int] = None, seed: Optional[int] = None) -> np.ndarray:
    """Poisson-disk sample the world-space surface of a Blender mesh object."""
    import bmesh
    import bpy  # noqa: F401  (kept for symmetry / future scene access)

    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()
    try:
        bm = bmesh.new()
        bm.from_mesh(mesh)
        bmesh.ops.triangulate(bm, faces=bm.faces)
        bm.verts.ensure_lookup_table()

        matrix_world = obj.matrix_world
        vertices = np.array(
            [matrix_world @ v.co for v in bm.verts], dtype=float
        )
        faces = np.array([[v.index for v in f.verts] for f in bm.faces], dtype=int)
        bm.free()
    finally:
        eval_obj.to_mesh_clear()

    return poisson_disk_surface_sample(vertices, faces, min_dist, max_points=max_points, seed=seed)


def sample_object_volume(obj, min_dist: float, seed: Optional[int] = None) -> np.ndarray:
    """Jittered-grid volumetric sample of a Blender mesh object's interior."""
    import bpy
    from mathutils.bvhtree import BVHTree
    from mathutils import Vector

    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    bvh = BVHTree.FromObject(eval_obj, depsgraph)

    corners = [Vector(c) for c in obj.bound_box]
    world_corners = [obj.matrix_world @ c for c in corners]
    xs = [c.x for c in world_corners]
    ys = [c.y for c in world_corners]
    zs = [c.z for c in world_corners]
    bounds_min = (min(xs), min(ys), min(zs))
    bounds_max = (max(xs), max(ys), max(zs))

    ray_direction = Vector((0.0, 0.0, 1.0))

    def inside_test(point: np.ndarray) -> bool:
        origin = Vector((float(point[0]), float(point[1]), float(point[2])))
        hits = 0
        current = origin
        # Walk repeated ray-casts along +Z counting surface crossings (parity test).
        while True:
            result = bvh.ray_cast(current, ray_direction)
            location = result[0]
            if location is None:
                break
            hits += 1
            current = location + ray_direction * 1e-5
        return hits % 2 == 1

    return volumetric_sample(bounds_min, bounds_max, min_dist, inside_test, seed=seed)
