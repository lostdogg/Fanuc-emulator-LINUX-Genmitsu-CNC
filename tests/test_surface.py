import math

import pytest

from funuc_emulator.conversational import AGEError
from funuc_emulator.surface import (
    ParametricSurface,
    WorkCoordinateTransform,
    adaptive_stepover,
    cutter_location,
    drop_cutter_height,
    minimum_surface_clearance,
    sample_chordal,
    scallop_height,
    sampled_surface_distance,
    surface_normal,
    surface_toolpath,
)


def plane():
    return ParametricSurface(
        lambda u, v: (u, v, 0.3 * u + 0.2 * v),
        lambda _u, _v: (1, 0, 0.3),
        lambda _u, _v: (0, 1, 0.2),
    )


def close_vec(actual, expected, tol=1e-6):
    assert actual == pytest.approx(expected, abs=tol)


def test_surface_normal_and_cutter_offsets():
    surface = plane()
    normal = surface_normal(surface, 0.5, 0.5)
    expected = (-0.3, -0.2, 1)
    length = math.sqrt(sum(v * v for v in expected))
    close_vec(normal, tuple(v / length for v in expected))
    contact = surface.point(0.5, 0.5)
    close_vec(cutter_location(contact, (0, 0, 1), 2), (0.5, 0.5, 2.25))
    close_vec(cutter_location(contact, (0, 0, 1), 2, "flat"), contact)
    close_vec(
        cutter_location(contact, (0, 0, 1), 3, "toroid", minor_radius=1),
        (0.5, 0.5, 1.25),
    )
    with pytest.raises(AGEError):
        cutter_location(contact, (0, 0, 0), 1)


def test_wcs_transform_surface_toolpath_and_chord_subdivision():
    transform = WorkCoordinateTransform((
        (0, -1, 0, 10),
        (1, 0, 0, 20),
        (0, 0, 1, 30),
        (0, 0, 0, 1),
    ))
    close_vec(transform.apply((1, 2, 3)), (8, 21, 33))
    curve = lambda t: (t, t * t, 0)
    points = sample_chordal(curve, 0.002)
    assert len(points) > 2
    for a, b in zip(points, points[1:]):
        t = (a[0] + b[0]) / 2
        midpoint_curve = curve(t)
        chord_mid = tuple((a[i] + b[i]) / 2 for i in range(3))
        assert math.dist(midpoint_curve, chord_mid) <= 0.002
    path = surface_toolpath(
        plane(), lambda t: (t, 0.5), 0.01,
        cutter="ball", radius=0.1, transform=transform,
    )
    assert len(path) >= 2
    assert path[0][0] > 9
    assert path[0][2] > 30


def test_scallop_clearance_and_drop_cutter_multi_surface():
    flat_surface = ParametricSurface(lambda u, v: (u, v, 0.0))
    upper_surface = ParametricSurface(lambda u, v: (u, v, 1.0))
    radius = 2.0
    surface_radius = 1e9
    effective_radius = radius * surface_radius / (radius + surface_radius)
    step = adaptive_stepover(0.01, radius, surface_radius)
    assert scallop_height(step, effective_radius) == pytest.approx(0.01)
    assert sampled_surface_distance((0.5, 0.5, 2), flat_surface, 5) == pytest.approx(2)
    assert minimum_surface_clearance(
        [(0.5, 0.5, 2)], [flat_surface, upper_surface], 5,
    ) == pytest.approx(1)
    assert drop_cutter_height(0.5, 0.5, [flat_surface, upper_surface], 0.5, "ball", 5) == pytest.approx(1.5)
    with pytest.raises(AGEError):
        adaptive_stepover(0, 1, 1)
