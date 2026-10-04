"""Numerical multi-surface cutter compensation and toolpath utilities.

Surface distance and drop-cutter operations are sampled approximations. They
are useful for toolpath planning and visualization, not certified verification.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable, Sequence, Tuple

from .conversational import AGEError

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]
Matrix4 = Tuple[Tuple[float, float, float, float], ...]
_MAX_CHORDAL_POINTS = 100_000


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, value: float) -> Vec3:
    return (a[0] * value, a[1] * value, a[2] * value)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _unit(a: Vec3, name: str) -> Vec3:
    magnitude = math.sqrt(_dot(a, a))
    if magnitude <= 1e-12:
        raise AGEError(f"{name} must be nonzero")
    return _scale(a, 1 / magnitude)


@dataclass(frozen=True)
class ParametricSurface:
    """A surface S(u,v) defined on the unit square parameter domain."""

    evaluate: Callable[[float, float], Vec3]
    derivative_u: Callable[[float, float], Vec3] | None = None
    derivative_v: Callable[[float, float], Vec3] | None = None

    def point(self, u: float, v: float) -> Vec3:
        if not _finite((u, v)) or not (0 <= u <= 1 and 0 <= v <= 1):
            raise AGEError("Surface parameters must lie in [0, 1]")
        point = tuple(self.evaluate(u, v))
        if len(point) != 3 or not _finite(point):
            raise AGEError("Surface evaluator must return a finite XYZ point")
        return point  # type: ignore[return-value]


def surface_normal(surface: ParametricSurface, u: float, v: float,
                   epsilon: float = 1e-5) -> Vec3:
    """Compute a unit normal using supplied or bounded finite derivatives."""
    if not math.isfinite(epsilon) or epsilon <= 0:
        raise AGEError("Derivative epsilon must be positive and finite")

    def derivative(fn, coordinate, other, is_u):
        if fn is not None:
            value = tuple(fn(u, v))
            if len(value) != 3 or not _finite(value):
                raise AGEError("Surface derivative must be a finite XYZ vector")
            return value
        lo, hi = max(0.0, coordinate - epsilon), min(1.0, coordinate + epsilon)
        if hi - lo <= 1e-15:
            raise AGEError("Cannot estimate surface derivative at this parameter")
        a = surface.point(lo, other) if is_u else surface.point(other, lo)
        b = surface.point(hi, other) if is_u else surface.point(other, hi)
        return _scale(_sub(b, a), 1 / (hi - lo))

    du = derivative(surface.derivative_u, u, v, True)
    dv = derivative(surface.derivative_v, v, u, False)
    return _unit(_cross(du, dv), "Surface normal")


def cutter_location(contact: Vec3, normal: Vec3, radius: float,
                    cutter: str = "ball", tool_axis: Vec3 = (0, 0, 1),
                    minor_radius: float | None = None) -> Vec3:
    """Offset a contact point for ball, flat, or toroidal cutter geometry."""
    if len(contact) != 3 or not _finite(contact):
        raise AGEError("Contact point must be finite XYZ")
    if not math.isfinite(radius) or radius <= 0:
        raise AGEError("Cutter radius must be positive and finite")
    n = _unit(normal, "Surface normal")
    axis = _unit(tool_axis, "Tool axis")
    if cutter == "ball":
        offset = _scale(n, radius)
    elif cutter in {"flat", "toroid"}:
        planar = _sub(n, _scale(axis, _dot(n, axis)))
        planar_length = math.sqrt(_dot(planar, planar))
        if cutter == "flat":
            offset = (_scale(planar, radius / planar_length)
                      if planar_length > 1e-12 else (0.0, 0.0, 0.0))
        else:
            if (minor_radius is None or not math.isfinite(minor_radius) or
                    not 0 < minor_radius <= radius):
                raise AGEError("Toroid minor radius must be in (0, major radius]")
            if planar_length > 1e-12:
                planar = _scale(planar, 1 / planar_length)
            offset = _add(
                _scale(n, minor_radius),
                _scale(planar, radius - minor_radius),
            )
    else:
        raise AGEError("Cutter must be 'ball', 'flat', or 'toroid'")
    return _add(contact, offset)


@dataclass(frozen=True)
class WorkCoordinateTransform:
    """Validated affine CAD-WCS to machine-WCS transform."""

    matrix: Matrix4

    def __post_init__(self):
        if len(self.matrix) != 4 or any(len(row) != 4 for row in self.matrix):
            raise AGEError("WCS transform must be a 4x4 matrix")
        if not _finite(value for row in self.matrix for value in row):
            raise AGEError("WCS transform must contain finite values")
        if any(abs(self.matrix[3][i] - expected) > 1e-12
               for i, expected in enumerate((0, 0, 0, 1))):
            raise AGEError("WCS transform must be affine")

    def apply(self, point: Vec3) -> Vec3:
        if len(point) != 3 or not _finite(point):
            raise AGEError("WCS point must be finite XYZ")
        return tuple(
            sum(self.matrix[row][col] * point[col] for col in range(3))
            + self.matrix[row][3]
            for row in range(3)
        )  # type: ignore[return-value]

    @classmethod
    def identity(cls):
        return cls(((1, 0, 0, 0), (0, 1, 0, 0),
                    (0, 0, 1, 0), (0, 0, 0, 1)))


def scallop_height(stepover: float, effective_radius: float) -> float:
    """Approximate scallop height h²/(8R), for a positive effective radius."""
    if (not _finite((stepover, effective_radius)) or stepover < 0 or
            effective_radius <= 0):
        raise AGEError("Stepover must be nonnegative and radius positive")
    return stepover ** 2 / (8 * effective_radius)


def adaptive_stepover(target_height: float, cutter_radius: float,
                      surface_radius: float) -> float:
    """Calculate curvature-adjusted stepover for a requested scallop height."""
    if (not _finite((target_height, cutter_radius, surface_radius)) or
            target_height <= 0 or cutter_radius <= 0 or surface_radius <= 0):
        raise AGEError("Scallop target and radii must be positive and finite")
    effective = cutter_radius * surface_radius / (cutter_radius + surface_radius)
    return math.sqrt(8 * target_height * effective)


def sample_chordal(curve: Callable[[float], Vec3], tolerance: float,
                   max_depth: int = 20,
                   max_points: int = _MAX_CHORDAL_POINTS) -> list[Vec3]:
    """Subdivide a parametric curve until midpoint chord deviation is bounded."""
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise AGEError("Chord tolerance must be positive and finite")
    if not isinstance(max_depth, int) or max_depth < 1:
        raise AGEError("Maximum subdivision depth must be a positive integer")
    if not isinstance(max_points, int) or max_points < 2:
        raise AGEError("Maximum point count must be an integer >= 2")

    def evaluate(t):
        point = tuple(curve(t))
        if len(point) != 3 or not _finite(point):
            raise AGEError("Curve must return finite XYZ points")
        return point

    start, end = evaluate(0.0), evaluate(1.0)
    output = [start]

    def subdivide(t0, p0, t1, p1, depth):
        tm = (t0 + t1) / 2
        pm = evaluate(tm)
        chord_mid = _scale(_add(p0, p1), 0.5)
        deviation = math.sqrt(_dot(_sub(pm, chord_mid), _sub(pm, chord_mid)))
        if deviation <= tolerance:
            if len(output) >= max_points:
                raise AGEError("Chord sampling exceeds the maximum point count")
            output.append(p1)
            return
        if depth >= max_depth:
            raise AGEError("Chord tolerance not reached within max_depth")
        subdivide(t0, p0, tm, pm, depth + 1)
        subdivide(tm, pm, t1, p1, depth + 1)

    subdivide(0.0, start, 1.0, end, 0)
    return output


def surface_toolpath(surface: ParametricSurface,
                     uv_curve: Callable[[float], Vec2], tolerance: float,
                     cutter: str = "ball", radius: float = 1.0,
                     tool_axis: Vec3 = (0, 0, 1),
                     minor_radius: float | None = None,
                     transform: WorkCoordinateTransform | None = None,
                     max_depth: int = 20) -> list[Vec3]:
    """Sample a surface-following compensated path and transform to machine WCS."""
    def cutter_curve(t):
        uv = tuple(uv_curve(t))
        if len(uv) != 2 or not _finite(uv):
            raise AGEError("UV curve must return finite parameter pairs")
        u, v = uv
        contact = surface.point(u, v)
        normal = surface_normal(surface, u, v)
        return cutter_location(
            contact, normal, radius, cutter, tool_axis, minor_radius,
        )

    points = sample_chordal(cutter_curve, tolerance, max_depth)
    if transform is not None:
        points = [transform.apply(point) for point in points]
    return points


def sampled_surface_distance(point: Vec3, surface: ParametricSurface,
                             resolution: int = 25) -> float:
    """Approximate point-to-surface distance using a uniform UV grid."""
    if len(point) != 3 or not _finite(point):
        raise AGEError("Distance query point must be finite XYZ")
    if not isinstance(resolution, int) or resolution < 2:
        raise AGEError("Surface resolution must be an integer >= 2")
    best = math.inf
    for i in range(resolution):
        u = i / (resolution - 1)
        for j in range(resolution):
            v = j / (resolution - 1)
            candidate = surface.point(u, v)
            distance = math.sqrt(_dot(_sub(point, candidate), _sub(point, candidate)))
            best = min(best, distance)
    return best


def minimum_surface_clearance(tool_samples: Sequence[Vec3],
                              surfaces: Sequence[ParametricSurface],
                              resolution: int = 25) -> float:
    """Return sampled minimum Euclidean clearance from tool samples to surfaces."""
    if not tool_samples or not surfaces:
        raise AGEError("At least one tool sample and surface are required")
    return min(
        sampled_surface_distance(point, surface, resolution)
        for point in tool_samples for surface in surfaces
    )


def drop_cutter_height(x: float, y: float, surfaces: Sequence[ParametricSurface],
                       radius: float, cutter: str = "ball",
                       resolution: int = 25) -> float:
    """Approximate vertical-axis drop-cutter height across sampled surfaces."""
    if not _finite((x, y, radius)) or radius <= 0 or not surfaces:
        raise AGEError("XY, positive cutter radius, and surfaces are required")
    if not isinstance(resolution, int) or resolution < 2:
        raise AGEError("Surface resolution must be an integer >= 2")
    heights = []
    for surface in surfaces:
        for i in range(resolution):
            u = i / (resolution - 1)
            for j in range(resolution):
                v = j / (resolution - 1)
                px, py, pz = surface.point(u, v)
                distance = math.hypot(px - x, py - y)
                if distance > radius:
                    continue
                if cutter == "ball":
                    heights.append(pz + math.sqrt(max(0, radius ** 2 - distance ** 2)))
                elif cutter == "flat":
                    heights.append(pz)
                else:
                    raise AGEError("Drop-cutter type must be 'ball' or 'flat'")
    if not heights:
        raise AGEError("No surface samples lie within the cutter footprint")
    return max(heights)
