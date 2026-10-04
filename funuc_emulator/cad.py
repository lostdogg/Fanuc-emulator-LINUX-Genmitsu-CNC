"""Dependency-free reader for common planar ASCII DXF entities."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple, Union

Point = Tuple[float, float]


class CadError(ValueError):
    """Raised when a CAD file is malformed or cannot be interpreted."""


@dataclass(frozen=True)
class Line:
    start: Point
    end: Point


@dataclass(frozen=True)
class Arc:
    center: Point
    radius: float
    start_angle: float
    end_angle: float


@dataclass(frozen=True)
class Circle:
    center: Point
    radius: float


@dataclass(frozen=True)
class Polyline:
    points: Tuple[Point, ...]
    closed: bool = False


Entity = Union[Line, Arc, Circle, Polyline]


@dataclass(frozen=True)
class DxfDocument:
    entities: Tuple[Entity, ...]
    units: str
    unit_scale: float


@dataclass(frozen=True)
class HoleFeature:
    center: Point
    diameter: float


@dataclass(frozen=True)
class ProfileFeature:
    points: Tuple[Point, ...]


@dataclass(frozen=True)
class CadFeatures:
    holes: Tuple[HoleFeature, ...]
    profiles: Tuple[ProfileFeature, ...]


_UNITS = {
    0: ("unitless", 1.0),
    1: ("inches", 25.4),
    2: ("feet", 304.8),
    3: ("miles", 1609344.0),
    4: ("millimeters", 1.0),
    5: ("centimeters", 10.0),
    6: ("meters", 1000.0),
}


def parse_dxf(source: str) -> DxfDocument:
    """Read supported entities from the ENTITIES section of ASCII DXF.

    Coordinates are converted to millimeters when the file declares units.
    Supported entity records are LINE, ARC, CIRCLE, and LWPOLYLINE.
    """
    lines = source.splitlines()
    if len(lines) % 2:
        raise CadError("DXF must contain complete group-code/value pairs")
    pairs = []
    for i in range(0, len(lines), 2):
        try:
            code = int(lines[i].strip())
        except ValueError as exc:
            raise CadError(f"Invalid DXF group code on line {i + 1}") from exc
        pairs.append((code, lines[i + 1].strip()))

    unit_code = 0
    in_header = False
    for i, (code, value) in enumerate(pairs):
        if code == 0 and value == "SECTION":
            in_header = i + 1 < len(pairs) and pairs[i + 1] == (2, "HEADER")
        elif code == 0 and value == "ENDSEC":
            in_header = False
        elif in_header and code == 9 and value == "$INSUNITS":
            if i + 1 < len(pairs) and pairs[i + 1][0] == 70:
                try:
                    unit_code = int(pairs[i + 1][1])
                except ValueError as exc:
                    raise CadError("Invalid $INSUNITS value") from exc
    units, scale = _UNITS.get(unit_code, ("unitless", 1.0))

    entities: List[Entity] = []
    section = ""
    records: List[Tuple[int, str]] = []

    def consume(record: List[Tuple[int, str]]) -> None:
        if not record:
            return
        kind = record[0][1]
        if kind not in {"LINE", "ARC", "CIRCLE", "LWPOLYLINE"}:
            return
        data = record[1:]

        def one(code, default=None):
            values = [v for c, v in data if c == code]
            return values[0] if values else default

        def number(code, default=None):
            value = one(code)
            if value is None:
                if default is not None:
                    return default
                raise CadError(f"{kind} is missing required group {code}")
            try:
                value = float(value)
            except ValueError as exc:
                raise CadError(f"{kind} has invalid numeric group {code}") from exc
            if not math.isfinite(value):
                raise CadError(f"{kind} contains a non-finite coordinate")
            return value

        if kind == "LINE":
            entities.append(Line((number(10) * scale, number(20) * scale),
                                 (number(11) * scale, number(21) * scale)))
        elif kind == "CIRCLE":
            radius = number(40) * scale
            if radius <= 0:
                raise CadError("CIRCLE radius must be positive")
            entities.append(Circle((number(10) * scale, number(20) * scale),
                                   radius))
        elif kind == "ARC":
            radius = number(40) * scale
            if radius <= 0:
                raise CadError("ARC radius must be positive")
            entities.append(Arc(
                (number(10) * scale, number(20) * scale), radius,
                number(50), number(51),
            ))
        else:
            raw_points: List[Point] = []
            pending_x = None
            bulges = []
            for code, value in data:
                if code == 10:
                    if pending_x is not None:
                        raise CadError("LWPOLYLINE vertex is missing its Y coordinate")
                    try:
                        pending_x = float(value)
                    except ValueError as exc:
                        raise CadError("Invalid LWPOLYLINE X coordinate") from exc
                elif code == 20:
                    if pending_x is None:
                        raise CadError("LWPOLYLINE Y coordinate has no X coordinate")
                    try:
                        y = float(value)
                    except ValueError as exc:
                        raise CadError("Invalid LWPOLYLINE Y coordinate") from exc
                    raw_points.append((pending_x * scale, y * scale))
                    pending_x = None
                elif code == 42:
                    try:
                        bulges.append(float(value))
                    except ValueError as exc:
                        raise CadError("Invalid LWPOLYLINE bulge") from exc
            if pending_x is not None:
                raise CadError("LWPOLYLINE vertex is missing its Y coordinate")
            if len(raw_points) < 2:
                raise CadError("LWPOLYLINE must contain at least two vertices")
            try:
                flags = int(one(70, "0"))
            except ValueError as exc:
                raise CadError("Invalid LWPOLYLINE flags") from exc
            if any(abs(bulge) > 1e-12 for bulge in bulges):
                raise CadError("Bulged LWPOLYLINE segments are not supported")
            entities.append(Polyline(tuple(raw_points), bool(flags & 1)))

    for code, value in pairs:
        if code == 0 and value == "SECTION":
            section = ""
            records = []
        elif code == 2 and not section:
            section = value
        elif code == 0 and value == "ENDSEC":
            if section == "ENTITIES":
                consume(records)
            section = ""
            records = []
        elif section == "ENTITIES":
            if code == 0:
                consume(records)
                records = [(code, value)]
            else:
                records.append((code, value))
    if section == "ENTITIES":
        consume(records)
    if not entities:
        raise CadError("No supported CAD entities found in DXF")
    return DxfDocument(tuple(entities), units, scale)


def extract_features(document: DxfDocument, tolerance: float = 0.001) -> CadFeatures:
    """Recognize circular holes and closed polylines/line chains."""
    if tolerance <= 0:
        raise CadError("Chaining tolerance must be positive")
    holes = [HoleFeature(entity.center, entity.radius * 2)
             for entity in document.entities if isinstance(entity, Circle)]
    profiles: List[ProfileFeature] = [
        ProfileFeature(entity.points)
        for entity in document.entities
        if isinstance(entity, Polyline) and entity.closed
    ]
    line_edges = [(entity.start, entity.end)
                  for entity in document.entities if isinstance(entity, Line)]
    profiles.extend(_closed_line_chains(line_edges, tolerance))
    return CadFeatures(tuple(holes), tuple(profiles))


def _closed_line_chains(edges: List[Tuple[Point, Point]],
                        tolerance: float) -> List[ProfileFeature]:
    remaining = set(range(len(edges)))
    profiles = []
    while remaining:
        first = min(remaining)
        start, end = edges[first]
        chain = [start, end]
        used = {first}
        remaining.remove(first)
        while math.dist(chain[-1], chain[0]) > tolerance:
            matches = []
            for index in remaining:
                a, b = edges[index]
                if math.dist(chain[-1], a) <= tolerance:
                    matches.append((index, b))
                elif math.dist(chain[-1], b) <= tolerance:
                    matches.append((index, a))
            if len(matches) != 1:
                break
            index, next_point = matches[0]
            used.add(index)
            remaining.remove(index)
            chain.append(next_point)
        if math.dist(chain[-1], chain[0]) <= tolerance and len(chain) >= 4:
            chain[-1] = chain[0]
            profiles.append(ProfileFeature(tuple(chain[:-1])))
        else:
            # Do not accidentally combine an open/branching chain with a
            # different profile on a later pass.
            remaining.difference_update(used)
    return profiles


def features_to_gcode(features: CadFeatures, depth: float, feed: float,
                      safe_z: float = 5.0) -> List[str]:
    """Convert extracted holes and closed boundaries to basic milling G-code."""
    if depth >= 0 or feed <= 0 or safe_z <= 0:
        raise CadError("Depth must be negative; feed and safe Z must be positive")
    from . import conversational

    out = ["G21 G90 G94"]
    for hole in features.holes:
        out.extend(conversational.drill(
            hole.center[0], hole.center[1], depth, safe_z, feed,
        ))
    for profile in features.profiles:
        points = profile.points
        if len(points) < 3:
            continue
        path = [("line", x, y) for x, y in points[1:]]
        path.append(("line", points[0][0], points[0][1]))
        out.extend(conversational.profile(
            points[0], path, depth, feed=feed, safe_z=safe_z,
        ))
    return out
